# Combining the measurements over time.
#
# e, ψe and κ (the curvature beside the car) are the state of an extended
# Kalman filter. Between scans it predicts how they change from the car's
# speed v and yaw rate r:
#
#   de/dt  = v⋅sin(ψe) − d⋅r⋅cos(ψe)   (d: the front axle's distance ahead of
#                                         the rear axle, which moves sideways
#                                         at d⋅r)
#   dψe/dt = κ⋅v − r
#   dκ/dt  = 0                          (changes count as noise)
#
# and each measurement corrects it, weighted by the network's own standard
# deviation. A measurement counts at the time of its scan: the filter keeps
# the last few scans' priors and measurements, so a late result from the
# cloud is put in its place (replacing the small network's result for
# that scan), and the scans after it are filtered again.
#
# The profiles (κ_profile, the widths) stay put on the ground while the car
# drives past them: each is shifted by the distance driven, then moved part
# of the way toward each new measurement. They are replayed the same way as
# the filter: a late measurement is blended in at its own scan (and replaces
# the small network's result for it), then shifted forward with the car.
# `sight` and `clearance` are the latest measurement, less the distance
# driven since its scan.

"""
    TrackerConfig(; kwargs...)

Tuning of the [`LaneTracker`](@ref).
"""
Base.@kwdef struct TrackerConfig
    # What the motion model can't explain, as rates: sideways speed [m/s],
    # heading rate [rad/s], change of curvature per meter driven [1/m²].
    q_e::Float64 = 0.5
    q_ψ::Float64 = 0.85
    q_κ::Float64 = 0.8
    # A measurement more than `gate` standard deviations from the prediction
    # is an outlier; after `max_outliers` scans of outliers in a row the
    # filter restarts from the measurements.
    gate::Float64 = 4.0
    max_outliers::Int = 3
    # Weight of each new measurement in a profile, and the change above which
    # it is taken as it is (a turn coming into view).
    profile_gain::Float64 = 0.25
    κ_jump::Float64 = 0.15
    width_gain::Float64 = 0.5
    width_jump::Float64 = 0.3
    history::Int = 10        # scans kept for late measurements
    lost_after::Int = 10     # scans without e or ψe before the lane is lost
end

"""
    LaneState

The tracked lane, at the front axle, now:

- `e`, `ψe`, `κ`: the filter's cross-track error [m], heading error [rad]
  and curvature beside the car [1/m] (see [`ESTIMATES`](@ref)).
- `κ_profile`, `w_left`, `w_right`: the profiles at `PROFILE_S`; NaN where
  nothing is known.
- `sight`, `clearance` (at `CLEARANCE_K`), and their standard deviations.
- `tiers`: the tier each estimate's latest measurement came from (`:none`
  before the first).
- `status`: `:ok` (e and ψe measured for this scan), `:predicting` (the
  newest scan with them is at most `lost_after` scans old: late cloud
  results, say) or `:lost`.
"""
struct LaneState
    e::Float64
    ψe::Float64
    κ::Float64
    κ_profile::Vector{Float64}
    w_left::Vector{Float64}
    w_right::Vector{Float64}
    sight::Float64
    σ_sight::Float64
    clearance::Vector{Float64}
    σ_clearance::Vector{Float64}
    tiers::NamedTuple
    status::Symbol
end

mutable struct _Step
    t::Float64
    v::Float64                # speed and yaw rate used to predict into this step
    r::Float64
    odo::Float64              # distance driven up to this step [m]
    x::Vector{Float64}        # the prior at this step
    P::Matrix{Float64}
    z::Dict{Symbol,Tuple{Float64,Float64,Symbol}}   # :e, :ψe, :κ => (value, variance, tier)
    profiles::NTuple{3,Vector{Float64}}             # κ_profile, w_left, w_right before this step's measurements
    pz::Dict{Symbol,Tuple{Vector{Float64},Symbol}}  # :κ_profile, :widths => (value, tier)
    start::Bool               # the filter (re)started from this scan's e and ψe
end

_Step(t, v, r, odo, x, P, profiles) = _Step(t, v, r, odo, x, P, Dict{Symbol,Tuple{Float64,Float64,Symbol}}(),
    profiles, Dict{Symbol,Tuple{Vector{Float64},Symbol}}(), false)

# Whether a measurement from `tier` may replace one from `old` for the same scan.
_replaces(old, tier) = old === nothing || !(old === :large && tier === :small)

mutable struct _Latest
    value::Vector{Float64}
    σ::Vector{Float64}
    scan_time::Float64
    odo::Float64
    tier::Symbol
end

"""
    LaneTracker(ref_offset; cfg=TrackerConfig())

Tracks the lane from the measurements. `ref_offset` is the front axle's
distance ahead of the rear axle [m]. Feed it every scan with
[`track!`](@ref).
"""
mutable struct LaneTracker
    cfg::TrackerConfig
    ref_offset::Float64
    steps::Vector{_Step}
    x::Vector{Float64}
    P::Matrix{Float64}
    ready::Bool
    outliers::Int
    unmeasured::Int           # scans since e or ψe was last measured
    κ_profile::Vector{Float64}
    w_left::Vector{Float64}
    w_right::Vector{Float64}
    sight::Union{Nothing,_Latest}
    clearance::Union{Nothing,_Latest}
    tiers::Dict{Symbol,Symbol}
end

LaneTracker(ref_offset::Real; cfg::TrackerConfig=TrackerConfig()) =
    LaneTracker(cfg, ref_offset, _Step[], zeros(3), zeros(3, 3), false, 0, typemax(Int) ÷ 2,
        fill(NaN, _NP), fill(NaN, _NP), fill(NaN, _NP), nothing, nothing, Dict(est => :none for est in ESTIMATES))

# Restart from the measurements of scan k, forgetting the scans before it (a
# late measurement must not be filtered from a state the restart replaced).
# The scans after k are then filtered again from it.
function _restart_at!(tr::LaneTracker, k)
    step = tr.steps[k]
    _restart!(tr, step.z)
    step.x, step.P = copy(tr.x), copy(tr.P)
    empty!(step.z)
    step.start = true
    deleteat!(tr.steps, 1:k-1)
    return tr
end

_measured(s::_Step) = s.start || (haskey(s.z, :e) && haskey(s.z, :ψe))

_wrap(a) = rem2pi(Float64(a), RoundNearest)

function _predict(c::TrackerConfig, d, x, P, v, r, dt)
    e, ψ, κ = x
    xn = [e + (v * sin(ψ) - d * r * cos(ψ)) * dt, ψ + (κ * v - r) * dt, κ]
    F = [1.0 (v*cos(ψ)+d*r*sin(ψ))*dt 0.0
         0.0 1.0 v*dt
         0.0 0.0 1.0]
    Q = Diagonal([(c.q_e * dt)^2, (c.q_ψ * dt)^2, (c.q_κ * abs(v) * dt)^2])
    return xn, F * P * F' + Q
end

const _COMPONENTS = (:e, :ψe, :κ)

# Correct x, P with one step's measurements; returns how many were outliers.
function _correct!(c::TrackerConfig, x, P, z)
    rejected = 0
    for (i, comp) in enumerate(_COMPONENTS)
        haskey(z, comp) || continue
        val, R, _ = z[comp]
        comp === :ψe && (val = x[2] + _wrap(val - x[2]))   # unwrap near the prediction
        y = val - x[i]
        S = P[i, i] + R
        if y^2 > c.gate^2 * S
            rejected += 1
            continue
        end
        K = P[:, i] / S
        x .+= K * y
        P .-= K * P[i:i, :]
    end
    return rejected
end

function _restart!(tr::LaneTracker, z)
    tr.x = [haskey(z, c) ? z[c][1] : tr.x[i] for (i, c) in enumerate(_COMPONENTS)]
    tr.P = Matrix(Diagonal([haskey(z, c) ? z[c][2] : 1.0 for c in _COMPONENTS]))
    tr.outliers = 0
    tr.ready = true
    return tr
end

# A profile sampled every PROFILE_STEP meters, moved d meters toward the car
# (linear interpolation; NaN where nothing was known).
function _shift(profile::AbstractVector{Float64}, d::Real)
    n = length(profile)
    out = fill(NaN, n)
    for j in 1:n
        u = j + d / PROFILE_STEP   # position in the old profile
        (u > n || u < 1) && continue
        i = floor(Int, u)
        a, b = profile[i], profile[min(i + 1, n)]
        out[j] = isnan(a) ? b : isnan(b) ? a : a + (u - i) * (b - a)
    end
    return out
end

function _blend!(p, z, gain, jump)
    for j in eachindex(p)
        isnan(z[j]) && continue
        p[j] = isnan(p[j]) || abs(z[j] - p[j]) > jump ? z[j] : p[j] + gain * (z[j] - p[j])
    end
    return p
end

_find_step(tr::LaneTracker, t) = findlast(s -> abs(s.t - t) < 1e-6, tr.steps)

"""
    track!(tr, measurements, v, yaw_rate, t) -> LaneState

Add the measurements given at time `t` [s] (see [`observe!`](@ref)), with
the car's speed `v` [m/s] and yaw rate [rad/s], and return the lane now.
"""
function track!(tr::LaneTracker, meas::AbstractVector{Measurement}, v::Real, yaw_rate::Real, t::Real)
    c, t, v, r = tr.cfg, Float64(t), Float64(v), Float64(yaw_rate)

    # a new step, predicted from the last
    if isempty(tr.steps)
        push!(tr.steps, _Step(t, v, r, 0.0, copy(tr.x), copy(tr.P), (copy(tr.κ_profile), copy(tr.w_left), copy(tr.w_right))))
    else
        last = tr.steps[end]
        dt = t - last.t
        x, P = tr.ready ? _predict(c, tr.ref_offset, tr.x, tr.P, v, r, dt) : (copy(tr.x), copy(tr.P))
        profiles = (_shift(tr.κ_profile, v * dt), _shift(tr.w_left, v * dt), _shift(tr.w_right, v * dt))
        push!(tr.steps, _Step(t, v, r, last.odo + v * dt, x, P, profiles))
        length(tr.steps) > c.history && popfirst!(tr.steps)
    end
    now = tr.steps[end]

    # file the measurements under their scans; the large tier's replace the small's
    first_changed = length(tr.steps)
    for m in sort(meas; by=m -> m.scan_time)
        k = _find_step(tr, m.scan_time)
        k === nothing && continue   # older than the history
        step = tr.steps[k]
        if m.estimate === :e || m.estimate === :ψe || m.estimate === :κ_profile
            comp = m.estimate === :κ_profile ? :κ : m.estimate
            old = get(step.z, comp, nothing)
            if _replaces(old === nothing ? nothing : old[3], m.tier)
                step.z[comp] = (m.value[1], m.σ[1]^2, m.tier)
                first_changed = min(first_changed, k)
            end
        end
        if m.estimate === :κ_profile || m.estimate === :widths
            old = get(step.pz, m.estimate, nothing)
            if _replaces(old === nothing ? nothing : old[2], m.tier)
                step.pz[m.estimate] = (m.value, m.tier)
                first_changed = min(first_changed, k)
            end
        elseif m.estimate === :sight || m.estimate === :clearance
            old = getfield(tr, m.estimate)
            if old === nothing || m.scan_time > old.scan_time || (m.scan_time == old.scan_time && m.tier === :large)
                setfield!(tr, m.estimate, _Latest(m.value, m.σ, m.scan_time, step.odo, m.tier))
            end
        end
        tr.tiers[m.estimate] = m.tier
    end

    # the profiles again from the earliest scan that changed
    κp, wl, wr = copy.(tr.steps[first_changed].profiles)
    for k in first_changed:length(tr.steps)
        s = tr.steps[k]
        if k > first_changed
            d = s.odo - tr.steps[k-1].odo
            κp, wl, wr = _shift(κp, d), _shift(wl, d), _shift(wr, d)
            s.profiles = (copy(κp), copy(wl), copy(wr))
        end
        if haskey(s.pz, :κ_profile)
            _blend!(κp, s.pz[:κ_profile][1], c.profile_gain, c.κ_jump)
        end
        if haskey(s.pz, :widths)
            w = s.pz[:widths][1]
            _blend!(wl, w_left(w), c.width_gain, c.width_jump)
            _blend!(wr, w_right(w), c.width_gain, c.width_jump)
        end
    end
    tr.κ_profile, tr.w_left, tr.w_right = κp, wl, wr

    # start from the earliest scan with e and ψe (they may have come late)
    if !tr.ready
        k = findfirst(s -> haskey(s.z, :e) && haskey(s.z, :ψe), tr.steps)
        if k !== nothing
            _restart_at!(tr, k)
            first_changed = 1
        end
    end
    # filter again from the earliest scan that changed
    if tr.ready
        x, P = copy(tr.steps[first_changed].x), copy(tr.steps[first_changed].P)
        rejected = 0
        for k in first_changed:length(tr.steps)
            s = tr.steps[k]
            if k > first_changed
                x, P = _predict(c, tr.ref_offset, x, P, s.v, s.r, s.t - tr.steps[k-1].t)
                s.x, s.P = copy(x), copy(P)
            end
            rejected = _correct!(c, x, P, s.z)
        end
        tr.x, tr.P = x, P
        # outliers in the current scan's e and ψe, scan after scan: the lane changed
        if haskey(now.z, :e) && haskey(now.z, :ψe) && rejected > 0
            tr.outliers += 1
            tr.outliers >= c.max_outliers && _restart_at!(tr, length(tr.steps))
        else
            tr.outliers = 0
        end
    end
    # scans since the newest one with e and ψe (late ones count at their scan)
    k = findlast(_measured, tr.steps)
    tr.unmeasured = k === nothing ? tr.unmeasured + 1 : length(tr.steps) - k

    status = !tr.ready ? :lost : tr.unmeasured == 0 ? :ok : tr.unmeasured <= c.lost_after ? :predicting : :lost
    driven(l) = l === nothing ? 0.0 : now.odo - l.odo
    sight = tr.sight === nothing ? 0.0 : max(tr.sight.value[1] - driven(tr.sight), 0.0)
    σ_sight = tr.sight === nothing ? Inf : tr.sight.σ[1]
    nk = length(CLEARANCE_K)
    clearance = tr.clearance === nothing ? zeros(nk) : max.(tr.clearance.value .- driven(tr.clearance), 0.0)
    σ_clearance = tr.clearance === nothing ? fill(Inf, nk) : copy(tr.clearance.σ)
    κp = copy(tr.κ_profile)
    tr.ready && (κp[1] = tr.x[3])
    return LaneState(tr.x[1], _wrap(tr.x[2]), tr.x[3], κp, copy(tr.w_left), copy(tr.w_right), sight, σ_sight,
        clearance, σ_clearance, NamedTuple{ESTIMATES}(Tuple(tr.tiers[est] for est in ESTIMATES)), status)
end

"""
    LaneState(labels::LaneLabels; tier=:truth)

A lane state that is exactly `labels` (with zero standard deviations), for
driving on known estimates: the true ones in a simulator, say.
"""
function LaneState(l::LaneLabels; tier::Symbol=:truth)
    nk = length(CLEARANCE_K)
    return LaneState(l.e[1], l.ψe[1], l.κ_profile[1], copy(l.κ_profile), w_left(l.widths), w_right(l.widths),
        l.sight[1], 0.0, copy(l.clearance), zeros(nk), NamedTuple{ESTIMATES}(ntuple(_ -> tier, 6)), :ok)
end
