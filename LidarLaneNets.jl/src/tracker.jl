# Combining the car's and the cloud's measurements over time.
#
# The measurement model. Both networks measure the same quantities, but their
# errors differ in kind:
#
#   car (small network):  z = x + b + v,   v ~ white, b the car network's bias
#   cloud (large network): z = x + w,      w ~ white
#
# The car network's error is largely the same from one scan to the next (the
# scene barely changes in 25 ms), so a part of it, b, persists: a first-order
# Gauss-Markov process per measured number, with stationary standard
# deviation σ_bias and correlation time τ. The rest, and the cloud's error,
# change from scan to scan; their variances are the networks' own σ² (the
# car's less the persisting share). A cloud result for a scan thus measures
# x, the car's result for the same scan x + b, and together they reveal the
# car network's current bias, which the filter then takes off the car's
# following results until the bias has drifted (over about τ). These
# parameters are measured on the validation tracks (see `load_noise`).
#
# The errors are long-tailed (Student t), so each scalar update is robust: the
# measurement's variance is inflated by (ν + y²/S)/(ν + 1) when its
# innovation y is large against its variance S (one step of the variational
# Student-t filter), ν being that network's tail weight. Except where both
# tiers measured the same number at the same scan: there a large difference
# is evidence of the car's bias (it, too, is long-tailed), and the bias's
# variance is raised to match the difference instead (adaptive covariance
# inflation), so the bias can jump to what the cloud shows.
#
# The state:
# - e, ψe, κ (the curvature beside the car) and the car network's biases on
#   them, in an unscented Kalman filter. Between scans the motion, from the
#   car's speed v and yaw rate r:
#       de/dt = v⋅sin(ψe) − d⋅r⋅cos(ψe)   (d: the front axle's distance ahead
#                                           of the rear axle)
#       dψe/dt = κ⋅v − r,   dκ/dt = 0 (changes count as noise)
# - every entry of κ_profile, w_left, w_right, sight and clearance: the car's
#   latest measurement less its bias (a Kalman filter on the bias alone, fed
#   by the scans both tiers measured), or the cloud's latest where the car has
#   none; see `_Cells` below.
#
# Out of sequence: a result counts at the time of its scan. The tracker keeps
# the last few scans' priors and measurements; a late cloud result is added to
# its scan and the scans after it are filtered again (the exact solution, by
# reprocessing).

"""
    EstimateNoise(; persist, τ, σ_bias, ν_small, ν_large)

The noise of one estimate's measurements: the share of the car network's
error variance that persists from scan to scan (`persist`), how long it
persists (`τ` [s]), its standard deviation (`σ_bias`, in the estimate's
units), and the tail weights (Student t degrees of freedom) of the car's
and the cloud's errors over their own σ (`Inf`: Gaussian).
"""
Base.@kwdef struct EstimateNoise
    persist::Float64 = 0.5
    τ::Float64 = 0.25
    σ_bias::Float64
    ν_small::Float64 = Inf
    ν_large::Float64 = Inf
end

"""
    default_noise()

Rough noise parameters for every estimate, for when none have been measured
(see [`load_noise`](@ref)).
"""
default_noise() = (e=EstimateNoise(σ_bias=0.005), ψe=EstimateNoise(σ_bias=0.01),
    κ_profile=EstimateNoise(σ_bias=0.03), widths=EstimateNoise(σ_bias=0.05), sight=EstimateNoise(σ_bias=0.1),
    clearance=EstimateNoise(σ_bias=0.15))

"""
    load_noise(dir; small=:small_weak, large=:large) -> NamedTuple

The noise parameters measured for the network sets `small` (the car's) and
`large` (the cloud's), from `dir/fusion.toml` (written by
`train/fusion.jl`), as `(e=EstimateNoise(…), …)`.
"""
function load_noise(dir::AbstractString; small::Symbol=:small_weak, large::Symbol=:large)
    d = TOML.parsefile(joinpath(dir, "fusion.toml"))
    get1(est) = (s = d[String(small)][file_name(est)];
        EstimateNoise(persist=s["persist"], τ=s["tau"], σ_bias=s["sigma_bias"], ν_small=s["nu"],
            ν_large=d[String(large)][file_name(est)]["nu"]))
    return NamedTuple{ESTIMATES}(Tuple(get1(est) for est in ESTIMATES))
end

"""
    TrackerConfig(; kwargs...)

Tuning of the [`LaneTracker`](@ref).
"""
Base.@kwdef struct TrackerConfig
    # What the motion model can't explain: e and ψe as random walks, their
    # variance growing by q_e² [m²/s] and q_ψ² [rad²/s] per second (tuned
    # at 40 Hz as 0.5 m/s and 0.85 rad/s per scan); κ by (q_κ ⋅ the distance
    # driven in the step)² [q_κ: 1/m²], as tuned per scan at 40 Hz.
    q_e::Float64 = 0.079
    q_ψ::Float64 = 0.134
    q_κ::Float64 = 0.8
    # How much a measurement goes stale per meter driven since its scan
    # (standard deviation): a profile's curvature [1/m] and widths [m] (on
    # the ground), sight and clearance [m] (from the car; the true ones change
    # by about 0.9 and 1.1 per meter over 100 ms, on the validation tracks).
    q_profile::Float64 = 0.05
    q_width::Float64 = 0.02
    q_sight::Float64 = 0.9
    q_clearance::Float64 = 1.2
    # e or ψe more than `gate` standard deviations from the prediction,
    # `max_outliers` scans in a row: the lane changed, and the filter restarts
    # from the measurements.
    gate::Float64 = 6.0
    max_outliers::Int = 3
    noise::NamedTuple{ESTIMATES,NTuple{6,EstimateNoise}} = default_noise()
    robust::Bool = true      # Student-t updates (else Gaussian)
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
- `tiers`: the tier of each estimate's latest measurement (`:none` before
  the first).
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

# --- every other number: the latest measurements, the car's bias taken off ---
#
# For the profiles, sight and clearance there is no good motion model (a
# corner changes them within a few scans), so they are not filtered states.
# Each entry keeps the car's and the cloud's latest measurements, and a
# Kalman filter on the car network's bias b alone: every scan both tiers
# measured gives an observation of it, z_car − z_cloud (the two errors' shared
# part cancels), and between scans it decays as a Gauss-Markov process. The
# estimate now is the car's latest measurement less b; only without one, the
# cloud's latest (the car's, less its bias, already holds the cloud's
# information: averaging both would count it twice). Each is aged (and, for
# the profiles, which stay put on the ground, shifted) by the distance driven
# since its scan.

mutable struct _Cells
    b::Vector{Float64}        # the car network's bias, per entry (in the car's view)
    Pbb::Vector{Float64}
    zs::Vector{Float64}       # the car's latest measurement (NaN: none), its white variance,
    Rs::Vector{Float64}       # and the distance driven up to its scan
    odo_s::Vector{Float64}
    zl::Vector{Float64}       # the cloud's latest, likewise
    Rl::Vector{Float64}
    odo_l::Vector{Float64}
end

_Cells(n, σ_bias) = _Cells(zeros(n), fill(σ_bias^2, n), fill(NaN, n), zeros(n), zeros(n), fill(NaN, n), zeros(n),
    zeros(n))
Base.copy(c::_Cells) = _Cells(copy(c.b), copy(c.Pbb), copy(c.zs), copy(c.Rs), copy(c.odo_s), copy(c.zl), copy(c.Rl),
    copy(c.odo_l))

# Between scans (dt seconds) the car network's bias decays toward zero.
function _predict!(c::_Cells, dt, n::EstimateNoise)
    a = exp(-dt / n.τ)
    c.b .*= a
    c.Pbb .= a^2 .* c.Pbb .+ n.σ_bias^2 * (1 - a^2)
    return c
end

# A residual z_car − z_cloud of one scan (variance R): an observation of the
# bias. One larger than expected is evidence of a larger bias: the bias's
# variance grows to match it first.
function _bias_update!(c::_Cells, i, y, R)
    isfinite(y) || return c
    inn = y - c.b[i]
    excess = inn^2 - (c.Pbb[i] + R)
    excess > 0 && (c.Pbb[i] += excess)
    K = c.Pbb[i] / (c.Pbb[i] + R)
    c.b[i] += K * inn
    c.Pbb[i] *= 1 - K
    return c
end

# The robust (Student t) variance of a measurement: inflated for a large
# innovation y against its variance S, under ν degrees of freedom (never
# shrunk below R).
_robust_R(R, y, S, ν, robust) = robust && isfinite(ν) ? R * max((ν + y^2 / S) / (ν + 1), 1.0) : R

# Interpolate entries of v moved d meters toward the car; `fillv` past the end.
function _shift_values(v::Vector{Float64}, d, fillv)
    n = length(v)
    out = fill(fillv, n)
    for j in 1:n
        u = j + d / PROFILE_STEP
        (u > n || u < 1) && continue
        i = floor(Int, u)
        f = u - i
        out[j] = i == n ? v[n] : (1 - f) * v[i] + f * v[i+1]
    end
    return out
end

# The estimate now (distance driven `odo`), and its variance, of every entry.
function _estimates(c::_Cells, odo, q; profile::Bool)
    n = length(c.b)
    car_z, car_v = fill(NaN, n), fill(Inf, n)
    cld_z, cld_v = fill(NaN, n), fill(Inf, n)
    for (z, R, o, bias, outz, outv) in ((c.zs, c.Rs, c.odo_s, true, car_z, car_v), (c.zl, c.Rl, c.odo_l, false, cld_z, cld_v))
        for i in 1:n
            isfinite(z[i]) || continue
            outz[i] = z[i] - (bias ? c.b[i] : 0.0)
            outv[i] = R[i] + (bias ? c.Pbb[i] : 0.0)
        end
        # aged by the distance driven since the scan (the oldest entry's, for a profile)
        if any(isfinite, outz)
            d = odo - minimum(o[i] for i in 1:n if isfinite(z[i]))
            if profile && d > 0
                known = _shift_values([isfinite(x) ? 1.0 : 0.0 for x in outz], d, 0.0)
                outz .= _shift_values(replace(outz, NaN => 0.0), d, NaN)
                outv .= _shift_values(replace(outv, Inf => 1e12), d, Inf)
                for i in 1:n
                    known[i] < 0.999 && (outz[i] = NaN; outv[i] = Inf)
                end
            end
            outv .+= (q * d)^2
        end
    end
    x, v = fill(NaN, n), fill(Inf, n)
    for i in 1:n
        wc = isfinite(car_z[i]) ? 1 / car_v[i] : 0.0
        # the cloud's own value only without the car's (whose bias it already corrected)
        wl = isfinite(cld_z[i]) && wc == 0 ? 1 / cld_v[i] : 0.0
        wc + wl > 0 || continue
        x[i] = ((wc > 0 ? wc * car_z[i] : 0.0) + (wl > 0 ? wl * cld_z[i] : 0.0)) / (wc + wl)
        v[i] = 1 / (wc + wl)
    end
    return x, v
end

# --- e, ψe, κ and the car's biases on them: an unscented Kalman filter ---

const _LANE = (:e, :ψe, :κ)

function _lane_motion(x, v, r, dt, d, a)
    e, ψ, κ = x[1], x[2], x[3]
    return [e + (v * sin(ψ) - d * r * cos(ψ)) * dt, ψ + (κ * v - r) * dt, κ, a[1] * x[4], a[2] * x[5], a[3] * x[6]]
end

# Unscented transform (α = 1, β = 2, κ = 0) of x, P through f, plus Q: the
# 2n + 1 sigma points x and x ± the columns of √(nP), weighted 0 (mean) and
# 2 (covariance) at x and 1/2n each elsewhere.
function _ukf_predict(x, P, f, Q)
    n = length(x)
    L = cholesky(Symmetric(n * P + 1e-12I)).L
    Y = Matrix{Float64}(undef, n, 2n + 1)
    Y[:, 1] = f(x)
    for i in 1:n
        Y[:, 1+i] = f(x .+ view(L, :, i))
        Y[:, 1+n+i] = f(x .- view(L, :, i))
    end
    w = 1 / (2n)
    m = w .* vec(sum(view(Y, :, 2:2n+1); dims=2))
    D = Y .- m
    D1, Dr = view(D, :, 1), view(D, :, 2:2n+1)
    Pn = 2.0 .* (D1 * D1') .+ w .* (Dr * Dr') .+ Q
    return m, (Pn + Pn') / 2
end

mutable struct _Lane
    ready::Bool
    x::Vector{Float64}       # e, ψe, κ, then the car's biases on them
    P::Matrix{Float64}
end

Base.copy(l::_Lane) = _Lane(l.ready, copy(l.x), copy(l.P))

_lane_noise(c::TrackerConfig) = (c.noise.e, c.noise.ψe, c.noise.κ_profile)

function _predict!(l::_Lane, c::TrackerConfig, ref, v, r, dt)
    l.ready || return l
    ns = _lane_noise(c)
    a = map(n -> exp(-dt / n.τ), ns)
    q = map((n, ak) -> n.σ_bias^2 * (1 - ak^2), ns, a)
    Q = Diagonal([c.q_e^2 * dt, c.q_ψ^2 * dt, (c.q_κ * abs(v) * dt)^2, q[1], q[2], q[3]])
    l.x, l.P = _ukf_predict(l.x, l.P, x -> _lane_motion(x, v, r, dt, ref, a), Q)
    return l
end

# Robust scalar update of component k with z = x_k (+ b_k for the car).
function _update!(l::_Lane, k, z, R, ν, bias::Bool, robust::Bool)
    x, P = l.x, l.P
    pred = bias ? x[k] + x[3+k] : x[k]
    k == 2 && (z = pred + rem2pi(z - pred, RoundNearest))   # unwrap near the prediction
    y = z - pred
    PH = bias ? P[:, k] .+ P[:, 3+k] : P[:, k]   # P h
    S0 = bias ? PH[k] + PH[3+k] : PH[k]          # h' P h
    Rr = _robust_R(R, y, S0 + R, ν, robust)
    g = 1 / (S0 + Rr)
    for j in axes(P, 2)
        x[j] += g * PH[j] * y
        for i in axes(P, 1)
            P[i, j] -= g * (PH[i] * PH[j])   # exactly symmetric
        end
    end
    return y^2 / (S0 + R)   # against the variance expected before down-weighting
end

# Start the lane from a scan's e, ψe (and κ, if given) measurements.
function _start!(l::_Lane, c::TrackerConfig, ms)
    ns = _lane_noise(c)
    x, P = zeros(6), zeros(6, 6)
    for (k, comp) in enumerate(_LANE)
        m = get(ms, comp, nothing)
        P[3+k, 3+k] = ns[k].σ_bias^2
        if m === nothing
            P[k, k] = comp === :κ ? 1.0 : 10.0
        else
            x[k] = m.z
            P[k, k] = m.R + (m.bias ? ns[k].σ_bias^2 : 0.0)
            m.bias && (P[k, 3+k] = P[3+k, k] = -ns[k].σ_bias^2)
        end
    end
    l.ready, l.x, l.P = true, x, P
    return l
end

# --- the whole state, and its history for late measurements ---

const _CELLS = (:κ_profile, :w_left, :w_right, :sight, :clearance)

const _CellSet = NamedTuple{_CELLS,NTuple{5,_Cells}}

mutable struct _State
    lane::_Lane
    cells::_CellSet
    odo::Float64              # distance driven [m]
end

Base.copy(s::_State) = _State(copy(s.lane), map(copy, s.cells), s.odo)

# The estimate a cell belongs to, and how many entries it has.
_cell_estimate(k::Symbol) = k === :w_left || k === :w_right ? :widths : k
_cell_size(k::Symbol) = k === :w_left || k === :w_right ? _NP : output_size(k)
_cell_noise(c::TrackerConfig, k) = c.noise[_cell_estimate(k)]

# The cell, and the entry in it, that entry j of estimate `est` goes to.
_cell(est::Symbol, j::Integer) = est === :widths ? (j <= _NP ? (:w_left, j) : (:w_right, j - _NP)) : (est, j)

_State(c::TrackerConfig) = _State(_Lane(false, zeros(6), zeros(6, 6)),
    _CellSet(Tuple(_Cells(_cell_size(k), _cell_noise(c, k).σ_bias) for k in _CELLS)), 0.0)

function _predict!(s::_State, c::TrackerConfig, ref, v, r, dt)
    s.odo += abs(v) * dt
    _predict!(s.lane, c, ref, v, r, dt)
    for k in _CELLS
        _predict!(s.cells[k], dt, _cell_noise(c, k))
    end
    return s
end

_q(c::TrackerConfig, k) = k === :κ_profile ? c.q_profile : k === :sight ? c.q_sight : k === :clearance ? c.q_clearance :
                         c.q_width

mutable struct _Step
    t::Float64
    v::Float64                # speed and yaw rate used to predict into this step
    r::Float64
    prior::_State             # before this step's measurements
    meas::Matrix{Union{Nothing,Measurement}}   # by estimate (in ESTIMATES' order) and tier (small, large)
end

_Step(t, v, r, prior) = _Step(t, v, r, prior, Matrix{Union{Nothing,Measurement}}(nothing, length(ESTIMATES), 2))

const _ORDER = (; (est => i for (i, est) in enumerate(ESTIMATES))...)
_slot(est::Symbol, tier::Symbol) = (_ORDER[est], tier === :small ? 1 : 2)
_get(step::_Step, est::Symbol, tier::Symbol) = step.meas[_slot(est, tier)...]

# The variance and tail weight of a measurement entry.
function _noise(c::TrackerConfig, est, tier, σ)
    n = c.noise[est]
    tier === :small && return max((1 - n.persist) * σ^2, (0.1σ)^2), n.ν_small, true
    return σ^2, n.ν_large, false
end


# The lane component (1: e, 2: ψe, 3: κ) the first entry of an estimate
# measures; 0 for none.
const _LANE_K = (e=1, ψe=2, κ_profile=3)
_lane_k(est::Symbol) = haskey(_LANE_K, est) ? _LANE_K[est] : 0

# Apply a step's measurements to state s; returns the e and ψe measurements'
# normalized innovations (the larger of the two tiers').
function _apply!(s::_State, c::TrackerConfig, step::_Step)
    ms = Measurement[m for m in permutedims(step.meas) if m !== nothing]   # by estimate, the car's first
    used = s.lane.ready ? Set{Tuple{Symbol,Symbol}}() : _start_lane!(s, c, ms)
    paired = _inflate_paired!(s, c, step, used)
    nis = _update_all!(s, c, ms, used, paired)
    _update_cell_biases!(s, c, step)
    return nis
end

# Start the lane from this step's e and ψe (and κ, if measured; the cloud's
# where both tiers measured one). Returns the measurements it started from,
# which are not applied again.
function _start_lane!(s::_State, c::TrackerConfig, ms)
    used = Set{Tuple{Symbol,Symbol}}()
    lane_ms, from = Dict{Symbol,Any}(), Dict{Symbol,Tuple{Symbol,Symbol}}()
    for m in ms   # the cloud's, if both (it comes later in the order)
        k = _lane_k(m.estimate)
        k == 0 && continue
        comp = _LANE[k]
        R, _, bias = _noise(c, m.estimate, m.tier, m.σ[1])
        lane_ms[comp] = (z=m.value[1], R, bias)
        from[comp] = (m.estimate, m.tier)
    end
    if haskey(lane_ms, :e) && haskey(lane_ms, :ψe)
        _start!(s.lane, c, lane_ms)
        union!(used, values(from))
    end
    return used
end

# Where both tiers measured e, ψe or κ at this scan, their difference shows
# the car's bias. A difference larger than the filter expects is evidence of
# a larger bias, not of a wild cloud result: the bias's variance grows to
# match it, and the pair updates without down-weighting either. Returns the
# estimates paired so.
function _inflate_paired!(s::_State, c::TrackerConfig, step::_Step, used)
    paired = Set{Symbol}()
    s.lane.ready || return paired
    for (est, k) in pairs(_LANE_K)
        sm, lg = _get(step, est, :small), _get(step, est, :large)
        (sm === nothing || lg === nothing) && continue
        ((est, :small) in used || (est, :large) in used) && continue
        Rs, _, _ = _noise(c, est, :small, sm.σ[1])
        Rl, _, _ = _noise(c, est, :large, lg.σ[1])
        d = sm.value[1] - lg.value[1]
        k == 2 && (d = rem2pi(d, RoundNearest))
        excess = (d - s.lane.x[3+k])^2 - (s.lane.P[3+k, 3+k] + Rs + Rl)
        excess > 0 && (s.lane.P[3+k, 3+k] += excess)
        push!(paired, est)
    end
    return paired
end

# Every measurement: its first entry into the lane filter (e, ψe, κ), the
# rest (and κ_profile's first) into the cells as the latest of its tier.
function _update_all!(s::_State, c::TrackerConfig, ms, used, paired)
    nis = Dict{Symbol,Float64}()
    for m in ms
        est, k = m.estimate, _lane_k(m.estimate)
        for (j, val) in enumerate(m.value)
            R, ν, bias = _noise(c, est, m.tier, m.σ[j])
            if j == 1 && k > 0 && s.lane.ready && !((est, m.tier) in used)
                q = _update!(s.lane, k, val, R, ν, bias, c.robust && !(est in paired))
                k <= 2 && (nis[est] = max(get(nis, est, 0.0), q))
            end
            (est === :e || est === :ψe) && continue
            isfinite(val) || continue
            cell, i = _cell(est, j)
            cl = s.cells[cell]
            if bias   # the car's
                cl.zs[i], cl.Rs[i], cl.odo_s[i] = val, R, s.odo
            else
                cl.zl[i], cl.Rl[i], cl.odo_l[i] = val, R, s.odo
            end
        end
    end
    return nis
end

# Every number both tiers measured at this scan: an observation of the car's bias.
function _update_cell_biases!(s::_State, c::TrackerConfig, step::_Step)
    for est in (:κ_profile, :widths, :sight, :clearance)
        small, large = _get(step, est, :small), _get(step, est, :large)
        (small === nothing || large === nothing) && continue
        for j in eachindex(small.value)
            Rs, _, _ = _noise(c, est, :small, small.σ[j])
            cell, i = _cell(est, j)
            _bias_update!(s.cells[cell], i, small.value[j] - large.value[j], Rs + large.σ[j]^2)
        end
    end
    return s
end

"""
    LaneTracker(ref_offset; cfg=TrackerConfig())

Tracks the lane from the measurements of both tiers. `ref_offset` is the
front axle's distance ahead of the rear axle [m]. Feed it every scan with
[`track!`](@ref).
"""
mutable struct LaneTracker
    cfg::TrackerConfig
    ref_offset::Float64
    steps::Vector{_Step}
    state::_State             # after the last step's measurements
    outliers::Int
    unmeasured::Int           # scans since e or ψe was last measured
    tiers::Dict{Symbol,Symbol}
end

LaneTracker(ref_offset::Real; cfg::TrackerConfig=TrackerConfig()) =
    LaneTracker(cfg, ref_offset, _Step[], _State(cfg), 0, typemax(Int) ÷ 2, Dict(est => :none for est in ESTIMATES))

_find_step(tr::LaneTracker, t) = findlast(s -> abs(s.t - t) < 1e-6, tr.steps)
# A step with e and ψe (from either tier).
_measured(s::_Step) = any(!isnothing, view(s.meas, _ORDER.e, :)) && any(!isnothing, view(s.meas, _ORDER.ψe, :))

# Filter again from step `first` to the newest, into tr.state; returns the
# normalized innovations of e and ψe at step `gate` (none for 0).
function _replay!(tr::LaneTracker, first::Integer, gate::Integer)
    c = tr.cfg
    s = copy(tr.steps[first].prior)
    nis = Dict{Symbol,Float64}()
    for k in first:length(tr.steps)
        step = tr.steps[k]
        if k > first
            s = _predict!(s, c, tr.ref_offset, step.v, step.r, step.t - tr.steps[k-1].t)
            step.prior = copy(s)
        end
        q = _apply!(s, c, step)
        k == gate && (nis = q)
    end
    tr.state = s
    return nis
end

"""
    track!(tr, measurements, v, yaw_rate, t) -> LaneState

Add the measurements given at time `t` [s] (see [`observe!`](@ref)), with
the car's speed `v` [m/s] and yaw rate [rad/s], and return the lane now.
"""
function track!(tr::LaneTracker, meas::AbstractVector{Measurement}, v::Real, yaw_rate::Real, t::Real)
    c, t, v, r = tr.cfg, Float64(t), Float64(v), Float64(yaw_rate)

    # a new step, predicted from the last (tr.state is replaced below, so it
    # becomes the step's prior without a copy)
    prior = isempty(tr.steps) ? tr.state : _predict!(tr.state, c, tr.ref_offset, v, r, t - tr.steps[end].t)
    push!(tr.steps, _Step(t, v, r, prior))
    length(tr.steps) > c.history && popfirst!(tr.steps)

    # file the measurements under their scans
    first_changed, newest = length(tr.steps), 0
    for m in sort(meas; by=m -> m.scan_time)
        k = _find_step(tr, m.scan_time)
        k === nothing && continue   # older than the history
        tr.steps[k].meas[_slot(m.estimate, m.tier)...] = m
        first_changed, newest = min(first_changed, k), max(newest, k)
        tr.tiers[m.estimate] = m.tier
    end

    # filter again from the earliest scan that changed
    nis = _replay!(tr, first_changed, newest)

    # e or ψe far off at the newest scan measured, scan after scan: the lane
    # changed; restart from that scan
    if get(nis, :e, 0.0) > c.gate^2 || get(nis, :ψe, 0.0) > c.gate^2
        tr.outliers += 1
        if tr.outliers >= c.max_outliers
            fresh = copy(tr.steps[newest].prior)
            fresh.lane.ready = false
            tr.steps[newest].prior = fresh
            deleteat!(tr.steps, 1:newest-1)
            _replay!(tr, 1, 0)
            tr.outliers = 0
        end
    else
        tr.outliers = 0
    end

    # scans since the newest one with e and ψe (late ones count at their scan)
    k = findlast(_measured, tr.steps)
    tr.unmeasured = k === nothing ? tr.unmeasured + 1 : length(tr.steps) - k
    return _lane_state(tr)
end

function _lane_state(tr::LaneTracker)
    s, c = tr.state, tr.cfg
    status = !s.lane.ready ? :lost : tr.unmeasured == 0 ? :ok : tr.unmeasured <= c.lost_after ? :predicting : :lost
    est(k) = _estimates(s.cells[k], s.odo, _q(c, k); profile=k in (:κ_profile, :w_left, :w_right))
    κp, _ = est(:κ_profile)
    s.lane.ready && (κp[1] = s.lane.x[3])
    wl, _ = est(:w_left)
    wr, _ = est(:w_right)
    sx, sv = est(:sight)
    cx, cv = est(:clearance)
    return LaneState(s.lane.x[1], rem2pi(s.lane.x[2], RoundNearest), s.lane.x[3], κp, wl, wr,
        isfinite(sx[1]) ? clamp(sx[1], 0.0, SIGHT_MAX) : 0.0, isfinite(sx[1]) ? sqrt(sv[1]) : Inf,
        [isfinite(x) ? clamp(x, 0.0, CLEARANCE_MAX) : 0.0 for x in cx],
        [isfinite(x) ? sqrt(v) : Inf for (x, v) in zip(cx, cv)],
        NamedTuple{ESTIMATES}(Tuple(tr.tiers[est] for est in ESTIMATES)), status)
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
