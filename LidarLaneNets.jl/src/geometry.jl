# Estimates measured directly in the scan, without a network: the free room
# beside the lane center, how far ahead the center is seen, and the free
# distance along the fan of arcs. What the scan doesn't show (behind a wall,
# past the LiDAR's range, in its blind spot behind the car) counts as
# blocked, so the clearance errs toward less room (on the held-out tracks it
# exceeded the true free distance by more than 0.25 m in 0.1% of the
# entries, the car network's in 3%); the widths and sight are measured along
# a center someone else estimated, and are as right as it is.
#
# And a way to drive on them: a `MixedEstimator` takes some estimates from
# such a source instead of the car's networks.

"""
    ScanGeometry(beams, x_ref; hw, range_max=RANGE_MAX)

Measures estimates directly in a scan from a LiDAR whose beams point at
`beams` [rad] (evenly spaced, counterclockwise from straight ahead), with
the front axle `x_ref` meters ahead of the LiDAR (negative: behind it), for
a car `2hw` wide. A range past `range_max` is no return within range.

Given the lane center (from `e`, `ψe` and `κ_profile`, see
[`center_line`](@ref)): [`scan_widths`](@ref), [`scan_sight`](@ref); and
[`scan_clearance`](@ref), which needs no center. Each measures what its
label measures on the map (see LidarLaneNets.jl's `train/truth.jl`), but in
the scan.
"""
struct ScanGeometry
    a0::Float64               # first beam's angle [rad]
    da::Float64               # angle between beams [rad]
    cos_a::Vector{Float64}
    sin_a::Vector{Float64}
    x_ref::Float64
    hw::Float64
    range_max::Float64
end

function ScanGeometry(beams::AbstractVector{<:Real}, x_ref::Real; hw::Real, range_max::Real=RANGE_MAX)
    b = collect(Float64, beams)
    length(b) >= 2 || throw(ArgumentError("need at least 2 beams"))
    da = (b[end] - b[1]) / (length(b) - 1)
    (da > 0 && all(d -> abs(d - da) < 1e-9, diff(b))) ||
        throw(ArgumentError("beams must be evenly spaced, counterclockwise"))
    return ScanGeometry(b[1], da, cos.(b), sin.(b), Float64(x_ref), Float64(hw), Float64(range_max))
end

_range(g::ScanGeometry, ranges, i) = (r = Float64(ranges[i]); r > g.range_max ? g.range_max : r)

# Whether the scan shows the point (x, y) (LiDAR frame) to be free: in the
# field of view and within range, and nearer than the returns of the beams on
# either side of it. Points on the car itself (within its half width of the
# LiDAR, the front axle among them) count as free.
function _visible(g::ScanGeometry, ranges, x, y)
    d = hypot(x, y)
    d <= g.hw && return true
    d > g.range_max && return false
    u = (atan(y, x) - g.a0) / g.da + 1
    (1 <= u <= length(g.cos_a)) || return false
    i = min(floor(Int, u), length(g.cos_a) - 1)
    return d <= min(_range(g, ranges, i), _range(g, ranges, i + 1))
end

"""
    center_line(e, ψe, κ_profile; reach=SIGHT_MAX) -> (x, y, h)

The lane center in the front axle's frame, every `PROFILE_STEP` meters from
the point beside the car out to `reach`: its point beside the car is `e` to
the side, it starts in direction `ψe` and bends by `κ_profile` (past its
end, or a NaN, by the last curvature known), with its heading `h`.
"""
function center_line(e::Real, ψe::Real, κ_profile::AbstractVector{<:Real}; reach::Real=SIGHT_MAX)
    n = floor(Int, reach / PROFILE_STEP + 1e-9) + 1
    xs, ys, hs = zeros(n), zeros(n), zeros(n)
    h, x, y = Float64(ψe), -e * sin(ψe), e * cos(ψe)
    κ_at(j) = j <= length(κ_profile) && isfinite(κ_profile[j]) ? Float64(κ_profile[j]) : NaN
    κ_prev = isfinite(κ_at(1)) ? κ_at(1) : 0.0
    xs[1], ys[1], hs[1] = x, y, h
    for j in 2:n
        κ = κ_at(j)
        κ = isfinite(κ) ? κ : κ_prev
        hn = h + PROFILE_STEP * (κ_prev + κ) / 2
        hm, half = (h + hn) / 2, (hn - h) / 2
        chord = PROFILE_STEP * (abs(half) < 1e-9 ? 1.0 : sin(half) / half)   # of an arc of constant curvature
        x, y, h = x + chord * cos(hm), y + chord * sin(hm), hn
        xs[j], ys[j], hs[j] = x, y, h
        κ_prev = κ
    end
    return xs, ys, hs
end

"""
    scan_widths(g, ranges, x, y, h; step=0.02) -> widths

The free room [m] from the lane center to the left, then to the right, at
`PROFILE_S` (the center as from [`center_line`](@ref)): how far the scan
shows free space along each side's normal, at most `WIDTH_MAX`. NaN where
the scan doesn't show the center point itself.
"""
function scan_widths(g::ScanGeometry, ranges, x, y, h; step::Real=0.02)
    np = length(PROFILE_S)
    w = fill(NaN, 2np)
    for j in 1:min(np, length(x))
        px, py = x[j] + g.x_ref, y[j]
        _visible(g, ranges, px, py) || continue
        for (k, side) in ((j, 1.0), (np + j, -1.0))
            nx, ny = -side * sin(h[j]), side * cos(h[j])
            free(t) = _visible(g, ranges, px + t * nx, py + t * ny)
            t = 0.0
            while t + step <= WIDTH_MAX && free(t + step)
                t += step
            end
            if t + step <= WIDTH_MAX   # the edge is within the next step: find it to 1 mm
                lo, hi = t, t + step
                while hi - lo > 0.001
                    mid = (lo + hi) / 2
                    free(mid) ? (lo = mid) : (hi = mid)
                end
                t = lo
            end
            w[k] = t
        end
    end
    return w
end

"""
    scan_sight(g, ranges, x, y) -> d

How far ahead along the lane center (as from [`center_line`](@ref), out to
`SIGHT_MAX`) the scan shows it [m]: up to the point before the first one
the scan doesn't show.
"""
function scan_sight(g::ScanGeometry, ranges, x, y)
    for j in 2:length(x)
        _visible(g, ranges, x[j] + g.x_ref, y[j]) || return (j - 2) * PROFILE_STEP
    end
    return min((length(x) - 1) * PROFILE_STEP, SIGHT_MAX)
end

# The point s meters along the arc of curvature κ leaving the front axle
# straight ahead (front axle frame).
function _arc_point(κ, s)
    abs(κ) < 1e-6 && return s, 0.0
    return sin(κ * s) / κ, (1 - cos(κ * s)) / κ
end

"""
    scan_clearance(g, ranges, κ; step=0.05) -> d

The free distance [m] along the arc of curvature `κ` [1/m] that leaves the
front axle straight ahead, keeping `hw` from every return: where the car,
driving along it, first comes within `hw` of one, or where the arc leaves
what the scan shows; `CLEARANCE_MAX` if neither happens within
`CLEARANCE_MAX` or half a turn (whichever is shorter).
"""
function scan_clearance(g::ScanGeometry, ranges, κ::Real; step::Real=0.05)
    limit = abs(κ) < 1e-6 ? CLEARANCE_MAX : min(CLEARANCE_MAX, π / abs(κ))
    hw = g.hw
    hit = limit
    @inbounds for i in eachindex(g.cos_a)
        r = Float64(ranges[i])
        r > g.range_max && continue
        x, y = r * g.cos_a[i] - g.x_ref, r * g.sin_a[i]   # front axle frame
        if abs(κ) < 1e-6
            (x > 0 && abs(y) <= hw) || continue
            s = x - sqrt(hw^2 - y^2)          # where the car's side first reaches it
        else
            R = 1 / κ                          # the arc's center is at (0, R)
            δ = hypot(x, y - R) - abs(R)
            abs(δ) <= hw || continue
            φ = sign(κ) * atan(R * x, -R * (y - R))   # angle swept to its nearest point on the arc
            φ > 0 || continue
            s = φ * abs(R) - sqrt(hw^2 - δ^2)
        end
        hit = min(hit, max(s, 0.0))
    end
    s = 0.0
    while s < hit
        x, y = _arc_point(κ, s)
        _visible(g, ranges, x + g.x_ref, y) || return s
        s += step
    end
    return hit >= limit ? CLEARANCE_MAX : hit
end

"""
    scan_clearance(g, ranges) -> Vector

Along every arc of `CLEARANCE_K`.
"""
scan_clearance(g::ScanGeometry, ranges) = [scan_clearance(g, ranges, κ) for κ in CLEARANCE_K]

# ---------------------------------------------------------------------------
# Driving on them

"""
    GEOMETRIC_ESTIMATES

The estimates [`ScanGeometry`](@ref) measures: `(:widths, :sight, :clearance)`.
"""
const GEOMETRIC_ESTIMATES = (:widths, :sight, :clearance)

"""
    (g::ScanGeometry)(ranges, meas) -> Dict{Symbol,Vector{Float64}}

The estimates measured in the scan `ranges`, along the lane center that the
car's results for this scan in `meas` give (`e`, `ψe`, `κ_profile`, tier
`:small`; without them, `clearance` alone): a source for a
[`MixedEstimator`](@ref).
"""
function (g::ScanGeometry)(ranges, meas)
    out = Dict{Symbol,Vector{Float64}}(:clearance => scan_clearance(g, ranges))
    car(est) = (i = findlast(m -> m.estimate === est && m.tier === :small, meas); i === nothing ? nothing : meas[i].value)
    e, ψe, κp = car(:e), car(:ψe), car(:κ_profile)
    (e === nothing || ψe === nothing || κp === nothing) && return out
    x, y, h = center_line(e[1], ψe[1], κp)
    out[:widths] = scan_widths(g, ranges, x, y, h)
    out[:sight] = [scan_sight(g, ranges, x, y)]
    return out
end

"""
    MixedEstimator(base, source, estimates, σ)

Some estimates (`estimates`) from `source` instead of the car's networks:
every scan, `base` (a [`TieredEstimator`](@ref), or `nothing` when `source`
gives every estimate) runs as usual, its results for `estimates` are
dropped, and `source(ranges, meas)` (`meas`: `base`'s results) gives theirs,
as a `Dict` from estimate to values (leaving out what it can't measure this
scan), each with the fixed standard deviations `σ[estimate]`, as the car's
(tier `:small`). Feed it with [`observe!`](@ref).

A [`ScanGeometry`](@ref) is such a source for [`GEOMETRIC_ESTIMATES`](@ref).
"""
struct MixedEstimator{B,F}
    base::B
    source::F
    estimates::Vector{Symbol}
    σ::Dict{Symbol,Vector{Float64}}
end

function MixedEstimator(base, source, estimates, σ::AbstractDict)
    ests = Symbol[_check(e) for e in estimates]
    for e in ests
        haskey(σ, e) && length(σ[e]) == output_size(e) || throw(ArgumentError("need σ for every entry of $e"))
    end
    base === nothing && Set(ests) != Set(ESTIMATES) && throw(ArgumentError("without a base, the source must give every estimate"))
    return MixedEstimator(base, source, ests, Dict{Symbol,Vector{Float64}}(Symbol(k) => Float64.(v) for (k, v) in σ))
end

# The cloud calls made (the base's; none without one).
function Base.getproperty(m::MixedEstimator, s::Symbol)
    s === :calls || return getfield(m, s)
    base = getfield(m, :base)
    return base === nothing ? Dict(est => 0 for est in ESTIMATES) : base.calls
end

function observe!(m::MixedEstimator, ranges::AbstractVector{<:Real}, t::Real; lane=nothing, v=0.0, yaw_rate=0.0)
    meas = m.base === nothing ? Measurement[] : observe!(m.base, ranges, t; lane, v, yaw_rate)
    from_base = filter(x -> !(x.estimate in m.estimates), meas)
    for (est, val) in m.source(ranges, meas)
        est in m.estimates || continue
        push!(from_base, Measurement(est, Float64(t), :small, Float64.(val), m.σ[est]))
    end
    return from_base
end
