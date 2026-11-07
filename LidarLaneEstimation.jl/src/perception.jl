# LiDAR lane perception: one scan to the lane, and where the vehicle is in it
#
# detect_lane!:
# 1. Scan to points in the vehicle frame.
# 2. Split the points into the left and the right wall: each wall starts at
#    the nearest return beside the vehicle and is followed beam by beam until a
#    gap, so a wall that curves across the front of the vehicle stays one wall.
# 3. Pair each point of a wall with the nearest point of the other wall; the
#    midpoints, taken in order along the wall, trace the lane center.
#    Nothing assumes the lane is straight or gently curved, so hairpins work
#    too.
# lane_corridor: how far each point of the center may move sideways while
#    keeping a margin from every return (for a path planner).
# measure!: the reference point against a line (the center, or a planned
#    path): errors, curvature ahead, and how far ahead the line reaches.

"""
    PerceptionConfig(; kwargs...)

Tuning of the scan processing. Distances in meters, angles in radians.
"""
Base.@kwdef struct PerceptionConfig
    # Returns farther than this are ignored. Larger sees curves earlier, but
    # far points are sparse and pair poorly; smaller can lose a wall in wide areas.
    r_max::Float64 = 6.0
    # Gap between neighboring returns that ends a wall. Too small splits one
    # wall at grazing angles; too large joins separate walls (e.g. across an
    # opening). Keep it well below the lane width.
    jump::Float64 = 0.3
    # Each wall starts at the nearest return in a sector around ±90°, reaching
    # this far forward... Larger can start a "side" wall on the wall ahead.
    seed_ahead::Float64 = 0.6
    # ... and this far back. 0.8 reaches the scan's edge, which keeps the inner
    # wall of a hairpin in view for as long as possible.
    seed_behind::Float64 = 0.8
    # Lane width = median of this many scans' measurements (25 s at 40 Hz).
    # Larger ignores longer wide stretches (hairpin bulbs, pit entries) but
    # adapts more slowly if the width really changes. Used only with one wall.
    width_memory::Int = 1000
    # Use every stride-th return. Larger is faster (wall pairing is
    # quadratic) but gives fewer, noisier centerline samples.
    stride::Int = 2
    # Moving-average half-window over the centerline samples. Larger smooths
    # out LiDAR noise but rounds off tight corners and shortens the line's ends.
    smooth::Int = 3
    # The centerline is resampled to points this far apart [m]. Smaller
    # follows the shape more closely but makes segment headings noisier.
    spacing::Float64 = 0.1
    # A centerline shorter than this [m] is not trusted: fall back to one wall,
    # or to driving straight. Larger is more cautious.
    min_length::Float64 = 1.0
    # Curvature is the circle through three points this far apart in total [m].
    # Larger is less noisy (false slowdowns on straights) but blurs short,
    # sharp turns, so they look gentler than they are.
    curve_base::Float64 = 2.0
    # `κ_profile` reaches this far ahead [m], and `κ_ahead` is its sharpest
    # curve. A controller that slows for curves ahead can brake earlier with a
    # larger value; past what the scan sees, the profile is NaN.
    preview::Float64 = 4.0
    # Half-width of the corridor straight ahead in which `clearance` is
    # measured [m], about half the vehicle's width. Larger is more cautious
    # (it catches the walls beside the vehicle); smaller can miss a wall its
    # corner will hit.
    car_halfwidth::Float64 = 0.2
end

"""
    LaneEstimate

Where the vehicle is in the lane, all measured at the reference point (see
[`LanePerception`](@ref)) against a line: the lane center
([`estimate_lane!`](@ref)) or any other, such as a planned path
([`measure!`](@ref)).

- `e`: cross-track error [m], the distance to the line; `> 0` when the line
  is to the left.
- `ψe`: heading error [rad], the line's direction minus the vehicle's; `> 0`
  when the line points to the left.
- `κ`: the line's curvature [1/m]; `> 0` turning left.
- `κ_ahead`: the largest `|κ|` within `preview` meters ahead.
- `width`: the lane width [m] (a long-running median), NaN until both walls
  have been seen.
- `walls`: which walls were used: `:both`; `:left` or `:right` alone (the
  center is half a lane width in from it); `:merged`, one wall seen on both
  sides, deep in a hairpin; or `:none`, when the other numbers are zero.
- `clearance`: the free distance straight ahead [m].
- `κ_profile`: the signed curvature every `PROFILE_STEP` meters ahead, NaN
  where the scan doesn't reach.
- `sight`: how far the line reaches ahead of the reference point [m]; what
  lies past it is unknown. 0 when no lane is seen.
"""
struct LaneEstimate
    e::Float64
    ψe::Float64
    κ::Float64
    κ_ahead::Float64
    width::Float64
    walls::Symbol
    clearance::Float64
    κ_profile::Vector{Float64}
    sight::Float64
end

"""
    LanePerception(angles, x_ref; cfg=PerceptionConfig())
    LanePerception(num_beams, fov, x_ref; cfg=PerceptionConfig())

Lane perception for a LiDAR whose beam `i` points at `angles[i]` [rad]
(counterclockwise from straight ahead, in increasing order), or for
`num_beams` beams spread evenly over the field of view `fov`.

The errors are measured at the reference point `x_ref` meters ahead of the
LiDAR (negative if behind it), on the vehicle's center line. Use the point
your controller expects: the front axle for the Stanley law, the rear axle
for pure pursuit, or the center of gravity for most LQR and MPC models.

After [`detect_lane!`](@ref) (or [`estimate_lane!`](@ref)), the fields
`center_x` and `center_y` hold the lane center in the LiDAR frame (x
forward, y left), in driving order, for controllers that work on the line
itself (such as pure pursuit), for path planners, or for drawing; `walls`
says which walls it came from, and `clearance` is the free distance straight
ahead.
"""
mutable struct LanePerception
    cfg::PerceptionConfig
    angles::Vector{Float64}
    cos_a::Vector{Float64}        # cos and sin of each beam's angle, computed once
    sin_a::Vector{Float64}
    x_ref::Float64                # reference point, meters ahead of the LiDAR
    xs::Vector{Float64}           # scan points in the LiDAR frame
    ys::Vector{Float64}
    left::Vector{Int}             # beam indices of each wall, in driving order
    right::Vector{Int}
    center_x::Vector{Float64}     # the lane center (LiDAR frame)
    center_y::Vector{Float64}
    walls::Symbol                 # which walls it came from (see LaneEstimate)
    clearance::Float64            # free distance straight ahead
    widths::Vector{Float64}       # recent lane width measurements (a ring buffer)
    width_next::Int
    width::Float64                # their median, NaN until both walls are seen
    merged::Bool                  # this scan's "two walls" turned out to be one
    ψe_prev::Float64              # last heading error, to keep a one-wall lane's direction
    bx::Vector{Float64}           # scratch for pairing the walls
    by::Vector{Float64}
    d2::Vector{Float64}
end

function LanePerception(angles::AbstractVector{<:Real}, x_ref::Real; cfg::PerceptionConfig=PerceptionConfig())
    n = length(angles)
    n >= 2 || throw(ArgumentError("need at least 2 beams"))
    issorted(angles) || throw(ArgumentError("beam angles must be in increasing (counterclockwise) order"))
    a = collect(Float64, angles)
    return LanePerception(cfg, a, cos.(a), sin.(a), x_ref, zeros(n), zeros(n), Int[], Int[],
        Float64[], Float64[], :none, Inf, Float64[], 1, NaN, false, 0.0, Float64[], Float64[], Float64[])
end

LanePerception(num_beams::Integer, fov::Real, x_ref::Real; cfg::PerceptionConfig=PerceptionConfig()) =
    LanePerception([-fov / 2 + (i - 1) * fov / (num_beams - 1) for i in 1:num_beams], x_ref; cfg)

# Nearest return beside the vehicle on one side (`side` = 1 left, -1 right), or 0
# if none is in range. The sector reaches back to the edge of the scan, so
# the inner wall of a tight turn is kept for as long as it can be seen.
function _seed(p::LanePerception, ranges, side)
    best, best_r = 0, p.cfg.r_max
    for i in eachindex(ranges)
        off = side * p.angles[i] - π / 2   # > 0 behind the side direction
        if -p.cfg.seed_ahead <= off <= p.cfg.seed_behind && ranges[i] < best_r
            best, best_r = i, ranges[i]
        end
    end
    return best
end

# Walk from `seed` in direction `step` while neighboring returns are close
# together (the same wall) and in range, not past index `stop`.
function _grow!(chain, p::LanePerception, ranges, seed, step, stop)
    i = seed
    while i != stop
        j = i + step
        (ranges[j] > p.cfg.r_max || hypot(p.xs[j] - p.xs[i], p.ys[j] - p.ys[i]) > p.cfg.jump) && break
        push!(chain, j)
        i = j
    end
    return chain
end

# Beams sweep counterclockwise, so the right wall runs forward with the beam
# index and the left wall against it. If the right wall runs without a gap
# into the left seed, both seeds are on one wall (deep in a hairpin, the
# outer wall wraps around the vehicle while the inner one is behind it, out of
# view): it is kept whole in `right` and `merged` is set.
function _walls!(p::LanePerception, ranges)
    empty!(p.left)
    empty!(p.right)
    p.merged = false
    n = length(ranges)
    sr, sl = _seed(p, ranges, -1), _seed(p, ranges, 1)
    if sr > 0
        push!(p.right, sr)
        _grow!(p.right, p, ranges, sr, -1, 1)
        _grow!(p.right, p, ranges, sr, +1, sl > 0 ? sl - 1 : n)
        if sl > 0 && (sl - 1 in p.right) && ranges[sl] <= p.cfg.r_max &&
           hypot(p.xs[sl] - p.xs[sl-1], p.ys[sl] - p.ys[sl-1]) <= p.cfg.jump
            p.merged = true
            push!(p.right, sl)
            _grow!(p.right, p, ranges, sl, +1, n)
        end
        sort!(p.right)
    end
    if sl > 0 && !p.merged && !(sl in p.right)
        right_end = isempty(p.right) ? 0 : last(p.right)
        push!(p.left, sl)
        _grow!(p.left, p, ranges, sl, +1, n)
        _grow!(p.left, p, ranges, sl, -1, right_end + 1)
        sort!(p.left; rev=true)
    end
    return p
end

# Moving average of the points (xs[k], ys[k]), then keep points at least
# `spacing` apart: a smooth polyline in the same order.
function _smooth(xs, ys, h, spacing)
    cx, cy = Float64[], Float64[]
    n = length(xs)
    for k in 1:n
        lo, hi = max(1, k - h), min(n, k + h)
        x = sum(@view xs[lo:hi]) / (hi - lo + 1)
        y = sum(@view ys[lo:hi]) / (hi - lo + 1)
        if isempty(cx) || hypot(x - cx[end], y - cy[end]) >= spacing
            push!(cx, x)
            push!(cy, y)
        end
    end
    return cx, cy
end

_polyline_length(xs, ys) = sum((hypot(xs[k+1] - xs[k], ys[k+1] - ys[k]) for k in 1:length(xs)-1); init=0.0)

# A line long enough to trust (see `min_length`).
_usable(xs, ys, cfg::PerceptionConfig) = length(xs) >= 3 && _polyline_length(xs, ys) >= cfg.min_length

# Lane center from wall `a` (in driving order) and the other wall `b`:
# the midpoints, and the wall-to-wall distance at each.
function _center_from(p::LanePerception, a, b)
    s = p.cfg.stride
    mx, my, mw = Float64[], Float64[], Float64[]
    # the other wall's points, side by side in memory so the distances to all
    # of them can be computed several at a time (SIMD)
    bj = b[1:s:end]
    n = length(bj)
    bx, by, d2 = resize!(p.bx, n), resize!(p.by, n), resize!(p.d2, n)
    @inbounds for m in 1:n
        bx[m], by[m] = p.xs[bj[m]], p.ys[bj[m]]
    end
    @inbounds for k in 1:s:length(a)
        n == 0 && break
        x, y = p.xs[a[k]], p.ys[a[k]]
        best_d2 = Inf
        @simd for m in 1:n
            d2[m] = (x - bx[m])^2 + (y - by[m])^2
            best_d2 = min(best_d2, d2[m])   # vectorizes (a branchy argmin wouldn't)
        end
        best_m = findfirst(==(best_d2), d2)   # the first nearest point
        push!(mx, (x + bx[best_m]) / 2)
        push!(my, (y + by[best_m]) / 2)
        push!(mw, sqrt(best_d2))
    end
    cx, cy = _smooth(mx, my, p.cfg.smooth, p.cfg.spacing)
    return cx, cy, mx, my, mw
end

# Lane center from one wall alone: the wall shifted by half the lane width
# toward the lane (to the left of the right wall, to the right of the left).
function _center_from_wall(p::LanePerception, chain, side)
    pts = chain[1:p.cfg.stride:end]
    wx, wy = _smooth(p.xs[pts], p.ys[pts], p.cfg.smooth, p.cfg.spacing)
    length(wx) < 2 && return wx, wy
    half = if isnan(p.width)
        # never seen both walls: assume the vehicle started centered
        minimum(hypot(wx[k] - p.x_ref, wy[k]) for k in eachindex(wx))
    else
        p.width / 2
    end
    sgn = side === :right ? 1.0 : -1.0
    cx, cy = similar(wx), similar(wy)
    for k in eachindex(wx)
        a, b = max(1, k - 1), min(length(wx), k + 1)
        h = atan(wy[b] - wy[a], wx[b] - wx[a])
        cx[k] = wx[k] - sgn * half * sin(h)
        cy[k] = wy[k] + sgn * half * cos(h)
    end
    return cx, cy
end

# Keep the lane width as a median over many steps, so a short wide stretch
# (the bulb of a hairpin, a pit entry) does not change it.
function _record_width!(p::LanePerception, w)
    if length(p.widths) < p.cfg.width_memory
        push!(p.widths, w)
    else
        p.widths[p.width_next] = w
        p.width_next = mod1(p.width_next + 1, p.cfg.width_memory)
    end
    k = cld(length(p.widths), 2)
    p.width = partialsort!(copyto!(resize!(p.d2, length(p.widths)), p.widths), k)   # the median
    return p
end

"""
    estimate_lane!(p::LanePerception, ranges) -> LaneEstimate

The lane in one scan, measured against its center: [`detect_lane!`](@ref)
then [`measure!`](@ref). `ranges[i]` is the distance measured by beam `i` [m].
"""
function estimate_lane!(p::LanePerception, ranges::AbstractVector{<:Real})
    detect_lane!(p, ranges)
    return measure!(p, p.center_x, p.center_y)
end

"""
    detect_lane!(p::LanePerception, ranges) -> walls

Find the lane in one scan (`ranges[i]` is the distance measured by beam `i`
[m]): its center (`p.center_x`, `p.center_y`), which walls it came from
(returned, and kept in `p.walls`: `:both`, `:left`, `:right`, `:merged`, or
`:none` when no lane was found), and the free distance straight ahead
(`p.clearance`).
"""
function detect_lane!(p::LanePerception, ranges::AbstractVector{<:Real})
    length(ranges) == length(p.angles) ||
        throw(DimensionMismatch("got $(length(ranges)) ranges for $(length(p.angles)) beams"))
    cfg = p.cfg
    xs, ys, cos_a, sin_a = p.xs, p.ys, p.cos_a, p.sin_a
    @inbounds @simd for i in eachindex(ranges, xs)
        xs[i] = ranges[i] * cos_a[i]
        ys[i] = ranges[i] * sin_a[i]
    end
    clearance = Inf
    @inbounds for i in eachindex(xs)
        if xs[i] > 0 && abs(ys[i]) <= cfg.car_halfwidth
            clearance = min(clearance, xs[i] - p.x_ref)
        end
    end
    _walls!(p, ranges)

    walls = :none
    cx, cy = Float64[], Float64[]
    if length(p.left) >= 2cfg.stride && length(p.right) >= 2cfg.stride
        # Build the center along each wall and keep the longer one: around a
        # turn that is the outer wall, which maps onto the center smoothly.
        rx, ry, _, _, _ = _center_from(p, p.right, p.left)
        lx, ly, mx, my, mw = _center_from(p, p.left, p.right)
        cx, cy = _polyline_length(rx, ry) >= _polyline_length(lx, ly) ? (rx, ry) : (lx, ly)
        walls = :both
        # lane width from the pairs beside the reference point
        near = [mw[k] for k in eachindex(mw) if hypot(mx[k] - p.x_ref, my[k]) <= 1.0]
        if !isempty(near)
            _record_width!(p, sort!(near)[cld(length(near), 2)])
        end
    end
    if !_usable(cx, cy, cfg)
        # One wall (or a broken pair): follow the longer wall at half the lane
        # width. A merged wall is kept in `right`, in counterclockwise order
        # with the lane on its left, like a right wall.
        side = p.merged || length(p.right) > length(p.left) ? :right : :left
        chain = side === :left ? p.left : p.right
        cx, cy = length(chain) >= 2cfg.stride ? _center_from_wall(p, chain, side) : (Float64[], Float64[])
        walls = p.merged ? :merged : side
    end
    p.center_x, p.center_y = cx, cy
    p.walls = _usable(cx, cy, cfg) ? walls : :none
    p.clearance = clearance
    return p.walls
end

"""
    measure!(p::LanePerception, x, y) -> LaneEstimate

Where the reference point is relative to the line through `(x[i], y[i])`
(LiDAR frame, in driving order) in the lane last found by
[`detect_lane!`](@ref): the lane center, or a path planned through the lane.
With no lane found, `walls` is `:none` and the errors are zero.
"""
function measure!(p::LanePerception, x::AbstractVector{<:Real}, y::AbstractVector{<:Real})
    cfg, walls, clearance = p.cfg, p.walls, p.clearance
    if walls === :none || length(x) < 2
        return LaneEstimate(0.0, 0.0, 0.0, 0.0, p.width, :none, clearance, fill(NaN, length(_profile_offsets(cfg))), 0.0)
    end
    line = _Polyline(x, y)
    err = _project(line, p.x_ref, 0.0)   # the vehicle faces +x
    e, ψe = err.e, err.h
    dir = 1   # +1 if the lane runs along the path, -1 if against it
    if walls === :merged && abs(_wrap(ψe - p.ψe_prev)) > π / 2
        # A wall wrapped around the vehicle does not say which way the lane runs;
        # keep the direction of the previous step (reversing a path flips e
        # and κ and turns its heading by π).
        e, ψe, dir = -e, _wrap(ψe + π), -1
    end
    p.ψe_prev = ψe
    κ = _curvature_at(line, err.s, cfg.curve_base)
    κ = isnan(κ) ? 0.0 : dir * κ
    # signed curvature every PROFILE_STEP meters ahead, NaN past what the scan sees
    profile = [dir * _curvature_at(line, err.s + dir * d, cfg.curve_base) for d in _profile_offsets(cfg)]
    sight = max(dir == 1 ? line.s[end] - err.s : err.s, 0.0)
    return LaneEstimate(e, ψe, κ, _sharpest(profile, κ), p.width, walls, clearance, profile, sight)
end

"""
    PROFILE_STEP

Spacing of `LaneEstimate.κ_profile` [m]: entry `j` is `(j - 1) * PROFILE_STEP`
meters ahead of the reference point.
"""
const PROFILE_STEP = 0.25

_profile_offsets(cfg::PerceptionConfig) = 0.0:PROFILE_STEP:cfg.preview

# Largest |κ| in a profile, ignoring unknown (NaN) entries.
_sharpest(profile, κ) = maximum((abs(k) for k in profile if !isnan(k)); init=abs(κ))
