# LiDAR lane perception: one scan to the lane center
#
# 1. Scan to points in the vehicle frame.
# 2. Split the points into the left and the right wall: each wall starts at
#    the nearest return beside the car and is followed beam by beam until a
#    gap, so a wall that curves across the front of the car stays one wall.
# 3. Pair each point of a wall with the nearest point of the other wall; the
#    midpoints, taken in order along the wall, trace the lane center.
# 4. Smooth them into a local path and measure the car against it with
#    `tracking_error`, exactly as for a path known in advance. Nothing assumes
#    the lane is straight or gently curved, so hairpins work too.

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
    # Lane width = median of this many scans' measurements (10 s at 100 Hz).
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
    # The car slows for the sharpest curve within this distance ahead [m].
    # Larger brakes earlier for turns; too large reaches past what the scan sees.
    preview::Float64 = 4.0
    # Half-width of the corridor straight ahead checked for a wall [m]; caps
    # speed by the braking distance. Larger is more cautious (it catches the
    # walls beside the car); smaller can miss a wall the car's corner will hit.
    car_halfwidth::Float64 = 0.2
end

"""
    LaneEstimate

Where the vehicle is in the lane: cross-track error `e` (`> 0` when the lane
center is to the left), heading error `ψe` (`> 0` when the lane points
left) and lane curvature `κ` (`> 0` turning left) at the front axle, the largest curvature in
the next `preview` meters `κ_ahead`, the lane width, which walls were used
(`:both`; `:left` or `:right` alone; `:merged`, one wall seen on both sides;
or `:none`), the free distance straight ahead of the front axle, and the
signed curvature every `PROFILE_STEP` meters ahead of the axle (NaN where the
scan doesn't reach).
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
end

"""
    LanePerception(angles, x_fa; cfg=PerceptionConfig())
    LanePerception(num_beams, fov, x_fa; cfg=PerceptionConfig())

Lane perception for a LiDAR whose beam `i` points at `angles[i]` [rad]
(counterclockwise from straight ahead, in increasing order), or for
`num_beams` beams spread evenly over the field of view `fov`. `x_fa` is how
far the front axle is ahead of the LiDAR [m]; the errors are measured there.

After [`estimate_lane!`](@ref), the fields `cx` and `cy` hold the estimated
lane center in the LiDAR frame, for drawing.
"""
mutable struct LanePerception
    cfg::PerceptionConfig
    angles::Vector{Float64}
    x_fa::Float64                 # front axle, meters ahead of the scan origin
    xs::Vector{Float64}           # scan points in the car frame
    ys::Vector{Float64}
    left::Vector{Int}             # beam indices of each wall, in driving order
    right::Vector{Int}
    cx::Vector{Float64}           # estimated lane center (car frame), for drawing
    cy::Vector{Float64}
    widths::Vector{Float64}       # recent lane width measurements (a ring buffer)
    width_next::Int
    width::Float64                # their median, NaN until both walls are seen
    merged::Bool                  # this scan's "two walls" turned out to be one
    ψe_prev::Float64              # last heading error, to keep a one-wall lane's direction
end

function LanePerception(angles::AbstractVector{<:Real}, x_fa::Real; cfg::PerceptionConfig=PerceptionConfig())
    n = length(angles)
    n >= 2 || throw(ArgumentError("need at least 2 beams"))
    issorted(angles) || throw(ArgumentError("beam angles must be in increasing (counterclockwise) order"))
    return LanePerception(cfg, collect(Float64, angles), x_fa, zeros(n), zeros(n), Int[], Int[],
        Float64[], Float64[], Float64[], 1, NaN, false, 0.0)
end

LanePerception(num_beams::Integer, fov::Real, x_fa::Real; cfg::PerceptionConfig=PerceptionConfig()) =
    LanePerception([-fov / 2 + (i - 1) * fov / (num_beams - 1) for i in 1:num_beams], x_fa; cfg)

# Nearest return beside the car on one side (`side` = 1 left, -1 right), or 0
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
# outer wall wraps around the car while the inner one is behind it, out of
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

# Lane center from wall `a` (in driving order) and the other wall `b`:
# the midpoints, and the wall-to-wall distance at each.
function _center_from(p::LanePerception, a, b)
    s = p.cfg.stride
    mx, my, mw = Float64[], Float64[], Float64[]
    for k in 1:s:length(a)
        i = a[k]
        best_d2, best_j = Inf, 0
        for m in 1:s:length(b)
            j = b[m]
            d2 = (p.xs[i] - p.xs[j])^2 + (p.ys[i] - p.ys[j])^2
            if d2 < best_d2
                best_d2, best_j = d2, j
            end
        end
        best_j == 0 && continue
        push!(mx, (p.xs[i] + p.xs[best_j]) / 2)
        push!(my, (p.ys[i] + p.ys[best_j]) / 2)
        push!(mw, sqrt(best_d2))
    end
    cx, cy = _smooth(mx, my, p.cfg.smooth, p.cfg.spacing)
    return cx, cy, mx, my, mw
end

# Lane center from one wall alone: the wall shifted by half the lane width
# toward the lane (to the left of the right wall, to the right of the left).
function _center_from_wall(p::LanePerception, chain, side)
    wx, wy = _smooth([p.xs[i] for i in chain[1:p.cfg.stride:end]], [p.ys[i] for i in chain[1:p.cfg.stride:end]],
        p.cfg.smooth, p.cfg.spacing)
    length(wx) < 2 && return wx, wy
    half = if isnan(p.width)
        # never seen both walls: assume the car started centered
        minimum(hypot(wx[k] - p.x_fa, wy[k]) for k in eachindex(wx))
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
    p.width = sort(p.widths)[cld(length(p.widths), 2)]
    return p
end

# Point of the path at arc length s (clamped to the path).
function _point_at(path::Path, s)
    s = clamp(s, 0.0, path.s[end])
    i = clamp(searchsortedlast(path.s, s), 1, length(path.seg_length))
    t = (s - path.s[i]) / path.seg_length[i]
    return (path.x[i] + t * (path.x[i+1] - path.x[i]), path.y[i] + t * (path.y[i+1] - path.y[i]))
end

# Curvature of the circle through the path's points at s - base/2, s and
# s + base/2 (positive turning left), or NaN when less than half of that
# stretch is on the path. Much less sensitive to noise in the points than
# differencing headings.
function _curvature_at(path::Path, s, base)
    lo, hi = max(s - base / 2, 0.0), min(s + base / 2, path.s[end])
    hi - lo < base / 2 && return NaN
    a, b, c = _point_at(path, lo), _point_at(path, (lo + hi) / 2), _point_at(path, hi)
    cross = (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1])
    return 2cross / (hypot(b[1] - a[1], b[2] - a[2]) * hypot(c[1] - b[1], c[2] - b[2]) * hypot(c[1] - a[1], c[2] - a[2]))
end

"""
    estimate_lane!(p::LanePerception, ranges) -> LaneEstimate

The lane in one scan: `ranges[i]` is the distance measured by beam `i` [m].
"""
function estimate_lane!(p::LanePerception, ranges::AbstractVector{<:Real})
    length(ranges) == length(p.angles) ||
        throw(DimensionMismatch("got $(length(ranges)) ranges for $(length(p.angles)) beams"))
    cfg = p.cfg
    clearance = Inf
    for i in eachindex(ranges)
        x, y = ranges[i] * cos(p.angles[i]), ranges[i] * sin(p.angles[i])
        p.xs[i], p.ys[i] = x, y
        if x > 0 && abs(y) <= cfg.car_halfwidth
            clearance = min(clearance, x - p.x_fa)
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
        # lane width from the pairs beside the front axle
        near = [mw[k] for k in eachindex(mw) if hypot(mx[k] - p.x_fa, my[k]) <= 1.0]
        if !isempty(near)
            _record_width!(p, sort!(near)[cld(length(near), 2)])
        end
    end
    if length(cx) < 3 || _polyline_length(cx, cy) < cfg.min_length
        # One wall (or a broken pair): follow the longer wall at half the lane
        # width. A merged wall is kept in `right`, in counterclockwise order
        # with the lane on its left, like a right wall.
        side = p.merged || length(p.right) > length(p.left) ? :right : :left
        chain = side === :left ? p.left : p.right
        cx, cy = length(chain) >= 2cfg.stride ? _center_from_wall(p, chain, side) : (Float64[], Float64[])
        walls = p.merged ? :merged : side
    end
    p.cx, p.cy = cx, cy
    if length(cx) < 3 || _polyline_length(cx, cy) < cfg.min_length
        return LaneEstimate(0.0, 0.0, 0.0, 0.0, p.width, :none, clearance, fill(NaN, length(_profile_offsets(cfg))))
    end

    center = Path(cx, cy)
    err = tracking_error(center, p.x_fa, 0.0, 0.0)   # the car frame: axle on the x axis, heading 0
    e, ψe = err.e, err.ψe
    dir = 1   # +1 if the lane runs along the path, -1 if against it
    if walls === :merged && abs(wrap_angle(ψe - p.ψe_prev)) > π / 2
        # A wall wrapped around the car does not say which way the lane runs;
        # keep the direction of the previous step (reversing a path flips e
        # and κ and turns its heading by π).
        e, ψe, dir = -e, wrap_angle(ψe + π), -1
    end
    p.ψe_prev = ψe
    κ = _curvature_at(center, err.s, cfg.curve_base)
    κ = isnan(κ) ? 0.0 : dir * κ
    # signed curvature every PROFILE_STEP meters ahead, NaN past what the scan sees
    profile = [dir * _curvature_at(center, err.s + dir * d, cfg.curve_base) for d in _profile_offsets(cfg)]
    return LaneEstimate(e, ψe, κ, _sharpest(profile, κ), p.width, walls, clearance, profile)
end

const PROFILE_STEP = 0.25   # spacing of the curvature profile ahead [m]

_profile_offsets(cfg::PerceptionConfig) = 0.0:PROFILE_STEP:cfg.preview

# Largest |κ| in a profile, ignoring unknown (NaN) entries.
_sharpest(profile, κ) = maximum((abs(k) for k in profile if !isnan(k)); init=abs(κ))
