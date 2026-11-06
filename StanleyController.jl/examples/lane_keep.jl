# Lane keeping on F1Tenth maps with the Stanley controller, using only the LiDAR.
#
# The walls are the lane boundaries. Each step, the scan is turned into the
# two numbers the Stanley law needs, measured at the front axle:
#
#   e   cross-track error: how far the lane center is to the left [m]
#   ψe  heading error: the lane direction relative to the car [rad]
#
# plus the lane curvature ahead and the free distance straight ahead, which
# set the speed (a fixed cruise speed, lower in turns). An unscented Kalman
# filter combines each scan with the earlier ones and the car's own speed and
# yaw rate, which removes most of the scan noise. The controller never
# sees the car's pose or the map. For the f1tenth_racetracks maps the track's
# centerline file is used only to place the car at the start line and to
# score how well it stays centered.
#
#     julia --project=examples examples/lane_keep.jl Spa             # one lap, prints a score
#     julia --project=examples examples/lane_keep.jl Spa --render    # watch it (needs a display)
#     julia --project=examples examples/lane_keep.jl all             # every racetrack
#
# The first time, set up the examples environment with
#     julia --project=examples -e 'using Pkg; Pkg.instantiate()'

using F1Tenth
using KalmanFilters
using LinearAlgebra
using StanleyController
using Printf

const USAGE = """
usage: lane_keep.jl MAP [options]

MAP is a racetrack name (Austin, Sochi, Spa, Shanghai, ...), "all" for every
racetrack, or any ROS map yaml path (without .yaml) together with --pose.

options:
  --speed V        cruise speed in m/s (default 5.0)
  --laps N         laps to drive (default 1)
  --pose X Y THETA start pose; required for maps that are not racetracks
  --map-ext EXT    image extension of a custom map (default .png)
  --render         open the render window (needs a display)
"""

# ---------------------------------------------------------------------------
# LiDAR lane perception
#
# 1. Scan to points in the car frame.
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
    # Lane width = median of this many steps' measurements (100 per second).
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

What the perception hands to the controller: cross-track error `e`, heading
error `ψe` and lane curvature `κ` at the front axle, the largest curvature in
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

function LanePerception(num_beams, fov, x_fa; cfg=PerceptionConfig())
    angles = [-fov / 2 + (i - 1) * fov / (num_beams - 1) for i in 1:num_beams]
    return LanePerception(cfg, angles, x_fa, zeros(num_beams), zeros(num_beams), Int[], Int[],
        Float64[], Float64[], Float64[], 1, NaN, false, 0.0)
end

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

Turn one scan into the Stanley inputs and the speed inputs.
"""
function estimate_lane!(p::LanePerception, ranges)
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

# ---------------------------------------------------------------------------
# Smoothing over time: an unscented Kalman filter
#
# Each scan's estimate is noisy (a few degrees of heading error), and the
# Stanley law passes heading noise straight to the wheels. At speed the car
# twitches, because the sideways jolt grows with v². Plain averaging over
# scans would delay turns. Instead a Kalman filter tracks the lane state
# x = (e, ψe, κ) with two steps per scan:
#
#   predict  how x changes until this scan, from the car's speed v and yaw
#            rate r, which are measured and far less noisy than the scan:
#              de/dt  = v⋅sin(ψe) − L⋅r⋅cos(ψe)   (L: wheelbase, no slip)
#              dψe/dt = κ⋅v − r
#              dκ/dt  = 0                        (changes count as noise)
#   correct  move part of the way toward the scan. The filter works out how
#            far from how uncertain the prediction is compared with the scan.
#
# The prediction is nonlinear (the sines and cosines), so it uses the
# unscented Kalman filter (Wan and van der Merwe, 2000) from KalmanFilters.jl:
# a few "sigma points" spread around the estimate are pushed through the
# model, and their spread after it gives the new uncertainty.
#
# Turning into a corner shows up in r first, so it is predicted, not delayed.
# A scan far from the prediction (measured in standard deviations) is
# treated as an outlier and ignored. If that keeps happening, the lane really
# changed (for example, a wall came into view) and the filter restarts from
# the scan.
#
# The curvature ahead, which sets the speed, is averaged over scans too. Each
# point of it stays where it is on the ground while the car drives past.

"""
    FilterConfig(; kwargs...)

Noise levels for the Kalman filter. Distances in meters, angles in radians.
"""
Base.@kwdef struct FilterConfig
    # Noise of one scan's estimate (standard deviations). Larger trusts the
    # scans less: smoother, but slower to correct the prediction.
    σ_e::Float64 = 0.03      # [m]
    σ_ψ::Float64 = 0.06      # [rad]
    σ_κ::Float64 = 0.03      # [1/m]
    # What the prediction can't explain, as rates. Larger follows the scans
    # more closely, with less smoothing.
    q_e::Float64 = 0.3       # sideways speed from wheel slip [m/s]
    q_ψ::Float64 = 0.5       # heading rate from slip [rad/s]
    q_κ::Float64 = 0.5       # change of the lane's curvature per meter driven [1/m²]
    # A scan more than this many standard deviations from the prediction is
    # an outlier; after max_outliers in a row the filter restarts from the scan.
    gate::Float64 = 4.0
    max_outliers::Int = 3
    # Weight of each new scan in the curvature ahead. Smaller removes more of
    # the noise that slows the car on straights but registers a gentle turn
    # later. A point that changes by more than gate_κ [1/m] (a sharp turn
    # coming into view) is taken as it is.
    curve_gain::Float64 = 0.25
    gate_κ::Float64 = 0.15
end

mutable struct LaneFilter
    cfg::FilterConfig
    wheelbase::Float64            # rear to front axle [m]
    ready::Bool
    x::Vector{Float64}            # e, ψe, κ (ψe not wrapped, so the sigma points stay continuous)
    P::Matrix{Float64}            # their covariance
    outliers::Int                 # outliers in a row
    profile::Vector{Float64}      # signed curvature ahead, as in LaneEstimate
end

LaneFilter(wheelbase; cfg=FilterConfig()) = LaneFilter(cfg, wheelbase, false, zeros(3), zeros(3, 3), 0, Float64[])

_scan_noise(c::FilterConfig) = Diagonal([c.σ_e, c.σ_ψ, c.σ_κ] .^ 2)

function _restart!(f::LaneFilter, lane::LaneEstimate)
    f.x = [lane.e, lane.ψe, lane.κ]
    f.P = Matrix(_scan_noise(f.cfg))
    f.profile = copy(lane.κ_profile)
    f.outliers = 0
    f.ready = true
    return f
end

"""
    track!(f::LaneFilter, lane::LaneEstimate, v, yaw_rate, dt) -> LaneEstimate

`lane` combined with the earlier scans. `v` and `yaw_rate` are the car's
speed [m/s] and yaw rate [rad/s], and `dt` the time since the last scan [s].
"""
function track!(f::LaneFilter, lane::LaneEstimate, v, yaw_rate, dt)
    c = f.cfg
    if !f.ready
        lane.walls === :none || _restart!(f, lane)
        return lane
    end
    # predict
    L = f.wheelbase
    motion((e, ψe, κ)) = [e + (v * sin(ψe) - L * yaw_rate * cos(ψe)) * dt, ψe + (κ * v - yaw_rate) * dt, κ]
    Q = Diagonal([c.q_e * dt, c.q_ψ * dt, c.q_κ * abs(v) * dt] .^ 2)
    tu = time_update(f.x, f.P, motion, Q)
    x, P = get_state(tu), get_covariance(tu)
    profile = _shift(f.profile, v * dt)   # the car moved forward along the lane
    if lane.walls !== :none               # correct (with no lane in view, keep the prediction)
        # the scan measures x directly; its heading is unwrapped to be near the prediction
        z = [lane.e, x[2] + wrap_angle(lane.ψe - x[2]), lane.κ]
        mu = measurement_update(x, P, z, Matrix(1.0I, 3, 3), _scan_noise(c))
        if calc_nis(mu) > c.gate^2        # squared distance in standard deviations
            f.outliers += 1
            if f.outliers >= c.max_outliers
                _restart!(f, lane)
                return lane
            end
        else
            f.outliers = 0
            x, P = get_state(mu), get_covariance(mu)
        end
        for j in eachindex(profile)
            z = lane.κ_profile[j]
            isnan(z) && continue                         # not in this scan: keep the old value
            profile[j] = isnan(profile[j]) || abs(z - profile[j]) > c.gate_κ ? z :
                         profile[j] + c.curve_gain * (z - profile[j])
        end
    end
    f.x, f.P, f.profile = x, P, profile
    ψe = wrap_angle(x[2])
    return LaneEstimate(x[1], ψe, x[3], _sharpest(profile, x[3]), lane.width, lane.walls, lane.clearance, profile)
end

# A profile sampled every PROFILE_STEP meters, moved d meters toward the car
# (linear interpolation; NaN where nothing was known).
function _shift(profile, d)
    n = length(profile)
    out = fill(NaN, n)
    for j in 1:n
        u = 1 + (j - 1) + d / PROFILE_STEP   # position in the old profile
        i = floor(Int, u)
        i > n && break
        a, b = profile[i], profile[min(i + 1, n)]
        out[j] = isnan(a) ? b : isnan(b) ? a : a + (u - i) * (b - a)
    end
    return out
end

# ---------------------------------------------------------------------------
# Speed

"""
    SpeedConfig(; kwargs...)

Cruise speed and the limits that lower it.
"""
Base.@kwdef struct SpeedConfig
    cruise::Float64 = 5.0     # the fixed speed on straights [m/s]
    v_min::Float64 = 1.0      # never slower than this [m/s]
    a_lat::Float64 = 4.0      # lateral acceleration allowed in turns [m/s²]
    a_brake::Float64 = 4.0    # deceleration assumed for the free distance ahead [m/s²]
    margin::Float64 = 0.5     # distance to keep from a wall straight ahead [m]
end

function target_speed(cfg::SpeedConfig, lane::LaneEstimate)
    v = curvature_speed(lane.κ_ahead; v_max=cfg.cruise, a_lat=cfg.a_lat, v_min=cfg.v_min)
    v_clear = sqrt(cfg.v_min^2 + 2cfg.a_brake * max(lane.clearance - cfg.margin, 0.0))
    return clamp(min(v, v_clear), cfg.v_min, cfg.cruise)
end

# ---------------------------------------------------------------------------
# Running a map

"""
    centerline(name) -> Union{Path,Nothing}

The racetrack's centerline (used only for the start pose and the score).
"""
function centerline(name)
    haskey(F1Tenth.RACETRACKS, name) || return nothing
    dir = dirname(F1Tenth.racetrack_yaml(name))
    rows = [parse.(Float64, split(l, ",")) for l in eachline(joinpath(dir, name * "_centerline.csv"))
            if !startswith(l, "#") && !isempty(strip(l))]
    return Path(getindex.(rows, 1), getindex.(rows, 2); closed=true)
end

"""
    RunResult

Outcome of one run. Offsets are distances of the car's reference point from
the track centerline; perception errors compare the LiDAR estimates with
the true errors at the front axle. Map-based numbers are NaN without a
centerline.
"""
struct RunResult
    map::String
    finished::Bool
    crashed::Bool
    sim_time::Float64
    mean_offset::Float64
    max_offset::Float64
    mean_speed::Float64
    e_rmse::Float64
    ψe_rmse::Float64
    single_wall::Float64      # fraction of steps that used one wall or none
    wall_time::Float64
end

default_stanley() = Stanley(k=6.0, k_soft=1.0, k_yaw=0.05, max_steer=VehicleParams().s_max)

function run_map(map_name; speed=5.0, laps=1, pose=nothing, map_ext=".png", render=false,
                 stanley=default_stanley(), perception=PerceptionConfig(), smoothing=FilterConfig(),
                 max_time=600.0)
    params = VehicleParams()
    path = centerline(map_name)
    if pose === nothing
        path === nothing && error("$map_name is not a racetrack; give a start pose with --pose X Y THETA")
        pose = (path.x[1], path.y[1], path.heading[1])
    end
    dt = 0.01
    env = F110Env(map=map_name, map_ext=map_ext, num_agents=1, timestep=dt, integrator=Integrator.RK4)
    perceive = LanePerception(1080, 4.7, params.lf - env.lidar_dist; cfg=perception)
    tracker = LaneFilter(params.lf + params.lr; cfg=smoothing)
    speed_cfg = SpeedConfig(cruise=speed)

    render && add_render_callback!(env, e -> render_callback(e, env, perceive))
    obs, _, done, _ = reset!(env, [pose[1] pose[2] pose[3]])
    render && render!(env)

    t, dist, steps, single = 0.0, 0.0, 0, 0
    sum_off, max_off, sum_e2, sum_ψ2 = 0.0, 0.0, 0.0, 0.0
    hint = 0
    start = time()
    while !done && obs.lap_counts[1] < laps && t < max_time
        x, y, θ = obs.poses_x[1], obs.poses_y[1], obs.poses_theta[1]
        v, r = obs.linear_vels_x[1], obs.ang_vels_z[1]

        lane = track!(tracker, estimate_lane!(perceive, obs.scans[1]), v, r, dt)
        steer = steering_angle(stanley, lane.e, lane.ψe, v; yaw_rate=r, path_yaw_rate=lane.κ * v)
        obs, step_time, done, _ = step!(env, [steer target_speed(speed_cfg, lane)])
        render && render!(env; mode=:human_fast)

        t += step_time
        steps += 1
        dist += abs(v) * step_time
        lane.walls === :both || (single += 1)
        if path !== nothing   # score against the true centerline
            truth = tracking_error(path, x, y, θ; hint, window=hint == 0 ? 0 : 30)
            hint = truth.segment
            sum_off += abs(truth.e)
            max_off = max(max_off, abs(truth.e))
            fa = tracking_error(path, front_axle(x, y, θ, params.lf)..., θ; hint, window=30)
            sum_e2 += (lane.e - fa.e)^2
            sum_ψ2 += wrap_angle(lane.ψe - fa.ψe)^2
        end
    end
    crashed = obs.collisions[1] > 0
    n = max(steps, 1)
    known(x) = path === nothing ? NaN : x
    return RunResult(basename(string(map_name)), obs.lap_counts[1] >= laps && !crashed, crashed, t,
        known(sum_off / n), known(max_off), dist / max(t, eps()), known(sqrt(sum_e2 / n)),
        known(sqrt(sum_ψ2 / n)), single / n, time() - start)
end

function print_header()
    @printf("%-14s %-8s %8s %7s %9s %9s %9s %8s %9s %7s\n", "map", "result", "time[s]", "v[m/s]",
        "|off|[m]", "max|off|", "e_err[m]", "ψe_err°", "1-wall[%]", "wall[s]")
end

function print_result(r::RunResult)
    status = r.finished ? "lap" : r.crashed ? "CRASH" : "timeout"
    @printf("%-14s %-8s %8.2f %7.2f %9.3f %9.3f %9.3f %8.2f %9.1f %7.1f\n", r.map, status, r.sim_time,
        r.mean_speed, r.mean_offset, r.max_offset, r.e_rmse, rad2deg(r.ψe_rmse), 100r.single_wall, r.wall_time)
end

# ---------------------------------------------------------------------------
# Rendering (only with --render; GLMakie is loaded on demand)

const _drawn = Ref{Any}(nothing)

function render_callback(e, env, perceive)
    GL = Main.GLMakie
    # camera follows the car, as in waypoint_follow.jl
    xs = [p[1] for p in e.cars[1][]]
    ys = [p[2] for p in e.cars[1][]]
    top, bottom, left, right = maximum(ys), minimum(ys), minimum(xs), maximum(xs)
    e.score_label.x = left
    e.score_label.y = top - 700
    e.left, e.right, e.top, e.bottom = left - 800, right + 800, top + 800, bottom - 800

    # the lane center the LiDAR sees, in orange (renderer units are 50 × meters)
    st = env.sim.agents[1].state
    θ = st[5]
    ox, oy = front_axle(st[1], st[2], θ, env.lidar_dist)   # scan origin
    pts = GL.Point2f[GL.Point2f(50 * (ox + x * cos(θ) - y * sin(θ)), 50 * (oy + x * sin(θ) + y * cos(θ)))
           for (x, y) in zip(perceive.cx, perceive.cy)]
    if _drawn[] === nothing
        _drawn[] = GL.Observable(pts)
        GL.lines!(e.axis, _drawn[]; color=:orange, linewidth=3)
    else
        _drawn[][] = pts
    end
    return nothing
end

# ---------------------------------------------------------------------------

function parse_args(args)
    isempty(args) && (print(USAGE); exit(1))
    opts = Dict{Symbol,Any}(:map => args[1], :speed => 5.0, :laps => 1, :pose => nothing,
        :map_ext => ".png", :render => false)
    i = 2
    while i <= length(args)
        a = args[i]
        if a == "--speed"
            opts[:speed] = parse(Float64, args[i+1]); i += 2
        elseif a == "--laps"
            opts[:laps] = parse(Int, args[i+1]); i += 2
        elseif a == "--pose"
            opts[:pose] = Tuple(parse.(Float64, args[i+1:i+3])); i += 4
        elseif a == "--map-ext"
            opts[:map_ext] = args[i+1]; i += 2
        elseif a == "--render"
            opts[:render] = true; i += 1
        else
            print(USAGE); error("unknown option $a")
        end
    end
    return opts
end

function main(args=ARGS)
    o = parse_args(args)
    if o[:map] == "all"
        print_header()
        for name in sort!(collect(keys(F1Tenth.RACETRACKS)))
            print_result(run_map(name; speed=o[:speed], laps=o[:laps]))
        end
    else
        o[:render] && @eval Main using GLMakie
        r = Base.invokelatest(run_map, o[:map]; speed=o[:speed], laps=o[:laps], pose=o[:pose],
            map_ext=o[:map_ext], render=o[:render])
        print_header()
        print_result(r)
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
