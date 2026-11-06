"""
    StanleyController

The Stanley steering law for path tracking (Hoffmann, Tomlin, Montemerlo and
Thrun, 2007; Section 2.3 of Snider, "Automatic Steering Methods for
Autonomous Automobile Path Tracking", 2009), with nothing tied to a
particular vehicle, simulator or sensor.

The law needs two errors, measured at the center of the front axle:

- `e`: cross-track error, the signed distance to the nearest point of the
  path. `e > 0` when the path is to the **left** of the axle.
- `ψe`: heading error, the path's heading minus the vehicle's heading,
  wrapped to `[-π, π]`. `ψe > 0` when the path points to the **left**.

Angles are counterclockwise-positive and `δ > 0` steers left. Get the errors
from a known path with [`Path`](@ref) and [`tracking_error`](@ref), or
estimate them some other way (for example from a sensor) and pass them to
[`steering_angle`](@ref) directly.
"""
module StanleyController

export Stanley, steering_angle, TrackingError, Path, tracking_error, front_axle, curvature_speed, wrap_angle

"""
    wrap_angle(a) -> Float64

`a` wrapped to `[-π, π]`.
"""
wrap_angle(a::Real) = rem2pi(Float64(a), RoundNearest)

"""
    Stanley(; k=2.0, k_soft=1.0, k_yaw=0.0, max_steer=Inf)

Stanley steering law

    δ = ψe + atan(k ⋅ e / (k_soft + |v|)) − k_yaw ⋅ (r − r_path)

- `k`: cross-track gain [1/s]. The wheels aim at a point on the path tangent
  `|v| / k` meters ahead of the nearest path point.
- `k_soft`: softening speed [m/s], so the law stays gentle near standstill.
  `k_soft = 0` gives the law exactly as in Snider (2009), Eq. 5.
- `k_yaw`: yaw-rate damping gain [s] (Hoffmann et al., 2007). `r` is the
  measured yaw rate and `r_path` the yaw rate the path asks for (curvature ×
  speed). Off by default.
- `max_steer`: steering limit [rad]; the result is clamped to `±max_steer`.
"""
struct Stanley
    k::Float64
    k_soft::Float64
    k_yaw::Float64
    max_steer::Float64
    function Stanley(k, k_soft, k_yaw, max_steer)
        k > 0 || throw(ArgumentError("k must be positive, got $k"))
        k_soft >= 0 || throw(ArgumentError("k_soft must be non-negative, got $k_soft"))
        k_yaw >= 0 || throw(ArgumentError("k_yaw must be non-negative, got $k_yaw"))
        max_steer > 0 || throw(ArgumentError("max_steer must be positive, got $max_steer"))
        return new(k, k_soft, k_yaw, max_steer)
    end
end

Stanley(; k=2.0, k_soft=1.0, k_yaw=0.0, max_steer=Inf) = Stanley(k, k_soft, k_yaw, max_steer)

"""
    TrackingError(e, ψe, κ, s, segment)

Where a point is relative to a path: cross-track error `e`, heading error
`ψe` (see [`StanleyController`](@ref) for signs), the path's curvature `κ`
[1/m] (`> 0` turning left) and arc length `s` at the nearest point, and the
index of the segment that point lies on.
"""
struct TrackingError
    e::Float64
    ψe::Float64
    κ::Float64
    s::Float64
    segment::Int
end

"""
    steering_angle(c::Stanley, e, ψe, v; yaw_rate=0.0, path_yaw_rate=0.0) -> δ
    steering_angle(c::Stanley, err::TrackingError, v; yaw_rate=0.0) -> δ

Steering angle [rad] for cross-track error `e` [m], heading error `ψe` [rad]
and forward speed `v` [m/s]. `yaw_rate` and `path_yaw_rate` [rad/s] are only
used when `c.k_yaw > 0`; with a `TrackingError`, `path_yaw_rate = err.κ ⋅ v`.
"""
function steering_angle(c::Stanley, e::Real, ψe::Real, v::Real; yaw_rate::Real=0.0, path_yaw_rate::Real=0.0)
    # Two-argument atan keeps the sign of e when k_soft = 0 and v = 0.
    δ = wrap_angle(ψe) + atan(c.k * e, c.k_soft + abs(v))
    δ -= c.k_yaw * (yaw_rate - path_yaw_rate)
    return clamp(δ, -c.max_steer, c.max_steer)
end

steering_angle(c::Stanley, err::TrackingError, v::Real; yaw_rate::Real=0.0) =
    steering_angle(c, err.e, err.ψe, v; yaw_rate, path_yaw_rate=err.κ * v)

"""
    front_axle(x, y, θ, d) -> (x, y)

The point `d` meters ahead of `(x, y)` along heading `θ`: the front axle,
when `(x, y)` is the vehicle's reference point and `d` the distance from it
to the front axle.
"""
front_axle(x::Real, y::Real, θ::Real, d::Real) = (x + d * cos(θ), y + d * sin(θ))

"""
    curvature_speed(κ; v_max, a_lat, v_min=0.0) -> v

The fastest speed that keeps lateral acceleration `v² |κ|` at or below
`a_lat` [m/s²] on a curve of curvature `κ` [1/m], clamped to
`[v_min, v_max]`.
"""
function curvature_speed(κ::Real; v_max::Real, a_lat::Real, v_min::Real=0.0)
    a_lat > 0 || throw(ArgumentError("a_lat must be positive, got $a_lat"))
    v_min <= v_max || throw(ArgumentError("v_min ($v_min) must not exceed v_max ($v_max)"))
    iszero(κ) && return Float64(v_max)
    return clamp(sqrt(a_lat / abs(κ)), v_min, v_max)
end

"""
    Path(x, y; closed=false)

A path through the points `(x[i], y[i])`, joined by straight segments. A
`closed` path also joins the last point back to the first (a repeated final
point is dropped). Headings and curvatures are estimated at the points and
interpolated along the segments.
"""
struct Path
    x::Vector{Float64}
    y::Vector{Float64}
    heading::Vector{Float64}     # at each point
    curvature::Vector{Float64}   # at each point
    s::Vector{Float64}           # arc length at each point
    seg_heading::Vector{Float64}
    seg_length::Vector{Float64}
    closed::Bool
end

function Path(x::AbstractVector{<:Real}, y::AbstractVector{<:Real}; closed::Bool=false)
    length(x) == length(y) || throw(DimensionMismatch("x and y have different lengths"))
    xs, ys = collect(Float64, x), collect(Float64, y)
    if closed && length(xs) > 2 && xs[end] == xs[1] && ys[end] == ys[1]
        pop!(xs)
        pop!(ys)
    end
    n = length(xs)
    n >= (closed ? 3 : 2) || throw(ArgumentError("a $(closed ? "closed" : "open") path needs at least $(closed ? 3 : 2) distinct points"))
    m = closed ? n : n - 1
    seg_heading = Vector{Float64}(undef, m)
    seg_length = Vector{Float64}(undef, m)
    for i in 1:m
        j = mod1(i + 1, n)
        dx, dy = xs[j] - xs[i], ys[j] - ys[i]
        seg_length[i] = hypot(dx, dy)
        seg_length[i] > 0 || throw(ArgumentError("points $i and $j are the same"))
        seg_heading[i] = atan(dy, dx)
    end
    heading = Vector{Float64}(undef, n)
    curvature = Vector{Float64}(undef, n)
    for i in 1:n
        if !closed && (i == 1 || i == n)
            heading[i] = seg_heading[i == 1 ? 1 : m]
            continue
        end
        a, b = mod1(i - 1, m), i <= m ? i : 1   # segments into and out of point i
        turn = wrap_angle(seg_heading[b] - seg_heading[a])
        heading[i] = wrap_angle(seg_heading[a] + turn / 2)
        curvature[i] = turn / ((seg_length[a] + seg_length[b]) / 2)
    end
    if !closed
        curvature[1] = n > 2 ? curvature[2] : 0.0
        curvature[n] = n > 2 ? curvature[n-1] : 0.0
    end
    s = zeros(n)
    for i in 2:n
        s[i] = s[i-1] + seg_length[i-1]
    end
    return Path(xs, ys, heading, curvature, s, seg_heading, seg_length, closed)
end

Base.length(p::Path) = length(p.x)

"""
    nsegments(p::Path) -> Int

Number of segments: one per point on a closed path, one fewer on an open one.
"""
nsegments(p::Path) = length(p.seg_length)

"""
    total_length(p::Path) -> Float64

Arc length of the whole path (including the closing segment if `closed`).
"""
total_length(p::Path) = sum(p.seg_length)

"""
    tracking_error(path, x, y, θ; hint=0, window=0) -> TrackingError

Errors of a vehicle with control point `(x, y)` (for the Stanley law, its
front axle) and heading `θ`, measured against the nearest point of `path`.
By default every segment is searched; with `hint > 0` and `window > 0`
only the segments within `window` of segment `hint` are, which is faster
and avoids jumping to another part of the path that passes close by.

Past either end of an open path, the end segment is extended: `e` is the
distance to that line and `s` is negative before the start or more than the
path's length after its end.
"""
function tracking_error(path::Path, x::Real, y::Real, θ::Real; hint::Integer=0, window::Integer=0)
    m, n = nsegments(path), length(path)
    segs = if hint > 0 && window > 0 && 2window + 1 < m
        path.closed ? (mod1(hint + k, m) for k in -window:window) : (max(1, hint - window):min(m, hint + window))
    else
        1:m
    end
    best_d2, best_i, best_t = Inf, 0, 0.0
    for i in segs
        j = mod1(i + 1, n)
        ax, ay = path.x[i], path.y[i]
        dx, dy = path.x[j] - ax, path.y[j] - ay
        t = clamp(((x - ax) * dx + (y - ay) * dy) / (dx^2 + dy^2), 0.0, 1.0)
        d2 = (ax + t * dx - x)^2 + (ay + t * dy - y)^2
        if d2 < best_d2
            best_d2, best_i, best_t = d2, i, t
        end
    end
    i, t = best_i, best_t
    j = mod1(i + 1, n)
    if !path.closed && ((i == 1 && t == 0) || (i == m && t == 1))
        # Beyond an end of an open path: extend the end segment, so `e` is the
        # perpendicular distance and `s` runs past the end.
        h = path.seg_heading[i]
        dx, dy = x - path.x[i], y - path.y[i]
        e = sin(h) * dx - cos(h) * dy
        s = path.s[i] + cos(h) * dx + sin(h) * dy
        return TrackingError(e, wrap_angle(h - θ), path.curvature[i == 1 ? 1 : n], s, i)
    end
    px = path.x[i] + t * (path.x[j] - path.x[i])
    py = path.y[i] + t * (path.y[j] - path.y[i])
    h = wrap_angle(path.heading[i] + t * wrap_angle(path.heading[j] - path.heading[i]))
    κ = (1 - t) * path.curvature[i] + t * path.curvature[j]
    s = path.s[i] + t * path.seg_length[i]
    # Positive when the path point is to the left of the control point.
    side = cos(h) * (py - y) - sin(h) * (px - x)
    e = sign(side) * sqrt(best_d2)
    return TrackingError(e, wrap_angle(h - θ), κ, s, i)
end

end # module
