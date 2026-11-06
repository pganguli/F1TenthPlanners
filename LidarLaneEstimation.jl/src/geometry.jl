# Geometry of the estimated center line: an open polyline in the vehicle
# frame, and where a point is relative to it.

_wrap(a) = rem2pi(Float64(a), RoundNearest)   # to [-π, π]

struct _Polyline
    x::Vector{Float64}
    y::Vector{Float64}
    s::Vector{Float64}             # arc length at each point
    seg_heading::Vector{Float64}
    seg_length::Vector{Float64}
    heading::Vector{Float64}       # at each point: halfway between its segments
end

function _Polyline(x, y)
    n = length(x)
    seg_heading = [atan(y[k+1] - y[k], x[k+1] - x[k]) for k in 1:n-1]
    seg_length = [hypot(x[k+1] - x[k], y[k+1] - y[k]) for k in 1:n-1]
    heading = [k == 1 ? seg_heading[1] : k == n ? seg_heading[n-1] :
               _wrap(seg_heading[k-1] + _wrap(seg_heading[k] - seg_heading[k-1]) / 2) for k in 1:n]
    return _Polyline(x, y, vcat(0.0, cumsum(seg_length)), seg_heading, seg_length, heading)
end

# Where the point (px, py) is relative to the line: signed distance `e` (> 0
# when the line is to the left of a vehicle facing +x), the line's heading `h`
# and the arc length `s` at the nearest point. Past either end, the end
# segment is extended.
function _project(line::_Polyline, px, py)
    m = length(line.seg_length)
    best_d2, i, t = Inf, 0, 0.0
    for k in 1:m
        ax, ay = line.x[k], line.y[k]
        dx, dy = line.x[k+1] - ax, line.y[k+1] - ay
        u = clamp(((px - ax) * dx + (py - ay) * dy) / (dx^2 + dy^2), 0.0, 1.0)
        d2 = (ax + u * dx - px)^2 + (ay + u * dy - py)^2
        if d2 < best_d2
            best_d2, i, t = d2, k, u
        end
    end
    if (i == 1 && t == 0) || (i == m && t == 1)
        h = line.seg_heading[i]
        dx, dy = px - line.x[i], py - line.y[i]
        return (e=sin(h) * dx - cos(h) * dy, h=h, s=line.s[i] + cos(h) * dx + sin(h) * dy)
    end
    qx = line.x[i] + t * (line.x[i+1] - line.x[i])
    qy = line.y[i] + t * (line.y[i+1] - line.y[i])
    h = _wrap(line.heading[i] + t * _wrap(line.heading[i+1] - line.heading[i]))
    side = cos(h) * (qy - py) - sin(h) * (qx - px)
    return (e=sign(side) * sqrt(best_d2), h=h, s=line.s[i] + t * line.seg_length[i])
end

# Point of the line at arc length s (clamped to the line).
function _point_at(line::_Polyline, s)
    s = clamp(s, 0.0, line.s[end])
    i = clamp(searchsortedlast(line.s, s), 1, length(line.seg_length))
    t = (s - line.s[i]) / line.seg_length[i]
    return (line.x[i] + t * (line.x[i+1] - line.x[i]), line.y[i] + t * (line.y[i+1] - line.y[i]))
end

# Curvature of the circle through the line's points at s - base/2, s and
# s + base/2 (positive turning left), or NaN when less than half of that
# stretch is on the line. Much less sensitive to noise in the points than
# differencing headings.
function _curvature_at(line::_Polyline, s, base)
    lo, hi = max(s - base / 2, 0.0), min(s + base / 2, line.s[end])
    hi - lo < base / 2 && return NaN
    a, b, c = _point_at(line, lo), _point_at(line, (lo + hi) / 2), _point_at(line, hi)
    cross = (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1])
    return 2cross / (hypot(b[1] - a[1], b[2] - a[2]) * hypot(c[1] - b[1], c[2] - b[2]) * hypot(c[1] - a[1], c[2] - a[2]))
end
