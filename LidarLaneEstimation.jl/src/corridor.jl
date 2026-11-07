# The free corridor around the lane center, for a path planner
#
# A planner that moves the lane center sideways (a minimum-curvature planner,
# say) needs to know how far each point may go. Resample the center to points
# `step` apart, with unit normals pointing left; for each point, the shifts
# along its normal that keep at least `margin` from every return in the scan.
# Where no return limits a point, it may move half the lane width from the
# center (the lane continues past what the scan shows).

"""
    lane_corridor(p::LanePerception, margin; step=0.2) -> NamedTuple or nothing

The corridor around the lane center last found by [`detect_lane!`](@ref):
the center resampled to points `step` meters apart (`x`, `y`, LiDAR frame),
their unit normals to the left (`nx`, `ny`), and the shifts along those
normals that keep at least `margin` meters from every return of the scan
(`lower[i] ≤ α ≤ upper[i]`, with `lower[i] ≤ 0 ≤ upper[i]`). `nothing` when no
lane was found or it is too short for three points.
"""
function lane_corridor(p::LanePerception, margin::Real; step::Real=0.2)
    cfg = p.cfg
    p.walls === :none && return nothing
    x, y = _resample(p.center_x, p.center_y, step)
    n = length(x)
    n < 3 && return nothing
    nx, ny = similar(x), similar(y)   # unit normals, to the left
    for i in 1:n
        a, b = max(1, i - 1), min(n, i + 1)
        h = atan(y[b] - y[a], x[b] - x[a])
        nx[i], ny[i] = -sin(h), cos(h)
    end
    m = margin
    cap = isnan(p.width) ? 0.0 : max(p.width / 2 - m, 0.0)
    upper, lower = fill(Inf, n), fill(-Inf, n)
    blocked = falses(n)
    @inbounds for j in 1:cfg.stride:length(p.xs)
        hypot(p.xs[j], p.ys[j]) > cfg.r_max && continue
        for i in 1:n
            dx, dy = p.xs[j] - x[i], p.ys[j] - y[i]
            t = dx * nx[i] + dy * ny[i]          # along the normal
            off = abs(dx * ny[i] - dy * nx[i])   # across it
            off >= m && continue
            s = sqrt(m^2 - off^2)
            # the return rules out α in (t - s, t + s)
            if t - s >= 0
                upper[i] = min(upper[i], t - s)
            elseif t + s <= 0
                lower[i] = max(lower[i], t + s)
            else
                blocked[i] = true                # the center itself is too close
            end
        end
    end
    for i in 1:n
        upper[i] = blocked[i] ? 0.0 : isinf(upper[i]) ? cap : upper[i]
        lower[i] = blocked[i] ? 0.0 : isinf(lower[i]) ? -cap : lower[i]
    end
    return (; x, y, nx, ny, lower, upper)
end

# Points of the polyline (x, y) every `step` meters, from its start to its end.
function _resample(x, y, step)
    line = _Polyline(x, y)
    n = max(2, round(Int, line.s[end] / step) + 1)
    pts = [_point_at(line, s) for s in range(0.0, line.s[end]; length=n)]
    return first.(pts), last.(pts)
end
