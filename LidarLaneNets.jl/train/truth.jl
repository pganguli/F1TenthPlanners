# The true value of every estimate, from the simulator's map and the track's
# centerline: the labels the networks learn, and what the data-collection
# driver steers by.

include("common.jl")   # the tracks, their walls, the data
using StanleyController: Path, tracking_error

# Curvature of the true center line: the circle through points this far apart [m].
const CURVE_BASE = 1.0

"""
    Track(name)

A racetrack's map and closed centerline.
"""
struct Track
    name::String
    map::F1Tenth.MapData
    center::Path
    length::Float64
end

function Track(name::AbstractString)
    center = centerline(name)
    return Track(name, F1Tenth.load_map(F1Tenth.racetrack_yaml(name), ".png"), center, center.s[end] + center.seg_length[end])
end

# Point of the closed centerline at arc length s.
function center_point(tr::Track, s)
    c = tr.center
    s = mod(s, tr.length)
    i = searchsortedlast(c.s, s)
    j = mod1(i + 1, length(c.x))
    t = (s - c.s[i]) / c.seg_length[i]
    return (c.x[i] + t * (c.x[j] - c.x[i]), c.y[i] + t * (c.y[j] - c.y[i]))
end

function center_heading(tr::Track, s)
    a, b = center_point(tr, s - 0.25), center_point(tr, s + 0.25)
    return atan(b[2] - a[2], b[1] - a[1])
end

function center_curvature(tr::Track, s)
    a, b, c = center_point(tr, s - CURVE_BASE / 2), center_point(tr, s), center_point(tr, s + CURVE_BASE / 2)
    cross = (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1])
    return 2cross / (hypot(b[1] - a[1], b[2] - a[2]) * hypot(c[1] - b[1], c[2] - b[2]) * hypot(c[1] - a[1], c[2] - a[2]))
end

# Whether the straight line from a to b crosses no wall.
function visible(m::F1Tenth.MapData, a, b)
    D = hypot(b[1] - a[1], b[2] - a[2])
    D < 1e-9 && return true
    return free_run(m, a[1], a[2], (b[1] - a[1]) / D, (b[2] - a[2]) / D, D; stop=0.7m.resolution) >= D
end

"""
    truth(tr, lidar_pose, x_ref; hw, scan_range, hint=0) -> (labels, segment) or nothing

The true estimates for a scan taken from `lidar_pose` `(x, y, θ)`, with the
front axle `x_ref` meters ahead of the LiDAR and the car `2hw` wide;
`nothing` if the front axle is off the track. `segment` (the centerline
segment beside the car) speeds up the next call when passed as `hint`.
"""
function truth(tr::Track, (lx, ly, θ), x_ref; hw, scan_range, hint=0)
    m = tr.map
    fx, fy = lx + x_ref * cos(θ), ly + x_ref * sin(θ)
    wall_distance(m, fx, fy) < m.resolution && return nothing
    err = tracking_error(tr.center, fx, fy, θ; hint, window=hint == 0 ? 0 : 40)
    s0 = err.s
    κp = [center_curvature(tr, s0 + d) for d in PROFILE_S]
    wl, wr = similar(κp), similar(κp)
    for (j, d) in enumerate(PROFILE_S)
        px, py = center_point(tr, s0 + d)
        h = center_heading(tr, s0 + d)
        wl[j] = free_run(m, px, py, -sin(h), cos(h), WIDTH_MAX)
        wr[j] = free_run(m, px, py, sin(h), -cos(h), WIDTH_MAX)
    end
    sight = SIGHT_MAX
    for d in PROFILE_STEP:PROFILE_STEP:SIGHT_MAX
        p = center_point(tr, s0 + d)
        if hypot(p[1] - lx, p[2] - ly) > scan_range || !visible(m, (lx, ly), p)
            sight = d - PROFILE_STEP
            break
        end
    end
    clearance = [arc_clearance(m, fx, fy, θ, κ, hw) for κ in CLEARANCE_K]
    labels = lane_labels(; e=err.e, ψe=err.ψe, κ_profile=κp, w_left=wl, w_right=wr, sight, clearance)
    return labels, err.segment
end
