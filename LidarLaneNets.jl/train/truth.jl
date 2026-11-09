# The true value of every estimate, from the simulator's map and the track's
# centerline: the labels the networks learn, and what the data-collection
# driver steers by.

using F1Tenth
using LidarLaneNets
using StanleyController: Path, tracking_error

# Tracks, split by name. The test tracks are never seen in training; the
# validation tracks choose when to stop training.
const TEST_TRACKS = ["Spa", "Monza", "Silverstone", "Oschersleben", "YasMarina"]
const VAL_TRACKS = ["Hockenheim", "Sepang"]
const TRAIN_TRACKS = sort!(setdiff(collect(keys(F1Tenth.RACETRACKS)), TEST_TRACKS, VAL_TRACKS))

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
    yaml = F1Tenth.racetrack_yaml(name)
    rows = [parse.(Float64, split(l, ",")) for l in eachline(joinpath(dirname(yaml), name * "_centerline.csv"))
            if !startswith(l, "#") && !isempty(strip(l))]
    center = Path(getindex.(rows, 1), getindex.(rows, 2); closed=true)
    return Track(name, F1Tenth.load_map(yaml, ".png"), center, center.s[end] + center.seg_length[end])
end

# Distance from (x, y) to the nearest wall [m] (0 off the map).
function wall_distance(m::F1Tenth.MapData, x, y)
    so, co = sincos(m.origin[3])
    px, py = x - m.origin[1], y - m.origin[2]
    col = floor(Int, (px * co + py * so) / m.resolution)
    row = floor(Int, (-px * so + py * co) / m.resolution)
    h, w = size(m.dt)
    (0 <= col < w && 0 <= row < h) || return 0.0
    return m.dt[row+1, col+1]
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

# March from (x, y) in direction (ux, uy) until a wall (sphere tracing on the
# distance transform); the distance travelled, at most `limit`.
function free_run(m::F1Tenth.MapData, x, y, ux, uy, limit; stop=m.resolution)
    t = 0.0
    while t < limit
        d = wall_distance(m, x + t * ux, y + t * uy)
        d < stop && return t
        t += max(d - stop / 2, m.resolution / 2)
    end
    return limit
end

# Whether the straight line from a to b crosses no wall.
function visible(m::F1Tenth.MapData, a, b)
    D = hypot(b[1] - a[1], b[2] - a[2])
    D < 1e-9 && return true
    return free_run(m, a[1], a[2], (b[1] - a[1]) / D, (b[2] - a[2]) / D, D; stop=0.7m.resolution) >= D
end

# The free distance along the arc of curvature κ leaving (x, y) in direction
# θ, keeping `hw` from the walls, over at most half a turn.
function arc_clearance(m::F1Tenth.MapData, x, y, θ, κ, hw)
    limit = abs(κ) < 1e-6 ? CLEARANCE_MAX : min(CLEARANCE_MAX, π / abs(κ))
    s = 0.0
    while s < limit
        if abs(κ) < 1e-6
            px, py = x + s * cos(θ), y + s * sin(θ)
        else
            R = 1 / κ
            px, py = x - R * sin(θ) + R * sin(θ + κ * s), y + R * cos(θ) - R * cos(θ + κ * s)
        end
        d = wall_distance(m, px, py) - hw
        d <= 0 && return s
        s += max(d, 0.02)   # an arc is at least as long as its chord
    end
    return CLEARANCE_MAX
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
