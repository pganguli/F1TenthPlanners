# The racetracks: which are for training, validation and testing, their
# centerlines, and the distances to their walls on the map. Shared by the
# training scripts and by LidarRacer.jl's race scripts (which score runs
# from the map), so both use the same split and the same geometry.

using F1Tenth
using LidarLaneNets: CLEARANCE_MAX
using StanleyController: Path

# Tracks, split by name. The test tracks are never seen in training; the
# validation tracks choose when to stop training.
const TEST_TRACKS = ["Spa", "Monza", "Silverstone", "Oschersleben", "YasMarina"]
const VAL_TRACKS = ["Hockenheim", "Sepang"]
const TRAIN_TRACKS = sort!(setdiff(collect(keys(F1Tenth.RACETRACKS)), TEST_TRACKS, VAL_TRACKS))

"""
    centerline(name) -> Union{Path,Nothing}

A racetrack's closed centerline (`nothing` for a map that is not a racetrack).
"""
function centerline(name)
    haskey(F1Tenth.RACETRACKS, name) || return nothing
    dir = dirname(F1Tenth.racetrack_yaml(name))
    rows = [parse.(Float64, split(l, ",")) for l in eachline(joinpath(dir, name * "_centerline.csv"))
            if !startswith(l, "#") && !isempty(strip(l))]
    return Path(getindex.(rows, 1), getindex.(rows, 2); closed=true)
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

"""
    arc_clearance(m, x, y, θ, κ, hw) -> d

The free distance [m] along the arc of curvature κ leaving (x, y) in
direction θ, keeping `hw` from the walls: where it first comes that close,
or `CLEARANCE_MAX` if it doesn't within `CLEARANCE_MAX` or half a turn
(whichever is shorter: past half a turn the arc comes back). This is the
`clearance` label the networks learn.
"""
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
