# Lane keeping on F1Tenth maps with the Stanley controller, using only the LiDAR.
#
# The walls are the lane boundaries. Each scan, the on-board controller
# (`LaneKeeper`) runs, using four sibling packages:
#
#   perception  LidarLaneEstimation.jl finds the lane and the room around its
#               center
#   path        MinimumCurvaturePlanner.jl plans the path through that room
#               that bends least (in wide before a corner, close to the inside
#               at its apex)
#   estimation  LidarLaneEstimation.jl measures the car against the path and
#               smooths the result with a Kalman filter: the cross-track error
#               e and heading error ψe at the front axle, the path's curvature
#               ahead, and how far ahead it is seen
#   steering    StanleyController.jl turns e and ψe into a steering angle
#   speed       VelocityProfilePlanner.jl: as fast as the car can still slow
#               down in time for what it sees, and for what it may not see yet
#
# The commands go to the car's own low-level controllers (in F1Tenth.jl: the
# command limiter, the steering servo, the VESC speed loop). The controller
# never sees the car's pose or the map. For the
# f1tenth_racetracks maps the track's centerline file is used only to place
# the car at the start line and to score the run.
#
# It needs F1Tenth.jl's realistic car (its main branch, not
# archive/python-compat): the actuator limits, the 40 Hz LiDAR and
# `obs.scan_times`.
#
#     julia --project=examples examples/lane_keep.jl Spa             # one lap, prints a score
#     julia --project=examples examples/lane_keep.jl Spa --render    # watch it (needs a display)
#     julia -t auto --project=examples examples/lane_keep.jl all     # every racetrack, in parallel
#
# The first time, set up the examples environment with
#     julia --project=examples -e 'using Pkg; Pkg.instantiate()'

using F1Tenth
using LidarLaneEstimation
using MinimumCurvaturePlanner
using StanleyController
using VelocityProfilePlanner
using Printf

const USAGE = """
usage: lane_keep.jl MAP [options]

MAP is a racetrack name (Austin, Sochi, Spa, Shanghai, ...), "all" for every
racetrack, or any ROS map yaml path (without .yaml) together with --pose.

options:
  --speed V        cap the speed at V m/s (default: the car's top speed)
  --laps N         laps to drive (default 1)
  --pose X Y THETA start pose; required for maps that are not racetracks
  --map-ext EXT    image extension of a custom map (default .png)
  --render         open the render window (needs a display)
"""

# ---------------------------------------------------------------------------
# Speed

"""
    SpeedConfig(params::VehicleParams; v_max=params.speed_cmd_max, kwargs...)

The limits that set the speed. There is no cruise speed: the car goes as
fast as it can still slow down in time for every curve it sees, and for the
sharpest curve it could meet just past what it sees.
"""
Base.@kwdef struct SpeedConfig
    v_max::Float64            # never faster than this [m/s]
    v_min::Float64 = 1.0      # never slower than this [m/s]
    # Lateral acceleration planned for turns [m/s²]: about half the tires'
    # peak grip (1.0 g at the rear; the front, which slides first, gives
    # 0.77 g at 0.2 rad of slip), leaving the rest for steering corrections
    # and braking. Every racetrack lap finishes from 2 to 6 m/s².
    a_lat::Float64 = 5.0
    a_brake::Float64          # deceleration the car can count on [m/s²]
    r_min::Float64            # the car's tightest turning radius [m]
    # Time from a scan's first beam until the car acts on it, and the speed
    # command changes [s]. The car drives on meanwhile.
    delay::Float64
    margin::Float64 = 0.5     # distance to keep from a wall straight ahead [m]
end

# From the car: its top speed command, the command limiter's deceleration,
# the turning radius at its smaller steering limit, and the delay from a
# scan's first beam until the next command could replace this one: the sweep
# (`sweep` seconds), the latency until the command takes effect, and one scan
# period until the next decision.
SpeedConfig(params::VehicleParams; sweep, latency, v_max=params.speed_cmd_max, kwargs...) = SpeedConfig(; v_max,
    a_brake=params.decel_cmd, r_min=(params.lf + params.lr) / tan(min(params.steer_cmd_max, -params.steer_cmd_min)),
    delay=sweep + latency + 1 / params.scan_rate, kwargs...)

"""
    speed_command(cfg::SpeedConfig, lane::LaneEstimate, v, yaw_rate) -> v_cmd

The first point of the velocity profile (forward-backward, from
VelocityProfilePlanner.jl) along the path ahead: the fastest speed from which
the car, braking at `a_brake` after driving on for `delay`, reaches every
point no faster than its curve allows (lateral acceleration at most `a_lat`),
can stop `margin` short of a wall straight ahead, and can still take, just
past the end of the path (`lane.sight`), a curve as sharp as it can turn.
Also capped so that the lateral acceleration in the turn the car is making
now (speed × yaw rate) is at most `a_lat`.
"""
function speed_command(cfg::SpeedConfig, lane::LaneEstimate, v, yaw_rate)
    coast = cfg.delay * abs(v)   # driven before the car reacts
    from_braking(d) = max(d - coast, 0.0)
    # points (distance from where braking can start, curvature, speed cap)
    pts = Tuple{Float64,Float64,Float64}[]
    for (j, κ) in enumerate(lane.κ_profile)
        d = (j - 1) * PROFILE_STEP
        d > lane.sight && break
        isnan(κ) || push!(pts, (from_braking(d), κ, Inf))
    end
    push!(pts, (from_braking(lane.sight), 0.0, cornering_speed(1 / cfg.r_min; a_lat=cfg.a_lat)))
    wall = lane.clearance - cfg.margin
    isfinite(wall) && push!(pts, (from_braking(wall), 0.0, cfg.v_min))
    sort!(pts; by=first)
    profile = velocity_profile(getindex.(pts, 1), getindex.(pts, 2); a_lat=cfg.a_lat, a_brake=cfg.a_brake,
        v_cap=getindex.(pts, 3))
    return clamp(min(profile[1], cfg.a_lat / max(abs(yaw_rate), eps())), cfg.v_min, cfg.v_max)
end

# ---------------------------------------------------------------------------
# The controller: what runs on the car, from a scan to a command

"""
    LaneKeeper(params, beams, lidar_dist; stanley, perception, smoothing, margin, latency, v_max)

The on-board controller of a car with parameters `params` and a LiDAR whose
beams point at `beams` [rad], `lidar_dist` meters ahead of the center of
gravity: lane perception, the path planner (keeping `margin` meters from
every return), the Kalman filter, the Stanley law (which measures the errors
at the front axle) and the speed planner. A scan is published `sweep`
seconds after its first beam, and its command takes effect `latency` after
that. `path_x`, `path_y` hold the last path planned (LiDAR frame).
"""
mutable struct LaneKeeper
    perceive::LanePerception
    margin::Float64
    smooth::LaneFilter
    stanley::Stanley
    speed::SpeedConfig
    sweep::Float64
    latency::Float64
    path_x::Vector{Float64}
    path_y::Vector{Float64}
end

function LaneKeeper(params::VehicleParams, beams, lidar_dist; stanley=default_stanley(params),
                    perception=default_perception(params), smoothing=default_smoothing(),
                    margin=default_margin(params), latency=default_latency(), v_max=params.speed_cmd_max)
    # the front axle is lf ahead of the center of gravity, so lf - lidar_dist
    # ahead of the LiDAR, and lf + lr ahead of the rear axle
    perceive = LanePerception(beams, params.lf - lidar_dist; cfg=perception)
    sweep = (beams[end] - beams[1]) / 2π / params.scan_rate
    return LaneKeeper(perceive, margin, LaneFilter(params.lf + params.lr; cfg=smoothing), stanley,
        SpeedConfig(params; sweep, latency, v_max), sweep, latency, Float64[], Float64[])
end

# From a scan's publication to its command taking effect [s]: computing it
# and passing it on. ForzaETH measured their whole race stack's latency on
# NUC5's computer (an Intel i5-10210U) at 7.5 ms at most on average (Baumann
# et al., "ForzaETH Race Stack", 2024, §8.1.1); this controller is lighter
# (about 0.25 ms here), so 7.5 ms is a cautious value. No transport delay
# (Ethernet, driver) has been published.
default_latency() = 0.0075

# The defaults below were tuned on 12 of the 23 racetracks (two LiDAR noise
# seeds each) and checked on the other 11; each sits in the middle of a range
# of values that all finish every lap.

# Yaw damping (Hoffmann et al.) lets the cross-track gain be stiff despite
# the servo lag: without it the car weaves from k = 3 on; with k_yaw = 0.1,
# every k from 2 to 5 finishes. The softening speed hardly matters.
default_stanley(params) = Stanley(k=3.0, k_soft=1.0, k_yaw=0.1, max_steer=params.s_max)

# Use every return the LiDAR guarantees (its detection range, 10 m on white
# walls), with the curvature as far ahead as that. Far along a wall seen at a
# grazing angle, neighboring returns are about r² Δθ / w apart (10² m² ×
# 0.25° / 1.1 m ≈ 0.4 m), so a wall is only cut at gaps over 0.8 m: 0.3 m cut
# far walls short and the car saw less of the lane (6% slower); 0.6 to 1.3 m
# all did about the same, and smaller keeps clear of real openings. The other
# settings changed nothing measurable on the racetracks (stride, smooth,
# spacing and curve_base by under 1%; the one-wall fallbacks never run
# there).
default_perception(params) = PerceptionConfig(car_halfwidth=params.width / 2, r_max=params.scan_range,
    preview=params.scan_range, jump=0.8)

# The path keeps half the car's width plus 0.15 m from every return (it
# seldom comes that close).
default_margin(params) = params.width / 2 + 0.15

# Scan noise as measured against the true lane center (standard deviations
# of the scan's errors on the tuning tracks). Process noise: 5 × the measured
# drift of the true lane from the filter's motion model (0.10 m/s, 0.17 rad/s,
# 0.16 1/m²), because the line followed is re-planned every scan and moves
# more than the lane does; 3× to 8× all work. The outlier gate and the
# curvature profile's averaging changed nothing measurable on the racetracks
# (the speed is limited by how far ahead the lane is seen, not by its
# curvature), so they keep their defaults.
default_smoothing() = FilterConfig(σ_e=0.05, σ_ψ=0.075, σ_κ=0.06, q_e=0.5, q_ψ=0.85, q_κ=0.8)

"""
    control!(k::LaneKeeper, ranges, v, yaw_rate, dt) -> (steer, speed, lane)

The steering angle [rad] and speed [m/s] to command for a new scan `ranges`,
given the measured speed and yaw rate and the time `dt` since the last scan,
with the lane estimate they came from.
"""
function control!(k::LaneKeeper, ranges, v, yaw_rate, dt)
    p = k.perceive
    detect_lane!(p, ranges)
    c = lane_corridor(p, k.margin)
    k.path_x, k.path_y = c === nothing ? (p.center_x, p.center_y) :
        min_curvature_path(c.x, c.y, c.lower, c.upper; normals=(c.nx, c.ny))
    lane = track!(k.smooth, measure!(p, k.path_x, k.path_y), v, yaw_rate, dt)
    steer = steering_angle(k.stanley, lane.e, lane.ψe, v; yaw_rate, path_yaw_rate=lane.κ * v)
    return steer, speed_command(k.speed, lane, v, yaw_rate), lane
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
the track centerline (the car follows its own line, so some offset is
intended); perception errors compare the lane center found in each scan
with the true centerline, at the front axle. Centerline-based numbers are
NaN without a centerline. `min_gap` is the closest the car's body came to a
wall, from the map.
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
    single_wall::Float64      # fraction of scans that used one wall or none
    min_gap::Float64          # [m]
    wall_time::Float64
end

"""
    wall_gap(map::F1Tenth.MapData, x, y, θ, length, width) -> d

About how far the car's body (`length` × `width`, centered at `(x, y)`,
heading `θ`) is from the nearest wall [m]: the map's distance to a wall,
least over the body's corners and the midpoints of its sides (to within a
map cell).
"""
function wall_gap(m::F1Tenth.MapData, x, y, θ, length, width)
    so, co = sincos(m.origin[3])
    s, c = sincos(θ)
    h, w = size(m.dt)
    gap = Inf
    for (u, v) in ((1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1))
        px = x + u * length / 2 * c - v * width / 2 * s - m.origin[1]
        py = y + u * length / 2 * s + v * width / 2 * c - m.origin[2]
        col = floor(Int, (px * co + py * so) / m.resolution)
        row = floor(Int, (-px * so + py * co) / m.resolution)
        (0 <= col < w && 0 <= row < h) || continue
        gap = min(gap, m.dt[row+1, col+1])
    end
    return gap
end

"""
    run_map(map_name; speed, laps, pose, map_ext, render, stanley, perception, smoothing, latency,
            timestep, max_time, seed) -> RunResult

Drive `laps` laps (or until a crash or `max_time` seconds) with a
[`LaneKeeper`](@ref); `speed` caps the speed, `stanley`, `perception`,
`smoothing` and `latency` replace its defaults, `timestep` is the
simulator's step [s] (it also sets how precisely a command's latency is
timed), and `seed` seeds the LiDAR noise.
"""
function run_map(map_name; speed=nothing, laps=1, pose=nothing, map_ext=".png", render=false,
                 stanley=nothing, perception=nothing, smoothing=default_smoothing(), latency=default_latency(),
                 timestep=0.0025, max_time=600.0, seed=12345)
    path = centerline(map_name)
    if pose === nothing
        path === nothing && error("$map_name is not a racetrack; give a start pose with --pose X Y THETA")
        pose = (path.x[1], path.y[1], path.heading[1])
    end
    env = F110Env(; map=map_name, map_ext, num_agents=1, timestep, integrator=Integrator.RK4, seed)
    walls = env.sim.scan_simulator.map
    params = env.params
    keeper = LaneKeeper(params, env.sim.agents[1].lidar.scan_angles, env.lidar_dist;
        stanley=something(stanley, default_stanley(params)), perception=something(perception, default_perception(params)),
        smoothing, latency, v_max=something(speed, params.speed_cmd_max))
    perceive = keeper.perceive

    render && add_render_callback!(env, e -> render_callback(e, env, keeper))
    obs, _, _, _ = reset!(env, [pose[1] pose[2] pose[3]])
    render && render!(env)

    t, dist, steps, scans, single, min_gap = 0.0, 0.0, 0, 0, 0, Inf
    sum_off, max_off, sum_e2, sum_ψ2 = 0.0, 0.0, 0.0, 0.0
    hint = 0
    last_scan, steer, v_cmd = -Inf, 0.0, 0.0
    # commands on their way, oldest first: (time it takes effect, steer, speed)
    pending = Tuple{Float64,Float64,Float64}[]
    start = time()
    # The env reports `done` after 2 laps (as in Python), so stop on our own
    # lap count, or on a crash.
    while obs.collisions[1] == 0 && obs.lap_counts[1] < laps && t < max_time
        x, y, θ = obs.poses_x[1], obs.poses_y[1], obs.poses_theta[1]
        v, r = obs.linear_vels_x[1], obs.ang_vels_z[1]
        min_gap = min(min_gap, wall_gap(walls, x, y, θ, params.length, params.width))

        truth = path === nothing ? nothing : tracking_error(path, x, y, θ; hint, window=hint == 0 ? 0 : 30)
        truth === nothing || (hint = truth.segment)
        # The LiDAR publishes a scan every 1/scan_rate seconds, when its sweep
        # ends, and holds it in between. As on the real car, the controller
        # runs once per new scan; its command takes effect `latency` after the
        # scan's publication and is held until the next.
        if obs.scan_times[1] != last_scan
            scan_dt = isfinite(last_scan) ? obs.scan_times[1] - last_scan : 1 / params.scan_rate
            last_scan = obs.scan_times[1]
            new_steer, new_speed, lane = control!(keeper, obs.scans[1], v, r, scan_dt)
            push!(pending, (last_scan + keeper.sweep + keeper.latency, new_steer, new_speed))
            scans += 1
            lane.walls === :both || (single += 1)
            if truth !== nothing && length(perceive.center_x) >= 3
                # the lane center in this scan against the true one, at the front axle
                seen = tracking_error(Path(perceive.center_x, perceive.center_y), perceive.x_ref, 0.0, 0.0)
                fa = tracking_error(path, front_axle(x, y, θ, params.lf)..., θ; hint, window=30)
                sum_e2 += (seen.e - fa.e)^2
                sum_ψ2 += wrap_angle(seen.ψe - fa.ψe)^2
            end
        end
        while !isempty(pending) && env.current_time >= pending[1][1] - 1e-9
            _, steer, v_cmd = popfirst!(pending)
        end
        obs, step_time, _, _ = step!(env, [steer v_cmd])
        render && render!(env; mode=:human_fast)

        t += step_time
        steps += 1
        dist += abs(v) * step_time
        if truth !== nothing   # score against the true centerline
            sum_off += abs(truth.e)
            max_off = max(max_off, abs(truth.e))
        end
    end
    crashed = obs.collisions[1] > 0
    n, ns = max(steps, 1), max(scans, 1)
    known(x) = path === nothing ? NaN : x
    return RunResult(basename(string(map_name)), obs.lap_counts[1] >= laps && !crashed, crashed, t,
        known(sum_off / n), known(max_off), dist / max(t, eps()), known(sqrt(sum_e2 / ns)),
        known(sqrt(sum_ψ2 / ns)), single / ns, min_gap, time() - start)
end

function print_header()
    @printf("%-14s %-8s %8s %7s %9s %9s %7s %9s %8s %9s %7s\n", "map", "result", "time[s]", "v[m/s]",
        "|off|[m]", "max|off|", "gap[m]", "e_err[m]", "ψe_err°", "1-wall[%]", "wall[s]")
end

function print_result(r::RunResult)
    status = r.finished ? "lap" : r.crashed ? "CRASH" : "timeout"
    @printf("%-14s %-8s %8.2f %7.2f %9.3f %9.3f %7.2f %9.3f %8.2f %9.1f %7.1f\n", r.map, status, r.sim_time,
        r.mean_speed, r.mean_offset, r.max_offset, r.min_gap, r.e_rmse, rad2deg(r.ψe_rmse), 100r.single_wall,
        r.wall_time)
end

# ---------------------------------------------------------------------------
# Rendering (only with --render; GLMakie is loaded on demand)

const _drawn = Ref{Any}(nothing)

function render_callback(e, env, keeper)
    GL = Main.GLMakie
    # camera follows the car, as in waypoint_follow.jl
    xs = [p[1] for p in e.cars[1][]]
    ys = [p[2] for p in e.cars[1][]]
    top, bottom, left, right = maximum(ys), minimum(ys), minimum(xs), maximum(xs)
    e.score_label.x = left
    e.score_label.y = top - 700
    e.left, e.right, e.top, e.bottom = left - 800, right + 800, top + 800, bottom - 800

    # the line the car follows, in orange
    st = env.sim.agents[1].state
    θ = st[5]
    ox, oy = front_axle(st[1], st[2], θ, env.lidar_dist)   # scan origin
    k = F1Tenth.RENDER_SCALE   # renderer units per meter
    pts = GL.Point2f[GL.Point2f(k * (ox + x * cos(θ) - y * sin(θ)), k * (oy + x * sin(θ) + y * cos(θ)))
           for (x, y) in zip(keeper.path_x, keeper.path_y)]
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
    opts = Dict{Symbol,Any}(:map => args[1], :speed => nothing, :laps => 1, :pose => nothing,
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
        # with several Julia threads (julia -t auto), the tracks run in parallel
        F1Tenth.THREAD_AGENTS[] = false
        names = sort!(collect(keys(F1Tenth.RACETRACKS)))
        results = Vector{RunResult}(undef, length(names))
        Threads.@threads :dynamic for i in eachindex(names)
            results[i] = run_map(names[i]; speed=o[:speed], laps=o[:laps])
        end
        print_header()
        foreach(print_result, results)
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
