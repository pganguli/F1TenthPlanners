# Lane keeping on F1Tenth maps with the Stanley controller, using only the LiDAR.
#
# The walls are the lane boundaries. Each step, the scan is turned into the
# two numbers the Stanley law needs, measured at the front axle:
#
#   e   cross-track error: how far the lane center is to the left [m]
#   ψe  heading error: the lane direction relative to the car [rad]
#
# plus the lane curvature ahead and the free distance straight ahead, which
# set the speed (a fixed cruise speed, lower in turns). The scan processing
# and the Kalman filter that smooths it over time are in the sibling package
# LidarLaneEstimation.jl. The controller never
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
using LidarLaneEstimation
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

"""
    target_speed(cfg::SpeedConfig, lane::LaneEstimate) -> v

The cruise speed, lowered so that the sharpest curve ahead keeps the lateral
acceleration at most `a_lat`, and so that the vehicle can stop within the
free distance straight ahead.
"""
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
    # Stanley measures the errors at the front axle: lf ahead of the center of
    # gravity, where the LiDAR is lidar_dist ahead, and lf + lr ahead of the rear axle.
    perceive = LanePerception(1080, 4.7, params.lf - env.lidar_dist; cfg=perception)
    tracker = LaneFilter(params.lf + params.lr; cfg=smoothing)
    speed_cfg = SpeedConfig(cruise=speed)

    render && add_render_callback!(env, e -> render_callback(e, env, perceive))
    obs, _, _, _ = reset!(env, [pose[1] pose[2] pose[3]])
    render && render!(env)

    t, dist, steps, single = 0.0, 0.0, 0, 0
    sum_off, max_off, sum_e2, sum_ψ2 = 0.0, 0.0, 0.0, 0.0
    hint = 0
    start = time()
    # The env reports `done` after 2 laps (as in Python), so stop on our own
    # lap count, or on a crash.
    while obs.collisions[1] == 0 && obs.lap_counts[1] < laps && t < max_time
        x, y, θ = obs.poses_x[1], obs.poses_y[1], obs.poses_theta[1]
        v, r = obs.linear_vels_x[1], obs.ang_vels_z[1]

        lane = track!(tracker, estimate_lane!(perceive, obs.scans[1]), v, r, dt)
        steer = steering_angle(stanley, lane.e, lane.ψe, v; yaw_rate=r, path_yaw_rate=lane.κ * v)
        obs, step_time, _, _ = step!(env, [steer target_speed(speed_cfg, lane)])
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
