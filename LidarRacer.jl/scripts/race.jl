# Race F1Tenth.jl maps with LidarRacer.jl, using only the LiDAR.
#
# Each scan, the on-board controller (a `Racer`) plans a minimum-curvature
# line through the lane the LiDAR sees, steers along it and picks the
# fastest speed it can still brake from in time (see src/LidarRacer.jl). Its
# commands go to the car's own low-level controllers (in F1Tenth.jl: the
# command limiter, the steering servo, the VESC speed loop). The controller
# never sees the car's pose or the map. For the f1tenth_racetracks maps, the
# track's centerline file is used only to place the car at the start line
# and to score the run.
#
# It needs F1Tenth.jl's realistic car (its main branch, not
# archive/python-compat): the actuator limits, the 40 Hz LiDAR and
# `obs.scan_times`.
#
#     julia --project=scripts scripts/race.jl Spa             # one lap, prints a score
#     julia --project=scripts scripts/race.jl Spa --render    # watch it (needs a display)
#     julia -t auto --project=scripts scripts/race.jl all     # every racetrack, in parallel
#
# The first time, set up the scripts environment with
#     julia --project=scripts -e 'using Pkg; Pkg.instantiate()'

using F1Tenth
using LidarRacer
using StanleyController: Path, tracking_error, front_axle, wrap_angle
using Printf

const USAGE = """
usage: race.jl MAP [options]

MAP is a racetrack name (Austin, Sochi, Spa, Shanghai, ...), "all" for every
racetrack, or any ROS map yaml path (without .yaml) together with --pose.

options:
  --speed V        cap the speed at V m/s (default: the car's top speed)
  --laps N         laps to drive (default 1)
  --pose X Y THETA start pose; required for maps that are not racetracks
  --map-ext EXT    image extension of a custom map (default .png)
  --render         watch it in real time in a window (needs a display)
"""

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
    run_map(map_name; speed, laps, pose, map_ext, render, timestep, max_time, seed, racer...) -> RunResult

Drive `laps` laps (or until a crash or `max_time` seconds) with a
[`Racer`](@ref); `speed` caps the speed, and any other keywords (`stanley`,
`perception`, `smoothing`, `margin`, `latency`) replace the Racer's defaults.
`timestep` is the simulator's step [s] (it also sets how precisely a
command's latency is timed), and `seed` seeds the LiDAR noise. With `render`,
the run is drawn at `RENDER_FPS` and paced to real time.
"""
function run_map(map_name; speed=nothing, laps=1, pose=nothing, map_ext=".png", render=false,
                 timestep=0.0025, max_time=600.0, seed=12345, racer...)
    path = centerline(map_name)
    if pose === nothing
        path === nothing && error("$map_name is not a racetrack; give a start pose with --pose X Y THETA")
        pose = (path.x[1], path.y[1], path.heading[1])
    end
    env = F110Env(; map=map_name, map_ext, num_agents=1, timestep, integrator=Integrator.RK4, seed)
    walls = env.sim.scan_simulator.map
    params = env.params
    car = CarParams(params)
    controller = Racer(car, env.sim.agents[1].lidar.scan_angles, env.lidar_dist;
        v_max=something(speed, car.speed_cmd_max), racer...)
    perceive = controller.perceive

    render && add_render_callback!(env, e -> render_callback(e, env, controller))
    obs, _, _, _ = reset!(env, [pose[1] pose[2] pose[3]])
    render && render!(env)

    t, dist, steps, scans, single, min_gap = 0.0, 0.0, 0, 0, 0, Inf
    sum_off, max_off, sum_e2, sum_ψ2 = 0.0, 0.0, 0.0, 0.0
    hint = 0
    last_scan, steer, v_cmd = -Inf, 0.0, 0.0
    # commands on their way, oldest first: (time it takes effect, steer, speed)
    pending = Tuple{Float64,Float64,Float64}[]
    next_frame = 1 / RENDER_FPS   # sim time of the next frame drawn
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
            new_steer, new_speed, lane = control!(controller, obs.scans[1], v, r, scan_dt)
            push!(pending, (last_scan + controller.sweep + controller.latency, new_steer, new_speed))
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
        # draw at the display's rate, not the simulator's (400 steps a
        # second), each frame waiting for the wall clock to catch up
        if render && env.current_time >= next_frame - 1e-9
            render!(env; mode=:human)
            next_frame += 1 / RENDER_FPS
        end

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

# Frames per second of sim time; with real-time pacing, also of wall time.
const RENDER_FPS = 60
# Pixels rendered per window unit. GLMakie defaults to the display's scale
# (3 on a high-DPI display: 9 times the pixels, about 18 ms a frame, too slow
# for 60 frames a second). 2 is about 10 ms a frame; 1 is about 6 ms but
# visibly blurry.
const RENDER_PX_PER_UNIT = 2.0

const _drawn = Ref{Any}(nothing)

function render_callback(e, env, controller)
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
           for (x, y) in zip(controller.path_x, controller.path_y)]
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
        if o[:render]
            @eval Main using GLMakie
            @eval Main GLMakie.activate!(px_per_unit=$RENDER_PX_PER_UNIT)
        end
        r = Base.invokelatest(run_map, o[:map]; speed=o[:speed], laps=o[:laps], pose=o[:pose],
            map_ext=o[:map_ext], render=o[:render])
        print_header()
        print_result(r)
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
