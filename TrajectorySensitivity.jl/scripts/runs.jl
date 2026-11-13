# What the experiment scripts share: driving a track with an experiment's
# controller (the whole pipeline: F1Tenth.jl's car and LiDAR, LidarLaneNets.jl's
# selector and Kalman filter, LidarRacer.jl's planners and controller, the
# networks replaced by the oracle), and summing each run up against the
# track's reference, per kind of situation (TrajectorySensitivity.SITUATIONS).
# Included by sensitivity.jl, cloud.jl and calibrate.jl.

const TRAIN = joinpath(@__DIR__, "..", "..", "LidarLaneNets.jl", "train")
include(joinpath(TRAIN, "truth.jl"))   # Track, truth, arc_clearance, wall_distance, the track split

using JLD2
using LidarRacer
using Printf
using Random
using Statistics
using TrajectorySensitivity
using TrajectorySensitivity: _nanmax

const OUT = joinpath(@__DIR__, "results")
const TRACK_DIR = joinpath(@__DIR__, "track")
const REPRESENTATIVE = "Representative"
const WEIGHTS = joinpath(@__DIR__, "..", "..", "LidarLaneNets.jl", "weights")

# Safety thresholds, as LidarRacer.jl's race.jl: a body this close to a wall
# [m], a time to collision this short [s].
const CLOSE_GAP = 0.2
const LOW_TTC = 0.5

"""
    Course(name; laps)

A track to drive: one of the 23 racetracks by name, or the representative
track (track.jl); `laps` from a standing start on its start line.
"""
struct Course
    name::String
    track::Track
    map::String
    ext::String
    laps::Int
end

function Course(name::AbstractString; laps::Integer=name == REPRESENTATIVE ? 2 : 1)
    if name == REPRESENTATIVE
        base = joinpath(TRACK_DIR, name * "_map")
        rows = [parse.(Float64, split(l, ",")) for l in eachline(joinpath(TRACK_DIR, name * "_centerline.csv"))
                if !startswith(l, "#") && !isempty(strip(l))]
        center = Path(getindex.(rows, 1), getindex.(rows, 2); closed=true)
        track = Track(name, F1Tenth.load_map(base * ".yaml", ".pgm"), center, center.s[end] + center.seg_length[end])
        return Course(name, track, base, ".pgm", laps)
    end
    return Course(name, Track(name), name, ".png", laps)
end

const ALL_TRACKS = sort!(collect(keys(F1Tenth.RACETRACKS)))

"""
    inputs(; nets=false, library=true) -> (models, library, nets)

The error models (error_model.jl), the recorded errors (error_library.jl;
`nothing` without `library`) and the real networks (with `nets`, for
:network experiments).
"""
function inputs(; nets::Bool=false, library::Bool=true)
    models = Dict(n => load_errors(joinpath(OUT, "error_model.toml"); network=n) for n in (:small_weak, :large))
    library = library ? load_library(joinpath(OUT, "error_library.jld2")) : nothing
    real = nets ? (n = load_nets(WEIGHTS); Dict(:small_weak => n.small, :large => n.large)) : nothing
    return models, library, real
end

# About how far the car's body is from the nearest wall [m] (as race.jl's wall_gap).
function wall_gap(m, x, y, θ, len, wid)
    s, c = sincos(θ)
    return minimum(wall_distance(m, x + u * len / 2 * c - v * wid / 2 * s, y + u * len / 2 * s + v * wid / 2 * c)
                   for (u, v) in ((1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)))
end

"""
    Run

One drive: its trajectory (a TrajectoryLog, a sample per scan), and per
scan its situation class and time to collision (along the arc the car is
driving, as race.jl measures it; Inf below 1 m/s).
"""
struct Run
    log::TrajectoryLog
    class::Vector{Int}
    ttc::Vector{Float64}
end

"""
    simulate(p, course, inputs; timestep=0.0025, lidar_seed, on_scan) -> Run

Drive `course` with the controller of the experiment `p` (a Perturbation or
a JointAccuracy), the oracle given
the true estimates of each scan, until the laps are done, the car crashes or
leaves the track, or 200 s a lap pass. `lidar_seed` seeds the LiDAR's noise
(by default `p.seed` for a real network, which sees it, else 1); the
oracle never replays errors recorded on the course's own track;
`on_scan(ranges, labels)` is called with each new scan and its true
estimates.
"""
function simulate(p, c::Course, (models, library, nets); timestep=0.0025,
                  lidar_seed=p isa Perturbation && p.kind === :network ? p.seed : 1, on_scan=nothing)
    seed = lidar_seed
    env = F110Env(; map=c.map, map_ext=c.ext, num_agents=1, timestep, integrator=Integrator.RK4, seed)
    params, agent = env.params, env.sim.agents[1]
    car = CarParams(params)
    # never the course's own recorded errors
    library = library === nothing ? nothing : without(library, c.name)
    racer, oracle = oracle_racer(car, agent.lidar.scan_angles, p, models; library, nets)
    tr = c.track
    x_ref, hw = car.lf - env.lidar_dist, car.width / 2
    (sx, sy), sθ = center_point(tr, 0.0), center_heading(tr, 0.0)
    obs, _, _, _ = reset!(env, [sx sy sθ])
    run = Run(TrajectoryLog(tr.center), Int[], Float64[])
    clock, commands = ScanClock(params.scan_rate), CommandQueue()
    t, hint = 0.0, 0
    while obs.collisions[1] == 0 && obs.lap_counts[1] < c.laps && t < 200.0c.laps
        x, y, θ = obs.poses_x[1], obs.poses_y[1], obs.poses_theta[1]
        v, r = obs.linear_vels_x[1], obs.ang_vels_z[1]
        dt = new_scan!(clock, obs.scan_times[1])
        if dt !== nothing
            res = truth(tr, agent.scan_origin, x_ref; hw, scan_range=car.scan_range, hint)
            res === nothing && break   # the front axle is off the track
            labels, hint = res
            on_scan === nothing || on_scan(obs.scans[1], labels)
            set_truth!(oracle, labels, clock.last)
            steer, speed, _ = control!(racer, obs.scans[1], v, r, dt)
            queue!(commands, clock.last + racer.sweep + racer.latency, steer, speed)
            log!(run.log, t, x, y, θ, v, wall_gap(tr.map, x, y, θ, params.length, params.width))
            push!(run.class, situation_class(labels))
            turning = abs(v) > 0.5 ? r / abs(v) : 0.0
            fx, fy = x + car.lf * cos(θ), y + car.lf * sin(θ)
            push!(run.ttc, v > 1.0 ? arc_clearance(tr.map, fx, fy, θ, turning, hw) / v : Inf)
        end
        steer, v_cmd = command_at!(commands, env.current_time)
        obs, step_time, _, _ = step!(env, [steer v_cmd])
        t += step_time
    end
    run.log.crashed = obs.collisions[1] > 0
    return run
end

# =====================================================================
# Per-class summaries

const ROW_HEADER = "course,kind,estimate,network,gain,delay_ms,seed,source,class,distance_m,time_s,scans,min_gap_m," *
                   "close_s,low_ttc_scans,max_dd_m,sum_dd2,n_dd,crashed,tube_max_m,tube_mean_m"

_csv(x::AbstractFloat) = isfinite(x) ? string(round(x; sigdigits=6)) : "NaN"
_csv(x) = string(x)

"""
    run_rows(p, course, run, ref, ref_class_at) -> Vector{String}

One CSV row per situation class the run met: how far and how long it drove
in it, the least gap to a wall there, the time within CLOSE_GAP of one, the
scans with time to collision under LOW_TTC, its lateral deviation from the
reference there (by the reference's class at each distance), and whether
it crashed there. Tube columns empty (see `tube_rows`).
"""
function run_rows(p::Perturbation, c::Course, run::Run, ref::Run, ref_class_at)
    lg = run.log
    n = length(lg.t)
    dist, time, scans, mingap, close, lowttc = zeros(10), zeros(10), zeros(Int, 10), fill(Inf, 10), zeros(10), zeros(Int, 10)
    for k in 1:n
        cl = run.class[k]
        scans[cl] += 1
        mingap[cl] = min(mingap[cl], lg.gap[k])
        lowttc[cl] += run.ttc[k] < LOW_TTC
        if k < n
            ds, dt = max(lg.s[k+1] - lg.s[k], 0.0), lg.t[k+1] - lg.t[k]
            dist[cl] += ds
            time[cl] += dt
            lg.gap[k] < CLOSE_GAP && (close[cl] += dt)
        end
    end
    cmp = compare(lg, ref.log; distance=distance_driven(ref.log) - 1.0)
    maxdd, sumdd2, ndd = zeros(10), zeros(10), zeros(Int, 10)
    for (s, d) in zip(cmp.s, cmp.Δd)
        isfinite(d) || continue
        cl = ref_class_at(s)
        maxdd[cl] = max(maxdd[cl], abs(d))
        sumdd2[cl] += d^2
        ndd[cl] += 1
    end
    crash_class = lg.crashed && n > 0 ? run.class[end] : 0
    rows = String[]
    for cl in 1:10
        scans[cl] == 0 && ndd[cl] == 0 && continue
        push!(rows, join((c.name, p.kind, something(p.estimate, ""), p.network, p.gain, 1000p.delay, p.seed, p.source,
            SITUATIONS[cl], _csv(dist[cl]), _csv(time[cl]), scans[cl], _csv(mingap[cl]), _csv(close[cl]), lowttc[cl],
            _csv(maxdd[cl]), _csv(sumdd2[cl]), ndd[cl], crash_class == cl, "", ""), ","))
    end
    return rows
end

"""
    tube_rows(p, course, runs, ref, ref_class_at) -> Vector{String}

The tube of a group of runs (one experiment, several seeds): the greatest
and the mean width, per class, of the band all their lateral positions lie
in (seed 0 marks a group's row).
"""
function tube_rows(p::Perturbation, c::Course, runs::Vector{Run}, ref::Run, ref_class_at)
    length(runs) < 2 && return String[]
    tb = tube([r.log for r in runs]; distance=distance_driven(ref.log) - 1.0)
    mx, sm, nn = zeros(10), zeros(10), zeros(Int, 10)
    for (s, w, k) in zip(tb.s, tb.width, tb.n)
        (isfinite(w) && k >= 2) || continue
        cl = ref_class_at(s)
        mx[cl] = max(mx[cl], w)
        sm[cl] += w
        nn[cl] += 1
    end
    return [join((c.name, p.kind, something(p.estimate, ""), p.network, p.gain, 1000p.delay, 0, p.source, SITUATIONS[cl],
        "", "", "", "", "", "", "", "", "", "", _csv(mx[cl]), _csv(sm[cl] / nn[cl])), ",") for cl in 1:10 if nn[cl] > 0]
end

"""
    class_lookup(ref) -> s -> class

The reference run's situation class at a distance `s` from its start.
"""
function class_lookup(ref::Run)
    s0 = ref.log.s[1]
    ss = ref.log.s .- s0
    return s -> ref.class[clamp(searchsortedlast(ss, s), 1, length(ss))]
end

"""
    campaign(name, groups, courses, inputs) -> path

Drive every course's reference, then every group of experiments (each a
vector of Perturbations differing only in seed) on every course, in
parallel, into results/<name>.csv: one row per run and class met, plus each
group's tube.
"""
function campaign(name, groups::Vector{Vector{Perturbation}}, course_names, ins)
    F1Tenth.THREAD_AGENTS[] = false
    courses = Dict(n => Course(n) for n in course_names)
    refs = Dict{String,Tuple{Run,Any}}()
    Threads.@threads for n in collect(course_names)
        r = simulate(reference(), courses[n], ins)
        r.log.crashed && @warn "the reference crashed on $n"
        lock(() -> (refs[n] = (r, class_lookup(r))), CSV_LOCK)
    end
    tasks = [(n, g) for n in course_names for g in groups]
    rows = String[]
    done = Threads.Atomic{Int}(0)
    start = time()
    Threads.@threads :dynamic for (n, g) in tasks
        c, (ref, at) = courses[n], refs[n]
        runs = [simulate(p, c, ins) for p in g]
        out = reduce(vcat, (run_rows(p, c, r, ref, at) for (p, r) in zip(g, runs)))
        append!(out, tube_rows(g[1], c, runs, ref, at))
        lock(() -> append!(rows, out), CSV_LOCK)
        k = Threads.atomic_add!(done, 1) + 1
        (k % 20 == 0 || k == length(tasks)) && @printf("%s: %d / %d groups, %.0f s\n", name, k, length(tasks), time() - start)
        flush(stdout)
    end
    # the references too, as kind "none"
    for n in course_names
        ref, at = refs[n]
        append!(rows, run_rows(reference(), courses[n], ref, ref, at))
    end
    path = joinpath(OUT, name * ".csv")
    open(path, "w") do io
        println(io, ROW_HEADER)
        foreach(r -> println(io, r), rows)
    end
    return path
end

const CSV_LOCK = ReentrantLock()
