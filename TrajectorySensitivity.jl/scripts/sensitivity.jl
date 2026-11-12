# The sensitivity runs: on the representative track (track.jl), the whole
# pipeline (F1Tenth.jl's car and LiDAR; LidarLaneNets.jl's selector and
# Kalman filter; LidarRacer.jl's planners and controller), with the networks
# replaced by the oracle (TrajectorySensitivity.jl). One estimate is changed
# at a time, the others true and on time:
#
# - accuracy: the estimate with the car network's (small_weak) errors times a
#   gain, from the car at every scan; and, for comparison, the cloud
#   network's (large) errors, also from the car (so without its delay);
# - latency: the estimate true, but only from the cloud, a delay after its
#   scan (dropped past the selector's max_age, as on the car).
#
# Every run is compared with the reference (every estimate true and on
# time) along the centerline. Writes results/runs.csv (one row per run),
# results/sensitivity.md (the summary), results/runs.jld2 (every
# trajectory) and the plots (plots.jl).
#
#     julia -t auto --project=. sensitivity.jl                 # both tests
#     julia -t auto --project=. sensitivity.jl accuracy        # one
#     julia -t auto --project=. sensitivity.jl --seeds 3 --estimates e,ψe
#
# Needs results/error_model.toml (error_model.jl) and track/ (track.jl).

const TRAIN = joinpath(@__DIR__, "..", "..", "LidarLaneNets.jl", "train")
include(joinpath(TRAIN, "truth.jl"))   # Track, truth (the oracle's true estimates), wall_distance

using JLD2
using LidarRacer
using Printf
using TrajectorySensitivity
using TrajectorySensitivity: _nanmax

const NAME = "Representative"
const TRACK_DIR = joinpath(@__DIR__, "track")
const OUT = joinpath(@__DIR__, "results")

# The sweeps. Gains: half to 16 times the car network's errors. Delays: 0 to
# 125 ms, in steps of the scan period (a result is used at the first scan
# after it arrives); past 100 ms the selector drops it.
const GAINS = [0.5, 1.0, 2.0, 4.0, 8.0, 16.0]
const DELAYS = [0.0, 0.025, 0.05, 0.075, 0.1, 0.125]
const LAPS = 2

"""
    load_track() -> (track, segments, base)

The representative track: as truth.jl's `Track`, its segments, and its map
(the yaml's path without the extension).
"""
function load_track()
    base = joinpath(TRACK_DIR, NAME * "_map")
    rows = [parse.(Float64, split(l, ",")) for l in eachline(joinpath(TRACK_DIR, NAME * "_centerline.csv"))
            if !startswith(l, "#") && !isempty(strip(l))]
    center = Path(getindex.(rows, 1), getindex.(rows, 2); closed=true)
    track = Track(NAME, F1Tenth.load_map(base * ".yaml", ".pgm"), center, center.s[end] + center.seg_length[end])
    return track, read_segments(joinpath(TRACK_DIR, NAME * "_segments.csv")), base
end

# About how far the car's body is from the nearest wall [m] (as race.jl's wall_gap).
function wall_gap(m, x, y, θ, len, wid)
    s, c = sincos(θ)
    return minimum(wall_distance(m, x + u * len / 2 * c - v * wid / 2 * s, y + u * len / 2 * s + v * wid / 2 * c)
                   for (u, v) in ((1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)))
end

"""
    simulate(p, track, base, models; laps=LAPS, timestep=0.0025, max_time=60laps) -> TrajectoryLog

Drive `laps` laps from a standing start on the start line (or until a
crash, the car leaving the track or `max_time` seconds) with the controller
of the experiment `p`, the oracle given the true estimates of each scan.
"""
function simulate(p::Perturbation, track, base, models; laps=LAPS, timestep=0.0025, max_time=60.0laps)
    env = F110Env(; map=base, map_ext=".pgm", num_agents=1, timestep, integrator=Integrator.RK4, seed=1)
    params, agent = env.params, env.sim.agents[1]
    car = CarParams(params)
    racer, oracle = oracle_racer(car, agent.lidar.scan_angles, p, models)
    x_ref, hw = car.lf - env.lidar_dist, car.width / 2
    (sx, sy), sθ = center_point(track, 0.0), center_heading(track, 0.0)
    obs, _, _, _ = reset!(env, [sx sy sθ])
    lg = TrajectoryLog(track.center)
    clock, commands = ScanClock(params.scan_rate), CommandQueue()
    t, hint = 0.0, 0
    while obs.collisions[1] == 0 && obs.lap_counts[1] < laps && t < max_time
        x, y, θ = obs.poses_x[1], obs.poses_y[1], obs.poses_theta[1]
        v, r = obs.linear_vels_x[1], obs.ang_vels_z[1]
        dt = new_scan!(clock, obs.scan_times[1])
        if dt !== nothing
            res = truth(track, agent.scan_origin, x_ref; hw, scan_range=car.scan_range, hint)
            res === nothing && break   # the front axle is off the track
            labels, hint = res
            set_truth!(oracle, labels, clock.last)
            steer, speed, _ = control!(racer, obs.scans[1], v, r, dt)
            queue!(commands, clock.last + racer.sweep + racer.latency, steer, speed)
            log!(lg, t, x, y, θ, v, wall_gap(track.map, x, y, θ, params.length, params.width))
        end
        steer, v_cmd = command_at!(commands, env.current_time)
        obs, step_time, _, _ = step!(env, [steer v_cmd])
        t += step_time
    end
    lg.crashed = obs.collisions[1] > 0
    lg.crashed && log!(lg, t, obs.poses_x[1], obs.poses_y[1], obs.poses_theta[1], obs.linear_vels_x[1], 0.0)
    return lg
end

"""
    experiments(which; estimates, seeds) -> Vector{Perturbation}

The runs of the tests `which` (`:accuracy`, `:latency`).
"""
function experiments(which; estimates=ESTIMATES, seeds=20)
    out = Perturbation[]
    if :accuracy in which
        for est in estimates, seed in 1:seeds
            append!(out, accuracy(est, g; seed) for g in GAINS)
            push!(out, accuracy(est, 1.0; network=:large, seed))
        end
    end
    if :latency in which
        append!(out, latency(est, d) for est in estimates for d in DELAYS)
    end
    return out
end

# --- the summary

const SEGMENT_KINDS = ["long straight", "hairpin", "medium corner", "fast sweeper", "chicane", "tight hairpin",
    "tightening corner", "double apex", "straight"]

fmt(x; d=3) = isfinite(x) ? string(round(x; sigdigits=d)) : "–"
pctile(x, p) = (y = filter(isfinite, x); isempty(y) ? NaN : quantile(y, p))

function csv_row(p::Perturbation, c::Comparison, seg)
    vals = Any[p.kind, something(p.estimate, ""), p.network, p.gain, 1000p.delay, p.seed, c.crashed, c.crash_s,
        c.max_Δd, c.rms_Δd, c.max_Δψ, c.max_Δv, c.Δlap, c.min_gap]
    append!(vals, get(seg, k, NaN) for k in SEGMENT_KINDS)
    return join(vals, ",")
end

const CSV_HEADER = "kind,estimate,network,gain,delay_ms,seed,crashed,crash_s,max_dd_m,rms_dd_m,max_dpsi_rad,max_dv_mps," *
                   "dlap_s,min_gap_m," * join(("max_dd_" * replace(k, " " => "_") * "_m" for k in SEGMENT_KINDS), ",")

function summarize(ps, logs, ref, segments, track)
    dist = distance_driven(ref) - 1.0   # each run's laps end a little apart
    comps = [compare(lg, ref; distance=dist) for lg in logs]
    segs = [by_segment(c.s, abs.(c.Δd), segments) for c in comps]
    open(joinpath(OUT, "runs.csv"), "w") do io
        println(io, CSV_HEADER)
        foreach((p, c, s) -> println(io, csv_row(p, c, s)), ps, comps, segs)
    end

    reflap = ref.t[end] / LAPS
    lines = String[]
    push!(lines, """
        # Trajectory sensitivity to each estimate

        From [`sensitivity.jl`](../sensitivity.jl), on the representative track ([track.md](track.md)),
        $(LAPS) laps from a standing start. The reference (every estimate true and on time) takes
        $(fmt(ref.t[end]; d=4)) s ($(fmt(reflap; d=3)) s a lap, $(fmt(dist / ref.t[end]; d=3)) m/s on average),
        its body at least $(fmt(minimum(ref.gap))) m from a wall. Each run changes one estimate, the others
        true and on time, and is compared with the reference at the same distance along the centerline:
        Δd is the change in the car's lateral position, Δv in its speed, Δlap in the time for the $(LAPS) laps
        (– after a crash). Per run in [runs.csv](runs.csv).
        """)

    # accuracy
    acc = findall(p -> p.kind === :accuracy, ps)
    if !isempty(acc)
        push!(lines, """
            ## Accuracy

            The estimate with a network's errors (drawn as [error_model.md](error_model.md) checks) times a
            gain, from the car at every scan, with the σ such a network would give; gain 1 of small_weak is the
            car's network, gain 1 of large the cloud's (here without its delay). $(maximum(p.seed for p in ps[acc]))
            runs (error seeds) each. Max |Δd|: the median over the runs (and the worst). Tube: the width of the
            band all runs' lateral positions lie in, at each distance along the track: its mean and greatest.

            | Estimate | Errors | Gain | Max \\|Δd\\| median (worst) [m] | Max \\|Δv\\| median [m/s] | Δlap median [s] | Least gap [m] | Crashes | Tube mean / max [m] |
            |---|---|---|---|---|---|---|---|---|""")
        tubes = Dict{Tuple{Symbol,Symbol,Float64},Any}()
        for est in unique(ps[i].estimate for i in acc), (net, gains) in ((:small_weak, GAINS), (:large, [1.0]))
            for g in gains
                idx = filter(i -> ps[i].estimate === est && ps[i].network === net && ps[i].gain == g, acc)
                isempty(idx) && continue
                tb = tube(logs[idx]; distance=dist)
                tubes[(est, net, g)] = tb
                cs = comps[idx]
                push!(lines, @sprintf("| `%s` | %s | %s | %s (%s) | %s | %s | %s | %d / %d | %s / %s |", est, net, fmt(g),
                    fmt(pctile([c.max_Δd for c in cs], 0.5)), fmt(_nanmax([c.max_Δd for c in cs])),
                    fmt(pctile([c.max_Δv for c in cs], 0.5)), fmt(pctile([c.Δlap for c in cs], 0.5)),
                    fmt(minimum(c.min_gap for c in cs)), count(c -> c.crashed, cs), length(cs),
                    fmt(sum(filter(isfinite, tb.width)) / max(1, count(isfinite, tb.width))), fmt(_nanmax(tb.width))))
            end
        end
        # where on the track: the tube's greatest width per kind of segment, at gain 1
        push!(lines, """

            Where on the track: the tube's greatest width [m] per kind of segment, with the car network's errors
            (small_weak, gain 1).

            | Estimate | $(join(SEGMENT_KINDS, " | ")) |
            |---|$(repeat("---|", length(SEGMENT_KINDS)))""")
        for est in unique(ps[i].estimate for i in acc)
            haskey(tubes, (est, :small_weak, 1.0)) || continue
            tb = tubes[(est, :small_weak, 1.0)]
            seg = by_segment(tb.s, tb.width, segments)
            push!(lines, "| `$est` | " * join((fmt(get(seg, k, NaN)) for k in SEGMENT_KINDS), " | ") * " |")
        end
        jldsave(joinpath(OUT, "tubes.jld2"); tubes=Dict(string(k) => v for (k, v) in tubes))
    end

    # latency
    lat = findall(p -> p.kind === :latency, ps)
    if !isempty(lat)
        push!(lines, """

            ## Latency

            The estimate true, but only from the cloud (as the selector and the Kalman filter handle a cloud
            result: counted at its scan's time, the scans after it filtered again), arriving a delay after its
            scan; the car's network gives none. The selector drops results older than 100 ms, so past that the
            estimate is never measured: the filter only predicts it (e, ψe, κ) or keeps its last value (the rest),
            and without e and ψe the lane is never found and the car stays put.

            | Estimate | $(join((@sprintf("%g ms", 1000d) for d in DELAYS), " | ")) |
            |---|$(repeat("---|", length(DELAYS)))""")
        for est in unique(ps[i].estimate for i in lat)
            cells = map(DELAYS) do d
                i = findfirst(j -> ps[j].estimate === est && ps[j].delay == d, lat)
                i === nothing && return "–"
                c, lg = comps[lat[i]], logs[lat[i]]
                distance_driven(lg) < 1.0 && return "no start"
                c.crashed ? "crash at $(fmt(c.crash_s)) m" : "$(fmt(c.max_Δd)) m, $(fmt(c.Δlap)) s"
            end
            push!(lines, "| `$est` | " * join(cells, " | ") * " |")
        end
        push!(lines, "\nEach cell: max |Δd|, and Δlap.")
    end
    write(joinpath(OUT, "sensitivity.md"), join(lines, "\n") * "\n")
    return comps
end

# --- running

function parse_args(args)
    which, estimates, seeds = Symbol[], collect(ESTIMATES), 20
    i = 1
    while i <= length(args)
        a = args[i]
        if a in ("accuracy", "latency")
            push!(which, Symbol(a)); i += 1
        elseif a == "--estimates"
            estimates = Symbol.(split(args[i+1], ",")); i += 2
        elseif a == "--seeds"
            seeds = parse(Int, args[i+1]); i += 2
        else
            error("unknown argument $a")
        end
    end
    return (isempty(which) ? [:accuracy, :latency] : which), estimates, seeds
end

function main(args=ARGS)
    which, estimates, seeds = parse_args(args)
    models_path = joinpath(OUT, "error_model.toml")
    models = Dict(n => load_errors(models_path; network=n) for n in (:small_weak, :large))
    track, segments, base = load_track()
    F1Tenth.THREAD_AGENTS[] = false
    start = time()
    ref = simulate(reference(), track, base, models)
    @printf("reference: %.2f s for %d laps, %s, %.1f s wall\n", ref.t[end], LAPS, ref.crashed ? "CRASH" : "no crash",
        time() - start)
    ref.crashed && error("the reference run crashed")
    ps = experiments(which; estimates, seeds)
    logs = Vector{TrajectoryLog}(undef, length(ps))
    done = Threads.Atomic{Int}(0)
    Threads.@threads :dynamic for i in eachindex(ps)
        logs[i] = simulate(ps[i], track, base, models)
        k = Threads.atomic_add!(done, 1) + 1
        @printf("%4d / %d  %-45s %s\n", k, length(ps), describe(ps[i]),
            logs[i].crashed ? "CRASH" : @sprintf("%.2f s", logs[i].t[end]))
        flush(stdout)
    end
    mkpath(OUT)
    jldsave(joinpath(OUT, "runs.jld2"); perturbations=ps, logs, reference=ref)
    summarize(ps, logs, ref, segments, track)
    @printf("%d runs in %.0f s\n", length(ps), time() - start)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
