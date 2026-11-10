# Race the held-out tracks with each way of using the two tiers, and compare.
#
#     julia -t auto --project=scripts scripts/compare.jl                       # 25 ms, 5 seeds
#     julia -t auto --project=scripts scripts/compare.jl 50 100                # also at 50 and 100 ms
#     julia -t auto --project=scripts scripts/compare.jl --seeds 10 50 100     # 10 seeds
#     julia -t auto --project=scripts scripts/compare.jl --small small_strong 50 100   # the stronger car networks
#
# The car runs the small_weak networks (`--small` to change), the cloud the
# large ones, from ../LidarLaneNets.jl/weights (`--weights DIR` to change).
#
# Configurations:
# - small only:       the small networks on the car, no cloud.
# - large only:       the cloud's networks for every estimate at every scan,
#                     and nothing else (between results the filter predicts).
# - both, every scan: small every scan, cloud for every estimate at every scan.
# - round robin:      small every scan, cloud for one estimate per scan, in turn.
# - far ahead:        small every scan, cloud for κ_profile, widths and sight
#                     at every scan.
#
# Each configuration runs one lap of every held-out track once per seed. A
# seed sets the LiDAR's noise and where the lap starts (seed k of n starts
# (k - 1)/n of the way round the centerline). Every configuration sees the
# same tracks, seeds and starts, so each is also compared with "small only"
# run for run (paired), which cancels most of the differences between tracks.
# Means come with 95% confidence intervals (t distribution) over the runs.
# Safety comes first (the focus); lap times after. Writes
# results/compare-<small set>.md and every run's numbers to
# results/runs-<small set>.csv, next to this script.

include("race.jl")

const CONFIGS = [
    ("small only", NoCalls, true),
    ("large only", AllCalls, false),
    ("both, every scan", AllCalls, true),
    ("round robin", RoundRobin, true),
    ("far ahead", () -> FixedCalls(:κ_profile, :widths, :sight), true),
]

# Two-sided 95% quantiles of Student's t, by degrees of freedom (normal past 30).
const T95 = [12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228, 2.201, 2.179, 2.160, 2.145,
    2.131, 2.120, 2.110, 2.101, 2.093, 2.086, 2.080, 2.074, 2.069, 2.064, 2.060, 2.056, 2.052, 2.048, 2.045, 2.042]
t95(df) = df <= 0 ? NaN : df <= length(T95) ? T95[df] : 1.96

"""
    ci(xs) -> (mean, half-width)

The mean of `xs` and the half-width of its 95% confidence interval.
"""
function ci(xs)
    n = length(xs)
    n == 0 && return NaN, NaN
    m = sum(xs) / n
    n == 1 && return m, NaN
    s = sqrt(sum(abs2, xs .- m) / (n - 1))
    return m, t95(n - 1) * s / sqrt(n)
end

fmt(m, h; d=2) = isnan(m) ? "–" : isnan(h) ? string(round(m; digits=d)) :
                 "$(round(m; digits=d)) ± $(round(h; digits=d))"

# Start pose of seed k of n on a racetrack: (k - 1)/n of the way round.
function start_pose(name, k, n)
    p = centerline(name)
    i = 1 + floor(Int, (k - 1) / n * length(p.x))
    return (p.x[i], p.y[i], p.heading[i])
end

function compare(latencies_ms; seeds=5, weights=DEFAULT_WEIGHTS, small=:small_weak)
    nets = load_nets(weights; small)
    suffix = "-$small"
    F1Tenth.THREAD_AGENTS[] = false
    runs = [(track, k) for track in TEST_TRACKS for k in 1:seeds]
    # every configuration's results, run by run (same order as `runs`)
    rows = Tuple{String,String,Vector{RunResult}}[]
    for lat in latencies_ms, (name, schedule, use_small) in CONFIGS
        lat != first(latencies_ms) && name == "small only" && continue   # no cloud, no latency
        selector = SelectorConfig(remote_latency=lat / 1000, use_small=use_small)
        results = Vector{RunResult}(undef, length(runs))
        Threads.@threads :dynamic for i in eachindex(runs)
            track, k = runs[i]
            results[i] = run_map(track; nets, schedule, selector, seed=1000k, pose=start_pose(track, k, seeds))
        end
        push!(rows, (name, name == "small only" ? "–" : string(round(Int, lat)), results))
        println(name, ", ", rows[end][2], " ms: ", count(r -> r.finished, results), " of ", length(results), " laps")
        flush(stdout)
    end
    base = rows[1][3]   # small only
    out = joinpath(@__DIR__, "results")
    mkpath(out)
    write_runs(joinpath(out, "runs$suffix.csv"), rows, runs)

    # paired: each run minus small only's run on the same track, seed and start
    paired(rs, f) = ci([f(r) - f(b) for (r, b) in zip(rs, base)])
    pfmt(name, rs, f; d=2) = name == "small only" ? "–" : fmt(paired(rs, f)...; d)
    gap(r) = r.min_gap
    close(r) = 100r.close
    low(r) = 100r.low_ttc
    opt(r) = 100r.optimistic
    slip(r) = rad2deg(r.max_slip)

    safety = ["| Configuration | Cloud latency [ms] | Crashes | Closest to a wall, worst run [m] | " *
              "Closest to a wall [m] | Time within $(CLOSE_GAP) m of a wall [%] | " *
              "Scans with time to collision under $(LOW_TTC) s [%] | Least time to collision [s] | " *
              "Largest slip angle [°] | Scans planning on too much free distance [%] |",
        "|---|---|---|---|---|---|---|---|---|---|"]
    vs_small = ["| Configuration | Cloud latency [ms] | Closest to a wall [m] (more is safer) | " *
                "Time within $(CLOSE_GAP) m of a wall [pp] | Scans with time to collision under $(LOW_TTC) s [pp] | " *
                "Largest slip angle [°] | Scans planning on too much free distance [pp] |",
        "|---|---|---|---|---|---|---|"]
    perf = ["| Configuration | Cloud latency [ms] | Laps finished | Lap time [s] | Lap time vs small only, paired [%] | " *
            "e error [m] | ψe error [°] | Cloud calls per scan |",
        "|---|---|---|---|---|---|---|---|"]
    for (name, lat, rs) in rows
        finite_ttc = [r.min_ttc for r in rs if isfinite(r.min_ttc)]
        push!(safety, "| $name | $lat | $(count(r -> r.crashed, rs)) | $(round(minimum(gap, rs); digits=2)) | " *
                      "$(fmt(ci(gap.(rs))...)) | $(fmt(ci(close.(rs))...)) | $(fmt(ci(low.(rs))...)) | " *
                      "$(fmt(ci(finite_ttc)...)) | $(fmt(ci(slip.(rs))...; d=1)) | $(fmt(ci(opt.(rs))...)) |")
        name == "small only" || push!(vs_small, "| $name | $lat | $(pfmt(name, rs, gap)) | $(pfmt(name, rs, close)) | " *
                                               "$(pfmt(name, rs, low)) | $(pfmt(name, rs, slip; d=1)) | $(pfmt(name, rs, opt)) |")
        done = [r.sim_time for r in rs if r.finished]
        both = [100 * (r.sim_time / b.sim_time - 1) for (r, b) in zip(rs, base) if r.finished && b.finished]
        push!(perf, "| $name | $lat | $(length(done)) / $(length(rs)) | $(fmt(ci(done)...)) | " *
                    (name == "small only" ? "–" : fmt(ci(both)...)) * " | $(fmt(ci([r.e_rmse for r in rs])...; d=4)) | " *
                    "$(fmt(ci([rad2deg(r.ψe_rmse) for r in rs])...)) | $(round(sum(r.calls for r in rs) / length(rs); digits=2)) |")
    end
    println.(safety); println.(vs_small); println.(perf)
    write(joinpath(out, "compare$suffix.md"), """
        # Racing the held-out tracks: safety first ($small on the car)

        Networks: `$small` on the car (the small networks), `large` in the cloud, from
        `$(relpath(weights, @__DIR__))`.

        Tracks: $(join(TEST_TRACKS, ", ")). Each configuration: one lap of each track per seed, $seeds seeds
        ($(length(runs)) runs). A seed sets the LiDAR's noise and where the lap starts. Values are means over
        the runs with 95% confidence intervals. Every run's numbers are in [runs$suffix.csv](runs$suffix.csv).

        ## Safety

        From the true map and state. Time to collision: the true free distance along the arc the car is
        driving, over its speed (above 1 m/s); least over each run. Planning on too much free distance: the
        free distance ahead the speed was set for exceeds the true one, along the same arcs from where the
        scan was taken, by more than $(OPTIMISM) m.

        $(join(safety, "\n"))

        ### Against small only, run for run

        Each run minus small only's run on the same track, seed and start (pp: percentage points). An
        interval that excludes 0 is a difference the runs support.

        $(join(vs_small, "\n"))

        ## Lap times and tracking

        $(join(perf, "\n"))
        """)
end

# Every run's numbers, one line each.
function write_runs(path, rows, runs)
    open(path, "w") do io
        println(io, "configuration,latency_ms,track,seed,finished,crashed,lap_time_s,min_gap_m,close_frac,min_ttc_s,",
            "low_ttc_frac,max_slip_rad,max_lat_acc,optimistic_frac,e_rmse_m,psi_e_rmse_rad,calls_per_scan")
        for (name, lat, rs) in rows, ((track, k), r) in zip(runs, rs)
            println(io, join(("\"$name\"", lat, track, k, r.finished, r.crashed, r.sim_time, r.min_gap, r.close, r.min_ttc,
                r.low_ttc, r.max_slip, r.max_lat_acc, r.optimistic, r.e_rmse, r.ψe_rmse, r.calls), ","))
        end
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    args = copy(ARGS)
    option(name, default) = (i = findfirst(==(name), args); i === nothing ? default :
                                                            (v = args[i+1]; deleteat!(args, i:i+1); v))
    seeds = parse(Int, option("--seeds", "5"))
    weights = abspath(option("--weights", DEFAULT_WEIGHTS))
    small = Symbol(option("--small", "small_weak"))
    compare(vcat(25.0, parse.(Float64, args)); seeds, weights, small)
end
