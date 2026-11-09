# Race the held-out tracks with each way of using the two tiers, and compare.
#
#     julia -t auto --project=scripts scripts/compare.jl                 # 25 ms cloud latency
#     julia -t auto --project=scripts scripts/compare.jl 50 100          # also at 50 and 100 ms
#
# Configurations:
# - small only:       the small networks on the car, no cloud.
# - large only:       the cloud's networks for every estimate at every scan,
#                     and nothing else (between results the filter predicts).
# - both, every scan: small every scan, cloud for every estimate at every scan.
# - round robin:      small every scan, cloud for one estimate per scan, in turn.
# - far ahead:        small every scan, cloud for κ_profile, widths and sight
#                     at every scan.
# Writes results/compare.md next to this script.

include("race.jl")

const CONFIGS = [
    ("small only", NoCalls, true),
    ("large only", AllCalls, false),
    ("both, every scan", AllCalls, true),
    ("round robin", RoundRobin, true),
    ("far ahead", () -> FixedCalls(:κ_profile, :widths, :sight), true),
]

function compare(latencies_ms)
    nets = load_nets(DEFAULT_WEIGHTS)
    F1Tenth.THREAD_AGENTS[] = false
    lines = ["| Configuration | Cloud latency [ms] | Laps finished | Mean lap time [s] | Mean speed [m/s] | " *
             "Closest to a wall [m] | e error [m] | ψe error [°] | Cloud calls per scan |",
        "|---|---|---|---|---|---|---|---|---|"]
    detail = String[]
    for lat in latencies_ms, (name, schedule, use_small) in CONFIGS
        lat != first(latencies_ms) && name == "small only" && continue   # no cloud, no latency
        selector = SelectorConfig(remote_latency=lat / 1000, use_small=use_small)
        results = Vector{RunResult}(undef, length(TEST_TRACKS))
        Threads.@threads :dynamic for i in eachindex(TEST_TRACKS)
            results[i] = run_map(TEST_TRACKS[i]; nets, schedule, selector)
        end
        done = filter(r -> r.finished, results)
        mean_or_nan(f, rs) = isempty(rs) ? NaN : sum(f, rs) / length(rs)
        push!(lines, @sprintf("| %s | %s | %d / %d | %.2f | %.2f | %.2f | %.3f | %.2f | %.2f |", name,
            name == "small only" ? "–" : string(round(Int, lat)), length(done), length(results),
            mean_or_nan(r -> r.sim_time, done), mean_or_nan(r -> r.mean_speed, results),
            minimum(r -> r.min_gap, results), mean_or_nan(r -> r.e_rmse, results),
            rad2deg(mean_or_nan(r -> r.ψe_rmse, results)), mean_or_nan(r -> r.calls, results)))
        println(lines[end])
        push!(detail, "\n**$name**" * (name == "small only" ? "" : ", $(round(Int, lat)) ms") * ":\n")
        push!(detail, "| Track | Result | Time [s] | Speed [m/s] | Closest to a wall [m] |\n|---|---|---|---|---|")
        for r in results
            push!(detail, @sprintf("| %s | %s | %.2f | %.2f | %.2f |", r.map,
                r.finished ? "lap" : r.crashed ? "crash" : "timeout", r.sim_time, r.mean_speed, r.min_gap))
        end
    end
    out = joinpath(@__DIR__, "results")
    mkpath(out)
    write(joinpath(out, "compare.md"), "# Racing the held-out tracks\n\nTracks: $(join(TEST_TRACKS, ", ")), " *
        "one lap each. Mean lap time over the laps finished.\n\n" * join(lines, "\n") * "\n\n## Per track\n" *
        join(detail, "\n") * "\n")
end

if abspath(PROGRAM_FILE) == @__FILE__
    compare(isempty(ARGS) ? [25.0] : vcat(25.0, parse.(Float64, ARGS)))
end
