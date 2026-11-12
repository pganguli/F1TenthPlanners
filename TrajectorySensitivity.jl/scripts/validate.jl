# A check of the oracle against the real thing: the real networks (no
# oracle) driving the representative track through LidarRacer.jl's race
# loop (scripts/race.jl), 10 runs each (LiDAR noise seeds): the car's
# networks alone and the cloud's alone, the two error sources of the oracle,
# into results/validate.md. Not part of the analysis: the analysis never
# runs a network.
#
#     julia --project=. validate.jl

include(joinpath(@__DIR__, "..", "..", "LidarRacer.jl", "scripts", "race.jl"))   # run_map, DEFAULT_WEIGHTS

const TRACK = joinpath(@__DIR__, "track", "Representative")
const SEEDS = 1:10
const CONFIGS = (("car only (small_weak)", NoCalls, SelectorConfig()),
    ("cloud only (large, all six, 25 ms)", AllCalls, SelectorConfig(use_small=false)))

function main()
    rows = [parse.(Float64, split(l, ",")) for l in eachline(TRACK * "_centerline.csv") if !startswith(l, "#")]
    x0, y0 = rows[1][1], rows[1][2]
    pose = (x0, y0, atan(rows[2][2] - y0, rows[2][1] - x0))
    nets = load_nets(DEFAULT_WEIGHTS)
    lines = String[]
    for (name, schedule, selector) in CONFIGS
        res = [run_map(TRACK * "_map"; nets, map_ext=".pgm", pose, laps=2, seed, schedule, selector) for seed in SEEDS]
        crashes = filter(r -> r.crashed, res)
        where = isempty(crashes) ? "–" : join((@sprintf("%.0f", r.sim_time * r.mean_speed) for r in crashes), ", ")
        push!(lines, @sprintf("| %s | %d / %d | %s | %.2f | %.2f |", name, length(crashes), length(res), where,
            minimum(r.min_gap for r in res), sum(r.mean_speed for r in res) / length(res)))
        println(lines[end]); flush(stdout)
    end
    write(joinpath(@__DIR__, "results", "validate.md"), """
        # The real networks on the representative track

        The oracle stands in for the networks; this checks the track and the oracle against the real
        networks, driving the representative track ([track.md](track.md)) through LidarRacer.jl's race loop,
        $(length(SEEDS)) runs each (LiDAR noise seeds), $(2) laps from a standing start. From
        [`validate.jl`](../validate.jl).

        | Networks | Crashes | Where [m from the start, about] | Least gap to a wall [m] | Mean speed [m/s] |
        |---|---|---|---|---|
        $(join(lines, "\n"))

        Against [sensitivity.md](sensitivity.md): there, the oracle's `κ_profile` alone crashes in most runs,
        in the chicane (about 97 m), with either network's errors. The real car networks crash too, at the
        tight hairpin just after it (all six estimates erring at once). The real cloud networks never crash,
        so the oracle's `κ_profile` errors are pessimistic there, at least for the cloud's.
        """)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
