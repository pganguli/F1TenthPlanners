# Sensitivity of the trajectory and of safety to each estimate's accuracy
# and latency, one estimate at a time (the others true and on time), on the
# 23 racetracks and the representative track (track.jl), grouped by kind of
# situation:
#
# - accuracy: the estimate with the car network's (small_weak) recorded
#   errors times a gain, and the cloud network's (large) at gain 1, from the
#   car at every scan;
# - latency: the estimate true, but only from the cloud, a delay after its
#   scan (dropped past the selector's max_age, as on the car).
#
# Into results/sensitivity.csv (one row per run and situation met, plus each
# group's tube); report.jl sums it up.
#
#     julia -t auto --project=. sensitivity.jl              # every track, 10 seeds
#     julia -t auto --project=. sensitivity.jl --seeds 3 --tracks Spa,Representative
#
# Needs results/error_model.toml (error_model.jl), results/error_library.jld2
# (error_library.jl) and track/ (track.jl).

include(joinpath(@__DIR__, "runs.jl"))

const GAINS = [0.5, 1.0, 2.0, 4.0, 8.0, 16.0]
const DELAYS = [0.0, 0.025, 0.05, 0.075, 0.1, 0.125]

function groups(seeds)
    g = Vector{Perturbation}[]
    for est in ESTIMATES
        append!(g, [accuracy(est, k; seed) for seed in seeds] for k in GAINS)
        push!(g, [accuracy(est, 1.0; network=:large, seed) for seed in seeds])
        append!(g, [latency(est, d)] for d in DELAYS)
    end
    return g
end

function parse_args(args)
    seeds, tracks = 1:10, vcat(ALL_TRACKS, REPRESENTATIVE)
    i = 1
    while i <= length(args)
        if args[i] == "--seeds"
            seeds = 1:parse(Int, args[i+1]); i += 2
        elseif args[i] == "--tracks"
            tracks = String.(split(args[i+1], ",")); i += 2
        else
            error("unknown argument $(args[i])")
        end
    end
    return seeds, tracks
end

function main(args=ARGS)
    seeds, tracks = parse_args(args)
    println(campaign("sensitivity", groups(seeds), tracks, inputs()))
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
