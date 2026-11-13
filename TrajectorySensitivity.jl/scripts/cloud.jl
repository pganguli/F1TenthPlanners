# What calling the cloud for one estimate buys, per kind of situation: the
# estimate from the car's network alone (its recorded errors, every scan)
# against the car's and the cloud's together (the cloud's recorded errors,
# from the same recorded scans, arriving 25, 50 or 100 ms after the scan),
# the other estimates true and on time, on the 23 racetracks and the
# representative track. Safety (crashes, the least gap to a wall, time close
# to one, time to collision) and the deviation from the reference, into
# results/cloud.csv; report.jl sums it up.
#
#     julia -t auto --project=. cloud.jl                   # every track, 10 seeds
#     julia -t auto --project=. cloud.jl --seeds 3 --tracks Spa

include(joinpath(@__DIR__, "runs.jl"))
include(joinpath(@__DIR__, "sensitivity.jl"))   # parse_args

const CLOUD_DELAYS = [0.025, 0.05, 0.1]

function cloud_groups(seeds)
    g = Vector{Perturbation}[]
    for est in ESTIMATES
        push!(g, [accuracy(est, 1.0; seed) for seed in seeds])   # the car alone
        append!(g, [cloud_call(est, d; seed) for seed in seeds] for d in CLOUD_DELAYS)
    end
    return g
end

function cloud_main(args=ARGS)
    seeds, tracks = parse_args(args)
    println(campaign("cloud", cloud_groups(seeds), tracks, inputs()))
end

if abspath(PROGRAM_FILE) == @__FILE__
    cloud_main()
end
