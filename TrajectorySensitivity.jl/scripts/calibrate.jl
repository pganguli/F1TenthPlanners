# How the oracle compares with the real networks: for each estimate, the
# real network on the car (the others true), against the oracle with that
# network's recorded errors (replay) and with its modeled ones (parametric),
# for the car's (small_weak) and the cloud's (large) networks, on the tracks
# the networks never trained on (the validation and held-out tracks: the
# error library's, each run replaying only the other tracks' errors) and the
# representative track. Into results/calibration.csv; report.jl sums it
# up. Not part of the analysis: only this script runs the networks in the
# loop.
#
#     julia -t 4 --project=. calibrate.jl          # 5 seeds (the real networks' LiDAR noise)
#     julia -t 4 --project=. calibrate.jl --seeds 2

include(joinpath(@__DIR__, "runs.jl"))
include(joinpath(@__DIR__, "sensitivity.jl"))   # parse_args

const CALIBRATION_TRACKS = vcat(VAL_TRACKS, TEST_TRACKS, REPRESENTATIVE)

function calibration_groups(seeds)
    g = Vector{Perturbation}[]
    for est in ESTIMATES, network in (:small_weak, :large)
        push!(g, [real_network(est; network, seed) for seed in seeds])
        push!(g, [accuracy(est, 1.0; network, source=:replay, seed) for seed in seeds])
        push!(g, [accuracy(est, 1.0; network, source=:parametric, seed) for seed in seeds])
    end
    return g
end

function calibrate_main(args=ARGS)
    seeds, _ = parse_args(args)
    seeds == 1:10 && (seeds = 1:5)
    println(campaign("calibration", calibration_groups(seeds), CALIBRATION_TRACKS, inputs(; nets=true)))
end

if abspath(PROGRAM_FILE) == @__FILE__
    calibrate_main()
end
