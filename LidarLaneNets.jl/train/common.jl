# What the training scripts share: where things are, the tracks (tracks.jl),
# loading the collected data, and running a network over it.

using JLD2
using LidarLaneNets
using Statistics

include("tracks.jl")

const DATA_DIR = joinpath(@__DIR__, "data")
const WEIGHTS_DIR = joinpath(@__DIR__, "..", "weights")
const RESULTS_DIR = joinpath(@__DIR__, "results")

# At most this many errors per distribution fit (a random subset past it).
const MAX_FIT = 200_000

"""
    load_all(names; T=Float64) -> (X, Y, files)

The scans (`(beams, N)`), the true estimates (a Dict of `(n_out, N)`,
converted to `T`; `nothing` keeps them as stored, Float32) and the raw files
of the tracks `names`, in that order.
"""
function load_all(names; T=Float64)
    files = [load(joinpath(DATA_DIR, n * ".jld2")) for n in names]
    X = reduce(hcat, (f["X"] for f in files))
    stack(est) = reduce(hcat, (f["Y"][String(est)] for f in files))
    Y = Dict(est => T === nothing ? stack(est) : T.(stack(est)) for est in ESTIMATES)
    return X, Y, files
end

# Which scans were taken while driving (not standing), over the files in order.
driving_scans(files) = .!reduce(vcat, (f["static"] for f in files))

"""
    means_sigmas(net, X) -> (μ, σ)

The network's means and standard deviations for every scan in `X`.
"""
function means_sigmas(net, X)
    F = scan_features(X, input_stride(net))
    n = output_size(net.estimate)
    μ, σ = Matrix{Float64}(undef, n, size(X, 2)), Matrix{Float64}(undef, n, size(X, 2))
    for idx in Iterators.partition(axes(X, 2), 4096)
        m, s = predict(net, F[:, idx])
        μ[:, idx], σ[:, idx] = m, s
    end
    return μ, σ
end

"""
    mask(est, Y) -> BitMatrix

Which entries of `est` count: for the profiles, the entries the car can see
(no farther ahead than the true sight); everything otherwise.
"""
function mask(est, Y)
    visible = PROFILE_S .<= Y[:sight]   # (NP, N)
    est === :κ_profile && return visible
    est === :widths && return vcat(visible, visible)
    return trues(size(Y[est]))
end
