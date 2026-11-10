# Train the networks on the data from collect.jl: for each estimate and
# tier, a network that outputs the estimate's mean and standard deviation,
# trained on the true values (so it can beat a hand-written estimator).
#
# Loss: the β-negative log-likelihood of a Gaussian (Seitzer et al., "On the
# pitfalls of heteroscedastic uncertainty estimation", ICLR 2022), β = 0.5:
# each output's squared error over its predicted variance, plus its log
# standard deviation, weighted by the (non-differentiated) σ^(2β) so that hard
# samples don't stop the mean from being learned. Targets are normalized per
# output. A large network starts as its trained small one (see
# `large_from_small`), so train.jl trains the small ones first. Half of each batch is mirrored left to right (the LiDAR is
# symmetric: the beams reverse, and so do the signs of e, ψe and κ, the two
# widths, and the fan of arcs), which doubles the data for free. The epoch
# kept is the one with the least mean squared error on the validation tracks.
#
# On a GPU: if LuxCUDA (NVIDIA), AMDGPU (AMD) or Metal (Apple) is installed in
# this environment, it is loaded and used; otherwise the CPU.
#
#     julia -t auto --project=. train.jl                    # every network set (small_weak, small_strong, large)
#     julia -t auto --project=. train.jl small_weak         # one set
#     julia -t auto --project=. train.jl large e ψe         # one set, some estimates
#
# The sets are LidarLaneNets.NETWORKS, saved to ../weights/<set>_<estimate>.jld2.
# A large network starts from the trained small_strong one, so train that
# first. small_weak (for the car) is used with the same large networks (for
# the cloud): the cloud needs only the scan.

using JLD2
using LidarLaneNets
using LinearAlgebra
using Lux
using Optimisers
using Printf
using Random
using Statistics
using Zygote

include("truth.jl")   # the track split (and F1Tenth.RACETRACKS)

const DATA_DIR = joinpath(@__DIR__, "data")
const WEIGHTS_DIR = joinpath(@__DIR__, "..", "weights")

# Load a GPU backend if one is installed here; MLDataDevices picks it up.
for pkg in (:LuxCUDA, :AMDGPU, :Metal)
    Base.find_package(String(pkg)) === nothing && continue
    try
        @eval using $pkg
    catch err
        @warn "could not load $pkg" err
    end
end

const EPOCHS = (small=parse(Int, get(ENV, "EPOCHS_SMALL", "40")), large=parse(Int, get(ENV, "EPOCHS_LARGE", "20")))
const BATCH = 256
# The large networks refine a trained small one: a gentler rate keeps the
# small part from being knocked off at the start.
const LR = (small=1e-3, large=3e-4)
const β = 0.5f0

"""
    load_split(names) -> (X, Y)

Raw scans (`(beams, N)`) and the true estimates (a Dict of `(n_out, N)`)
from the tracks `names`.
"""
function load_split(names)
    parts = [load(joinpath(DATA_DIR, n * ".jld2")) for n in names]
    X = reduce(hcat, (p["X"] for p in parts))
    Y = Dict(est => reduce(hcat, (p["Y"][String(est)] for p in parts)) for est in ESTIMATES)
    return X, Y
end

function features(X::AbstractMatrix, name::Symbol)
    stride = beam_stride(network_tier(name); spec=network_spec(name))
    F = Matrix{Float32}(undef, cld(size(X, 1), stride), size(X, 2))
    Threads.@threads for j in axes(X, 2)
        scan_features!(F, view(X, :, j), stride, j)
    end
    return F
end

"""
    mirror!(est, Y) -> Y

The estimates of the mirror image of each scan (columns of `Y`): left and
right swapped.
"""
function mirror!(est::Symbol, Y::AbstractMatrix)
    if est === :e || est === :ψe || est === :κ_profile
        Y .= .-Y
    elseif est === :widths
        n = size(Y, 1) ÷ 2
        Y .= vcat(Y[n+1:end, :], Y[1:n, :])
    elseif est === :clearance
        Y .= Y[end:-1:1, :]   # CLEARANCE_K is symmetric about 0
    end
    return Y
end

# β-NLL on normalized targets; y and the outputs are on the device.
function loss_fn(model, ps, st, (x, y))
    out, st = model(x, ps, st)
    n = size(y, 1)
    μ, logσ = out[1:n, :], clamp.(out[n+1:2n, :], LidarLaneNets.LOGσ_MIN, LidarLaneNets.LOGσ_MAX)
    σ² = exp.(2 .* logσ)
    w = Zygote.dropgrad(σ² .^ β)
    nll = (μ .- y) .^ 2 ./ (2 .* σ²) .+ logσ
    return mean(w .* nll), st, (; mse=mean((μ .- y) .^ 2))
end

# Mean squared error of the mean, in normalized units, over a whole split.
function evaluate(model, ps, st, F, Yn, dev)
    stt = Lux.testmode(st)
    se, n = 0.0, 0
    for idx in Iterators.partition(axes(F, 2), 4096)
        x = dev(reshape(F[:, idx], size(F, 1), 1, length(idx)))
        out, _ = model(x, ps, stt)
        μ = Array(out)[1:size(Yn, 1), :]
        se += sum(abs2, μ .- Yn[:, idx])
        n += length(μ)
    end
    return se / n
end

function train_one(est, name, Ftr, Ytr, Fva, Yva; dev, epochs, rng=Xoshiro(42))
    tier, spec = network_tier(name), network_spec(name)
    size(Ftr, 1) == input_size(tier, 1081; spec) || error("expected features of 1081 beams")
    if tier === :small
        both = hcat(Ytr, mirror!(est, copy(Ytr)))   # normalize over the data and its mirror image
        y_mean = vec(mean(both; dims=2))
        y_std = max.(vec(std(both; dims=2)), 1f-3)
        net = EstimatorNet(est, tier, 1081; spec, rng, y_mean, y_std)
    else
        # the trained small network, plus a correction that starts at zero
        net = large_from_small(load_net(weights_file(WEIGHTS_DIR, est, :small_strong)); rng)
        y_mean, y_std = net.y_mean, net.y_std
    end
    norm(Y) = (Y .- y_mean) ./ y_std
    Yn_va = norm(Yva)
    lr = LR[tier]
    ts = Training.TrainState(net.model, dev(net.ps), dev(net.st), Adam(lr))
    # epoch 0, the starting point, is a candidate too: a large network is
    # never kept worse than its small one
    # (copies: on the CPU, cpu_device returns the training's own arrays, which
    # the optimizer keeps changing)
    best, best_ps, best_epoch = evaluate(net.model, ts.parameters, ts.states, Fva, Yn_va, dev), deepcopy(net.ps), 0
    tier === :large && @printf("  %-12s %-10s epoch  0  (the small network)  val mse %.5f\n", name, est, best)
    N = size(Ftr, 2)
    steps = epochs * cld(N, BATCH)
    k = 0
    for epoch in 1:epochs
        t0 = time()
        perm = randperm(rng, N)
        train_mse = 0.0
        for idx in Iterators.partition(perm, BATCH)
            k += 1
            Optimisers.adjust!(ts.optimizer_state, lr * (1 + cos(π * k / steps)) / 2)   # cosine decay
            xb, yb = Ftr[:, idx], Ytr[:, idx]
            flip = rand(rng, length(idx)) .< 0.5
            xb[:, flip] .= xb[end:-1:1, flip]
            yb[:, flip] .= mirror!(est, yb[:, flip])
            x = dev(reshape(xb, size(xb, 1), 1, length(idx)))
            y = dev(norm(yb))
            _, _, stats, ts = Training.single_train_step!(AutoZygote(), loss_fn, (x, y), ts)
            train_mse += stats.mse * length(idx)
        end
        val = evaluate(net.model, ts.parameters, ts.states, Fva, Yn_va, dev)
        if val < best
            best, best_epoch = val, epoch
            best_ps = deepcopy(cpu_device()(ts.parameters))
        end
        @printf("  %-12s %-10s epoch %2d  train mse %.5f  val mse %.5f  (%.0f s)\n", name, est, epoch,
            train_mse / N, val, time() - t0)
        flush(stdout)
    end
    trained = EstimatorNet(est, tier, 1081, net.spec, net.model, best_ps, cpu_device()(ts.states), y_mean, y_std)
    mkpath(WEIGHTS_DIR)
    save_net(weights_file(WEIGHTS_DIR, est, name), trained)
    @printf("  %-12s %-10s kept epoch %d (val mse %.5f)\n", name, est, best_epoch, best)
    return trained
end

function main(args=ARGS)
    names = String.(NETWORKS)
    sets = isempty(args) || !(args[1] in names) ? collect(NETWORKS) : [Symbol(args[1])]
    rest = isempty(args) ? String[] : args[(args[1] in names ? 2 : 1):end]
    ests = isempty(rest) ? collect(ESTIMATES) : Symbol.(rest)
    dev = gpu_device()
    BLAS.set_num_threads(Threads.nthreads())
    println("device: ", dev, ", threads: ", Threads.nthreads())
    Xtr, Ytr = load_split(TRAIN_TRACKS)
    Xva, Yva = load_split(VAL_TRACKS)
    println("train: $(size(Xtr, 2)) scans from $(length(TRAIN_TRACKS)) tracks; validation: $(size(Xva, 2)) from ",
        join(VAL_TRACKS, ", "))
    for name in sets
        Ftr, Fva = features(Xtr, name), features(Xva, name)
        for est in ests
            train_one(est, name, Ftr, Ytr[est], Fva, Yva[est]; dev, epochs=EPOCHS[network_tier(name)])
        end
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
