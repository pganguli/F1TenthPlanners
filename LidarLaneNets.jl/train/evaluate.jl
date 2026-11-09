# How accurate the networks are, scan by scan, on the tracks they never saw
# in training, next to the hand-written estimator (LidarLaneEstimation.jl)
# on the same scans. Everything against the true values.
#
#     julia -t auto --project=. evaluate.jl          # prints, and writes results/accuracy.md
#
# Per estimate: the RMS error (over the profile entries the car can see:
# no farther ahead than the true `sight`); for the hand-written estimator,
# how many of those entries it gives at all; for the networks, how often the
# truth is within one predicted standard deviation (68% if calibrated); and
# the correlation between the small and the large network's errors (how
# much a cloud result adds to the small one for the same scan).

import LidarLaneEstimation as LLE
using JLD2
using LidarLaneNets
using LinearAlgebra
using Printf
using Statistics
using StanleyController: Path, tracking_error

include("truth.jl")

const DATA_DIR = joinpath(@__DIR__, "data")
const WEIGHTS_DIR = joinpath(@__DIR__, "..", "weights")
const RESULTS_DIR = joinpath(@__DIR__, "results")

# NUC5 and its LiDAR, as in F1Tenth.jl: the front axle 0.013 m behind the LiDAR.
const BEAMS = range(-3π / 4, 3π / 4; length=1081)
const X_REF = 0.130 - 0.143
const HALFWIDTH = 0.228 / 2

# The hand-written estimator as LidarRacer used it, with its curvature
# profile reaching as far as the networks'.
hand_config() = LLE.PerceptionConfig(car_halfwidth=HALFWIDTH, r_max=10.0, preview=PROFILE_LENGTH, jump=0.8)

# The hand-written estimator's estimates for every scan in X (in order: it
# keeps a running lane width). Profiles are NaN where it gives nothing.
function hand_estimates(X)
    p = LLE.LanePerception(BEAMS, X_REF; cfg=hand_config())
    N, NP, NK = size(X, 2), length(PROFILE_S), length(CLEARANCE_K)
    out = Dict(:e => zeros(1, N), :ψe => zeros(1, N), :κ_profile => fill(NaN, NP, N), :widths => fill(NaN, 2NP, N),
        :sight => zeros(1, N), :clearance => zeros(NK, N))
    none = 0
    for j in 1:N
        ranges = Float64.(view(X, :, j))
        est = LLE.estimate_lane!(p, ranges)
        if est.walls === :none
            none += 1
        else
            out[:e][j], out[:ψe][j] = est.e, est.ψe
            out[:κ_profile][:, j] .= est.κ_profile
            out[:sight][j] = min(est.sight, SIGHT_MAX)
            # the room to each side, at the points beside and ahead of the front axle
            m = 0.01
            c = LLE.lane_corridor(p, m; step=0.2)
            if c !== nothing
                s = vcat(0.0, cumsum(hypot.(diff(c.x), diff(c.y))))
                s_ref = tracking_error(Path(c.x, c.y), X_REF, 0.0, 0.0).s
                for (k, d) in enumerate(PROFILE_S)
                    u = s_ref + d
                    (0 <= u <= s[end]) || continue
                    i = clamp(searchsortedlast(s, u), 1, length(s) - 1)
                    f = (u - s[i]) / (s[i+1] - s[i])
                    out[:widths][k, j] = m + (1 - f) * c.upper[i] + f * c.upper[i+1]
                    out[:widths][NP+k, j] = m - ((1 - f) * c.lower[i] + f * c.lower[i+1])
                end
            end
        end
        out[:clearance][:, j] .= [min(LLE.clearance(p, κ), CLEARANCE_MAX) for κ in CLEARANCE_K]
    end
    return out, none / N
end

function net_estimates(nets, X)
    μs, σs = Dict{Symbol,Matrix{Float64}}(), Dict{Symbol,Matrix{Float64}}()
    for est in ESTIMATES
        net = nets[est]
        F = Matrix{Float32}(undef, input_size(net.tier, size(X, 1)), size(X, 2))
        Threads.@threads for j in axes(X, 2)
            scan_features!(F, view(X, :, j), net.tier, j)
        end
        μ, σ = Matrix{Float64}(undef, output_size(est), size(X, 2)), Matrix{Float64}(undef, output_size(est), size(X, 2))
        for idx in Iterators.partition(axes(X, 2), 4096)
            m, s = predict(net, F[:, idx])
            μ[:, idx], σ[:, idx] = m, s
        end
        μs[est], σs[est] = μ, σ
    end
    return μs, σs
end

# Which entries count: profile entries the car can see (no farther than the true sight).
function mask(est, Y)
    visible = PROFILE_S .<= Y[:sight]   # (NP, N)
    est === :κ_profile && return visible
    est === :widths && return vcat(visible, visible)
    return trues(size(Y[est]))
end

rmse(err) = isempty(err) ? NaN : sqrt(mean(abs2, err))

function score(Y, ests, est, M)
    p = ests[est]
    ok = M .& isfinite.(p)
    return rmse((p .- Y[est])[ok]), count(ok) / max(count(M), 1)
end

function main()
    data = [load(joinpath(DATA_DIR, n * ".jld2")) for n in TEST_TRACKS]
    X = reduce(hcat, (d["X"] for d in data))
    Y = Dict(est => Float64.(reduce(hcat, (d["Y"][String(est)] for d in data))) for est in ESTIMATES)
    driving = .!reduce(vcat, (d["static"] for d in data))
    nets = load_nets(WEIGHTS_DIR)
    println("test: $(size(X, 2)) scans ($(count(driving)) while driving) from ", join(TEST_TRACKS, ", "))

    hand, none = hand_estimates(X)
    small, σ_small = net_estimates(nets.small, X)
    large, σ_large = net_estimates(nets.large, X)

    unit = Dict(:e => "m", :ψe => "rad", :κ_profile => "1/m", :widths => "m", :sight => "m", :clearance => "m")
    lines = String[]
    push!(lines, "Test tracks: $(join(TEST_TRACKS, ", ")) ($(count(driving)) scans while driving). " *
                 "RMS error against the true values; profiles over the entries the car can see.\n")
    push!(lines, "| Estimate | Unit | Hand-written | (entries given) | Small DNN | Large DNN | Small within 1σ | " *
                 "Large within 1σ | Error correlation, small vs large |")
    push!(lines, "|---|---|---|---|---|---|---|---|---|")
    for est in ESTIMATES
        Yd = Dict(k => v[:, driving] for (k, v) in Y)
        M = mask(est, Yd)
        h, hc = score(Yd, Dict(est => hand[est][:, driving]), est, M)
        s, _ = score(Yd, Dict(est => small[est][:, driving]), est, M)
        l, _ = score(Yd, Dict(est => large[est][:, driving]), est, M)
        es = (small[est][:, driving].-Yd[est])[M]
        el = (large[est][:, driving].-Yd[est])[M]
        cs = mean(abs.(es) .<= σ_small[est][:, driving][M])
        cl = mean(abs.(el) .<= σ_large[est][:, driving][M])
        ρ = cor(es, el)
        push!(lines, @sprintf("| `%s` | %s | %.4f | %.0f%% | %.4f | %.4f | %.0f%% | %.0f%% | %.2f |", est, unit[est], h,
            100hc, s, l, 100cs, 100cl, ρ))
    end
    push!(lines, @sprintf("\nThe hand-written estimator found no lane in %.1f%% of all test scans (it then gives zeros).",
        100none))
    text = join(lines, "\n")
    println(text)
    mkpath(RESULTS_DIR)
    write(joinpath(RESULTS_DIR, "accuracy.md"), "# Per-scan accuracy\n\n" * text * "\n")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
