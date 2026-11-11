# How the networks do, and what they cost: every network set (see
# LidarLaneNets.NETWORKS) against the hand-written estimator
# (LidarLaneEstimation.jl), on the tracks they never saw in training.
#
#     julia -t auto --project=. evaluate.jl      # prints, and writes results/networks.md
#
# Per estimate and network:
# - MACs: multiply-accumulates for one scan, counted layer by layer from the
#   saved network (convolutions and dense layers); a large network includes
#   its small part. Parameters.
# - Validation R²: 1 − the mean squared error of the normalized targets on
#   the validation tracks (what training picks the epoch by; 1 is perfect).
# - Test RMS error, on the held-out tracks' scans while driving: profiles over
#   the entries the car can see (no farther ahead than the true sight); the
#   hand-written estimator over the entries it gives.
# - Within 1σ: how often the truth is within the network's own standard
#   deviation (68% if calibrated).
# - Error correlation with the large network's (for the small sets): how much
#   a cloud result adds to the car's for the same scan.

import LidarLaneEstimation as LLE
using LinearAlgebra
using Lux
using Printf
using StanleyController: Path, tracking_error

include("common.jl")

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

# MACs of a layer for an input of length L with C channels; returns (macs, L, C).
macs(l::Conv, L, C) = (Lout = fld(L + sum(l.pad) - l.kernel_size[1], l.stride[1]) + 1;
    (Lout * l.out_chs * l.in_chs * l.kernel_size[1], Lout, l.out_chs))
macs(l::Dense, L, C) = (l.in_dims * l.out_dims, l.out_dims, 1)
macs(::FlattenLayer, L, C) = (0, L * C, 1)
macs(l::WrappedFunction, L, C) = (0, cld(L, l.func.k), C)   # every k-th beam
function macs(c::Chain, L, C)
    total = 0
    for layer in c.layers
        m, L, C = macs(layer, L, C)
        total += m
    end
    return total, L, C
end
function macs(p::Parallel, L, C)   # branches on the same input, outputs added
    outs = [macs(layer, L, C) for layer in p.layers]
    return sum(first, outs), outs[1][2], outs[1][3]
end
macs(net::EstimatorNet) = first(macs(net.model, input_size(net), 1))

rmse(err) = isempty(err) ? NaN : sqrt(mean(abs2, err))
commas(n) = replace(string(n), r"(?<=\d)(?=(\d{3})+$)" => ",")
g(x; d=4) = isnan(x) ? "–" : string(round(x; sigdigits=d))

function main()
    Xva, Yva, _ = load_all(VAL_TRACKS)
    Xte, Yte, dte = load_all(TEST_TRACKS)
    driving = driving_scans(dte)
    Yd = Dict(k => v[:, driving] for (k, v) in Yte)
    println("test: $(size(Xte, 2)) scans ($(count(driving)) while driving); validation: $(size(Xva, 2))")
    hand, none = hand_estimates(Xte)

    totals = Dict(name => [0, 0] for name in NETWORKS)
    rows = ["| Estimate | Estimator | Weights | Beams in | MACs per scan | Parameters | Validation R² | " *
            "Test RMS error | Unit | Entries given | Within 1σ | Error correlation with large |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for est in ESTIMATES
        M = mask(est, Yd)
        h = hand[est][:, driving]
        ok = M .& isfinite.(h)
        push!(rows, "| `$est` | hand-written | – | 1081 | – | – | – | $(g(rmse((h .- Yd[est])[ok]))) | $(UNITS[est]) | " *
                    "$(round(Int, 100count(ok) / count(M)))% | – | – |")
        errs = Dict{Symbol,Vector{Float64}}()
        nets = Dict(name => load_net(weights_file(WEIGHTS_DIR, est, name)) for name in NETWORKS)
        stats = Dict{Symbol,Any}()
        for name in NETWORKS
            net = nets[name]
            μv, _ = means_sigmas(net, Xva)
            z = (μv .- Yva[est]) ./ Float64.(net.y_std)
            μ, σ = means_sigmas(net, Xte)
            err = (μ[:, driving].-Yd[est])[M]
            errs[name] = err
            m, p = macs(net), Lux.parameterlength(net.model)
            totals[name] .+= (m, p)
            stats[name] = (m, p, 1 - sum(abs2, z) / length(z), rmse(err), mean(abs.(err) .<= σ[:, driving][M]))
        end
        for name in NETWORKS
            m, p, r2, e, cal = stats[name]
            file = basename(weights_file(WEIGHTS_DIR, est, name))
            ρ = network_tier(name) === :small ? g(cor(errs[name], errs[:large]); d=2) : "–"
            push!(rows, "| `$est` | $name | [`$file`](../../weights/$file) | $(input_size(nets[name])) | $(commas(m)) | " *
                        "$(commas(p)) | $(g(r2)) | $(g(e)) | $(UNITS[est]) | 100% | $(round(Int, 100cal))% | $ρ |")
        end
        foreach(println, rows[end-3:end])
    end
    summary = ["| Network set | Runs on | Weights | Beams in | MACs per scan, all six | Parameters, all six |",
        "|---|---|---|---|---|---|"]
    where = Dict(:small_weak => "the car (edge)", :small_strong => "(for comparison)", :large => "the cloud")
    for name in NETWORKS
        net = load_net(weights_file(WEIGHTS_DIR, :e, name))
        push!(summary, "| $name | $(where[name]) | [`weights/$(name)_*.jld2`](../../weights) | $(input_size(net)) | " *
                       "$(commas(totals[name][1])) | $(commas(totals[name][2])) |")
    end
    mkpath(RESULTS_DIR)
    write(joinpath(RESULTS_DIR, "networks.md"), """
        # The networks: cost and accuracy

        Every network set against the hand-written estimator (LidarLaneEstimation.jl), on the tracks never
        seen in training ($(join(TEST_TRACKS, ", ")); $(count(driving)) scans while driving). Validation:
        $(join(VAL_TRACKS, ", ")) ($(size(Xva, 2)) scans). From [`evaluate.jl`](../evaluate.jl), which
        explains each column. The hand-written estimator found no lane in $(round(100none; digits=1))% of the
        test scans (it then gives zeros). The error distributions are in [errors.md](errors.md).

        $(join(summary, "\n"))

        $(join(rows, "\n"))
        """)
    println.(summary)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
