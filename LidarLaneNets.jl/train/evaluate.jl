# How the networks do, and what they cost: every network set (see
# LidarLaneNets.NETWORKS) against the hand-written estimator
# (LidarLaneEstimation.jl), on the tracks they never saw in training.
#
#     julia -t auto --project=. evaluate.jl      # prints, and writes results/networks.md (and .toml)
#
# Per estimate and network:
# - MACs: multiply-accumulates for one scan, counted layer by layer from the
#   saved network (convolutions and dense layers); a large network includes
#   its small part. Parameters.
# - Validation R²: 1 − the mean squared error of the normalized targets on
#   the validation tracks (what training picks the epoch by; 1 is perfect).
# - Test RMS error, on the held-out tracks' scans while driving: profiles over
#   the entries the car can see (no farther ahead than the true sight); the
#   hand-written estimator, and widths, sight and clearance measured in the
#   scan (along the center small_weak gives; see sources.jl), over the
#   entries they give.
# - Within 1σ: how often the truth is within the network's own standard
#   deviation (68% if calibrated).
# - Error correlation with the large network's (for the small sets): how much
#   a cloud result adds to the car's for the same scan.

using LinearAlgebra
using Lux
using Printf
using StanleyController: Path, tracking_error
using Statistics: median, quantile
using TOML

include("common.jl")

include("sources.jl")   # hand_estimates, geometric_estimates

# Where the corridor's walls end up. The planner rebuilds the lane center from
# e, ψe and κ_profile and puts each wall at its width along the center's
# normal (LidarRacer.lane_center); the labels give the widths from the true
# center. So a width measured from an estimated center can miss its label and
# still put the wall where it is: what counts is the distance from each wall
# point (within the true sight) to the true wall (the true center and widths,
# likewise).
function _seg_dist(px, py, xs, ys)
    best = Inf
    for k in 1:length(xs)-1
        ax, ay = xs[k], ys[k]
        dx, dy = xs[k+1] - ax, ys[k+1] - ay
        L2 = dx^2 + dy^2
        t = L2 < 1e-12 ? 0.0 : clamp(((px - ax) * dx + (py - ay) * dy) / L2, 0.0, 1.0)
        best = min(best, hypot(px - ax - t * dx, py - ay - t * dy))
    end
    return best
end

function _walls(e, ψe, κ, w)
    np = length(PROFILE_S)
    x, y, h = center_line(e, ψe, κ; reach=PROFILE_LENGTH)
    return (x .- sin.(h) .* w[1:np], y .+ cos.(h) .* w[1:np]), (x .+ sin.(h) .* w[np+1:end], y .- cos.(h) .* w[np+1:end])
end

"""
    wall_distances(Y, E) -> Vector

For each scan (column) of the labels `Y`, the distance [m] from every wall
point of the corridor that the estimates `E` (`:e`, `:ψe`, `:κ_profile`,
`:widths`, as matrices) give, within the true sight, to the true wall on the
same side.
"""
function wall_distances(Y, E)
    out = [Float64[] for _ in 1:Threads.maxthreadid()]
    Threads.@threads :static for j in axes(Y[:e], 2)
        vis = findall(PROFILE_S .<= Y[:sight][1, j])
        length(vis) < 2 && continue
        c = (E[:e][1, j], E[:ψe][1, j], E[:κ_profile][:, j])
        isfinite(c[1]) && isfinite(c[2]) || continue
        TL, TR = _walls(Y[:e][1, j], Y[:ψe][1, j], Y[:κ_profile][:, j], Y[:widths][:, j])
        EL, ER = _walls(c..., E[:widths][:, j])
        for (est, tru) in ((EL, TL), (ER, TR)), i in vis
            isfinite(est[1][i]) && isfinite(est[2][i]) &&
                push!(out[Threads.threadid()], _seg_dist(est[1][i], est[2][i], tru[1][vis], tru[2][vis]))
        end
    end
    return reduce(vcat, out)
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
    # widths, sight and clearance measured in the scan, along the center the car's networks give
    center = Dict(est => first(means_sigmas(load_net(weights_file(WEIGHTS_DIR, est, :small_weak)), Xte))
                  for est in (:e, :ψe, :κ_profile))
    geo = geometric_estimates(Xte, center[:e], center[:ψe], center[:κ_profile])

    totals = Dict(name => [0, 0] for name in NETWORKS)
    data = Dict{String,Any}()   # the numbers, for plots.jl: estimate => network set => …
    rows = ["| Estimate | Estimator | Weights | Beams in | MACs per scan | Parameters | Validation R² | " *
            "Test RMS error | Unit | Entries given | Within 1σ | Error correlation with large |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for est in ESTIMATES
        M = mask(est, Yd)
        h = hand[est][:, driving]
        ok = M .& isfinite.(h)
        push!(rows, "| `$est` | hand-written | – | 1081 | – | – | – | $(g(rmse((h .- Yd[est])[ok]))) | $(UNITS[est]) | " *
                    "$(round(Int, 100count(ok) / count(M)))% | – | – |")
        if est in GEOMETRIC_ESTIMATES
            h = geo[est][:, driving]
            ok = M .& isfinite.(h)
            push!(rows, "| `$est` | measured in the scan | – | 1081 | – | – | – | $(g(rmse((h .- Yd[est])[ok]))) | " *
                        "$(UNITS[est]) | $(round(Int, 100count(ok) / count(M)))% | – | – |")
        end
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
        data[file_name(est)] = Dict(String(name) => Dict("macs" => stats[name][1], "parameters" => stats[name][2],
            "validation_r2" => stats[name][3], "test_rmse" => stats[name][4], "within_1sigma" => stats[name][5])
                                    for name in NETWORKS)
        for name in NETWORKS
            m, p, r2, e, cal = stats[name]
            file = basename(weights_file(WEIGHTS_DIR, est, name))
            ρ = network_tier(name) === :small ? g(cor(errs[name], errs[:large]); d=2) : "–"
            push!(rows, "| `$est` | $name | [`$file`](../../weights/$file) | $(input_size(nets[name])) | $(commas(m)) | " *
                        "$(commas(p)) | $(g(r2)) | $(g(e)) | $(UNITS[est]) | 100% | $(round(Int, 100cal))% | $ρ |")
        end
        foreach(println, rows[end-(est in GEOMETRIC_ESTIMATES ? 4 : 3):end])
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
    open(io -> TOML.print(io, data; sorted=true), joinpath(RESULTS_DIR, "networks.toml"), "w")
    write(joinpath(RESULTS_DIR, "networks.md"), """
        # The networks: cost and accuracy

        Every network set against the hand-written estimator (LidarLaneEstimation.jl), on the tracks never
        seen in training ($(join(TEST_TRACKS, ", ")); $(count(driving)) scans while driving). Validation:
        $(join(VAL_TRACKS, ", ")) ($(size(Xva, 2)) scans). From [`evaluate.jl`](../evaluate.jl), which
        explains each column. The hand-written estimator found no lane in $(round(100none; digits=1))% of the
        test scans (it then gives zeros). The error distributions are in [errors.md](errors.md).

        $(join(summary, "\n"))

        $(join(rows, "\n"))

        ## Where the corridor's walls end up

        The path planner rebuilds the lane center from `e`, `ψe` and `κ_profile` and puts each wall at its
        width along the center's normal. The `widths` labels are measured from the true center, so widths
        measured in the scan from an estimated center miss their labels by that center's error, yet put
        the walls where they are. What the planner needs is the walls: the distance from every wall point
        (within the true sight) to the true wall.

        $(join(walls_table(Xte, Yd, driving, hand, geo), "\n"))
        """)
    println.(summary)
end

# The corridor walls' placement (see wall_distances), on the test tracks' driving scans.
function walls_table(Xte, Yd, driving, hand, geo)
    est(name) = Dict(e => first(means_sigmas(load_net(weights_file(WEIGHTS_DIR, e, name)), Xte))[:, driving]
                     for e in (:e, :ψe, :κ_profile, :widths))
    sw = est(:small_weak)
    sources = [("hand-written", Dict(e => hand[e][:, driving] for e in (:e, :ψe, :κ_profile, :widths))),
        ("small_weak", sw), ("large", est(:large)),
        ("small_weak's center, widths measured in the scan", merge(sw, Dict(:widths => geo[:widths][:, driving])))]
    rows = ["| Center and widths from | Wall points | RMS distance to the true wall [m] | Median [m] | 95th percentile [m] |",
        "|---|---|---|---|---|"]
    for (name, E) in sources
        d = wall_distances(Yd, E)
        push!(rows, "| $name | $(commas(length(d))) | $(g(rmse(d); d=3)) | $(g(median(d); d=3)) | $(g(quantile(d, 0.95); d=3)) |")
        println(rows[end])
    end
    return rows
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
