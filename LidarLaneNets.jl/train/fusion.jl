# The noise parameters the tracker fuses the car's and the cloud's results
# with, measured on the validation tracks' drives (sequences of scans at
# 40 Hz), into ../weights/fusion.toml (see LidarLaneNets.load_noise).
#
#     julia -t auto --project=. fusion.jl
#
# For each estimate and small (car) network set:
# - persist, tau: the error's correlation from scan to scan, ρ(k) at k scans
#   apart, fitted as persist ⋅ a^k (k = 1 … 20): the share of the error
#   variance that persists, and how long (tau = −25 ms / log a);
# - sigma_bias: the standard deviation of that persisting part;
# - nu: the Student t degrees of freedom of the errors over the network's own
#   σ (a tail weight for the robust updates).
# For the large (cloud) set, nu alone.

include("errors.jl")   # data, means_sigmas, mask, fit_t
using TOML

const LAGS = 1:20
const DT = 0.025

# The driving scans of the validation tracks, as runs of consecutive scans
# (indices into the loaded matrix).
function driving_runs(files)
    runs, offset = Vector{Vector{Int}}(), 0
    for f in files
        drv = findall(.!f["static"]) .+ offset
        n = length(drv) ÷ 2   # collect.jl: two drives per track, one after the other
        push!(runs, drv[1:n], drv[n+1:end])
        offset += length(f["static"])
    end
    return runs
end

# Correlation of the errors k scans apart, over the runs and every entry that counts.
function autocorr(E, M, runs, k)
    num, a2, b2 = 0.0, 0.0, 0.0
    for run in runs, t in 1:length(run)-k
        i, j = run[t], run[t+k]
        for r in axes(E, 1)
            (M[r, i] && M[r, j]) || continue
            num += E[r, i] * E[r, j]
            a2 += E[r, i]^2
            b2 += E[r, j]^2
        end
    end
    return num / sqrt(a2 * b2)
end

# persist ⋅ a^k fitted to ρ(k) by least squares (a on a grid, persist in closed form).
function fit_persistence(ρ)
    best = (Inf, 0.0, 0.5)
    for a in 0.5:0.0005:0.9995
        w = a .^ LAGS
        f = clamp(sum(ρ .* w) / sum(w .^ 2), 0.0, 1.0)
        sse = sum(abs2, ρ .- f .* w)
        sse < best[1] && (best = (sse, f, a))
    end
    return best[2], best[3]
end

function main()
    X, Y, files = load_all(VAL_TRACKS)
    runs = driving_runs(files)
    drive = reduce(vcat, runs)
    M = Dict(est => mask(est, Y) for est in ESTIMATES)
    out = Dict{String,Any}()
    rng = Xoshiro(1)
    for name in NETWORKS
        section = Dict{String,Any}()
        for est in ESTIMATES
            net = load_net(weights_file(WEIGHTS_DIR, est, name))
            μ, σ = means_sigmas(net, X)
            E = μ .- Y[est]
            z = (E./σ)[:, drive][M[est][:, drive]]
            length(z) > MAX_FIT && (z = z[randperm(rng, length(z))[1:MAX_FIT]])
            ν = fit_t(z)[1].ν
            entry = Dict{String,Any}("nu" => round(ν; sigdigits=3))
            if network_tier(name) === :small
                ρ = [autocorr(E, M[est], runs, k) for k in LAGS]
                f, a = fit_persistence(ρ)
                errs = E[:, drive][M[est][:, drive]]
                entry["persist"] = round(f; sigdigits=3)
                entry["tau"] = round(-DT / log(a); sigdigits=3)
                entry["sigma_bias"] = round(sqrt(f * mean(abs2, errs)); sigdigits=3)
                @printf("%-12s %-10s persist %.2f  tau %.3f s  sigma_bias %.4g  nu %.3g   (ρ1 %.2f, ρ5 %.2f, ρ20 %.2f)\n",
                    name, est, f, -DT / log(a), entry["sigma_bias"], ν, ρ[1], ρ[5], ρ[20])
            else
                @printf("%-12s %-10s nu %.3g\n", name, est, ν)
            end
            section[LidarLaneNets._FILE_NAMES[est]] = entry
        end
        out[String(name)] = section
    end
    open(joinpath(WEIGHTS_DIR, "fusion.toml"), "w") do io
        println(io, "# Noise parameters for fusing the car's and the cloud's results (LidarLaneNets.load_noise),")
        println(io, "# measured on the validation tracks by train/fusion.jl. Per estimate: persist, tau [s] and")
        println(io, "# sigma_bias (the car network's error that persists from scan to scan), nu (Student t tail")
        println(io, "# weight of the errors over the network's own σ).")
        TOML.print(io, out; sorted=true)
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
