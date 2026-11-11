# How the networks' errors are distributed: a Student t, fitted to each.
#
#     julia -t auto --project=. errors.jl      # prints, and writes results/errors.md (and .toml)
#
# For every network (estimate × network set, see LidarLaneNets.NETWORKS): its
# errors (estimate − truth) on the held-out tracks' scans while driving, over
# the same entries as the RMS errors (profiles: the entries the car can see,
# pooled over the distance ahead); and the same errors divided by the
# network's own standard deviation. A Gaussian, a Laplace and a Student t
# were each fitted by maximum likelihood; the Student t had the least AIC for
# every network's errors, so it alone is reported: location μ, scale s,
# degrees of freedom ν (tails heavier the smaller ν; a Gaussian as ν → ∞, a
# Cauchy at ν = 1). How often each candidate won is checked and written too.

using Printf
using Random
using TOML

include("common.jl")
include("tdist.jl")   # fit_t

# --- the three fits (x: the errors); each returns (params, log-likelihood)

function fit_gauss(x)
    μ = sum(x) / length(x)
    σ = sqrt(sum(abs2, x .- μ) / length(x))
    return (; μ, σ), -length(x) * (log(σ) + 0.5log(2π) + 0.5)
end

function fit_laplace(x)
    μ = median(x)
    b = sum(abs, x .- μ) / length(x)
    return (; μ, b), -length(x) * (log(2b) + 1)
end

const MODELS = ((:gauss, fit_gauss, 2), (:laplace, fit_laplace, 2), (:t, fit_t, 3))
const NAMES = Dict(:gauss => "Gaussian", :laplace => "Laplace", :t => "Student t")

# Every model fitted to x, and the one with the least AIC.
function fit_all(x)
    fits = Dict(name => f(x) for (name, f, _) in MODELS)
    aic = Dict(name => 2k - 2fits[name][2] for (name, _, k) in MODELS)
    return fits, argmin(aic)
end

g(x; d=3) = string(round(x; sigdigits=d))
t_text(p) = "$(g(p.μ)) | $(g(p.s)) | $(g(p.ν; d=2))"

function main()
    rng = Xoshiro(1)
    Xte, Yte, dte = load_all(TEST_TRACKS)
    driving = driving_scans(dte)
    Yd = Dict(k => v[:, driving] for (k, v) in Yte)
    rows = ["| Estimate | Network | μ | s | ν | Unit | μ / σ | s / σ | ν / σ |", "|---|---|---|---|---|---|---|---|---|"]
    wins = Dict(:raw => Dict(n => 0 for (n, _, _) in MODELS), :std => Dict(n => 0 for (n, _, _) in MODELS))
    exceptions = String[]
    data = Dict{String,Any}()   # the fits, for plots.jl: estimate => network set => …
    for est in ESTIMATES, name in NETWORKS
        net = load_net(weights_file(WEIGHTS_DIR, est, name))
        μ, σ = means_sigmas(net, Xte)
        M = mask(est, Yd)
        err = (μ[:, driving].-Yd[est])[M]
        z = err ./ σ[:, driving][M]
        if length(err) > MAX_FIT
            keep = randperm(rng, length(err))[1:MAX_FIT]
            err, z = err[keep], z[keep]
        end
        fits_e, best_e = fit_all(err)
        fits_z, best_z = fit_all(z)
        wins[:raw][best_e] += 1
        wins[:std][best_z] += 1
        best_e === :t || push!(exceptions, "`$est`, $name (errors): $(NAMES[best_e])")
        best_z === :t || push!(exceptions, "`$est`, $name (errors over σ): $(NAMES[best_z])")
        push!(rows, "| `$est` | $name | $(t_text(fits_e[:t][1])) | $(UNITS[est]) | $(t_text(fits_z[:t][1])) |")
        pe, pz = fits_e[:t][1], fits_z[:t][1]
        get!(data, file_name(est), Dict{String,Any}())[String(name)] =
            Dict("mu" => pe.μ, "s" => pe.s, "nu" => pe.ν, "mu_over_sigma" => pz.μ, "s_over_sigma" => pz.s,
                "nu_over_sigma" => pz.ν)
        println(rows[end]); flush(stdout)
    end
    n = length(ESTIMATES) * length(NETWORKS)
    won = "The Student t had the least AIC of the three for $(wins[:raw][:t]) of $n networks' errors and " *
          "$(wins[:std][:t]) of $n of the errors over σ" *
          (isempty(exceptions) ? "." : " (otherwise: " * join(exceptions, "; ") * ").")
    mkpath(RESULTS_DIR)
    open(io -> TOML.print(io, data; sorted=true), joinpath(RESULTS_DIR, "errors.toml"), "w")
    write(joinpath(RESULTS_DIR, "errors.md"), """
        # How the networks' errors are distributed

        A Student t, fitted by maximum likelihood to every network's errors (estimate − truth) on the
        held-out tracks ($(join(TEST_TRACKS, ", "))) while driving, over the same entries as the RMS errors
        (profiles: the entries the car can see, pooled over the distance ahead; at most $(MAX_FIT) errors
        per fit, a random subset); and to the same errors divided by the network's own standard deviation
        (columns "/ σ": what the selector and the Kalman filter treat as a standard Gaussian). Location μ,
        scale s, degrees of freedom ν: the tails are heavier the smaller ν (a Gaussian as ν → ∞, a Cauchy
        at ν = 1). Its density: Γ((ν+1)/2) / (Γ(ν/2) √(νπ) s) · (1 + ((x − μ)/s)² / ν)^(−(ν+1)/2).

        $won A Gaussian and a Laplace were the other candidates. From [`errors.jl`](../errors.jl); the
        networks are those of [networks.md](networks.md).

        $(join(rows, "\n"))
        """)
    println(won)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
