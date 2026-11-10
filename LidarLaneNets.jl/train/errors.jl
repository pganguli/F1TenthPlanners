# How the networks' errors are distributed: a Student t, fitted to each.
#
#     julia -t auto --project=. errors.jl      # prints, and writes results/errors.md
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

include("evaluate.jl")   # data loading, means_sigmas, mask
using Random
using SpecialFunctions: loggamma, beta_inc, erfc

const MAX_FIT = 200_000   # errors per fit (a random subset past this)

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

t_logpdf(z, ν) = loggamma((ν + 1) / 2) - loggamma(ν / 2) - 0.5log(ν * π) - (ν + 1) / 2 * log1p(z^2 / ν)

# Student t at a given ν: location and scale by EM (iteratively reweighted).
function fit_t_at(x, ν; iters=200)
    μ = median(x)
    s = 1.4826 * median(abs.(x .- μ)) + eps()
    for _ in 1:iters
        w = (ν + 1) ./ (ν .+ ((x .- μ) ./ s) .^ 2)
        μn = sum(w .* x) / sum(w)
        sn = sqrt(sum(w .* (x .- μn) .^ 2) / length(x))
        done = abs(μn - μ) < 1e-10 * s && abs(sn - s) < 1e-9 * s
        μ, s = μn, sn
        done && break
    end
    ll = sum(t_logpdf((xi - μ) / s, ν) for xi in x) - length(x) * log(s)
    return μ, s, ll
end

# Student t: ν over a grid (log-spaced 0.5 to 500), then refined around the best.
function fit_t(x)
    best = (μ=0.0, s=1.0, ν=1.0, ll=-Inf)
    for ν in exp.(range(log(0.5), log(500), length=40))
        μ, s, ll = fit_t_at(x, ν)
        ll > best.ll && (best = (; μ, s, ν, ll))
    end
    for ν in exp.(range(log(best.ν / 1.2), log(best.ν * 1.2), length=11))
        μ, s, ll = fit_t_at(x, ν)
        ll > best.ll && (best = (; μ, s, ν, ll))
    end
    return (μ=best.μ, s=best.s, ν=best.ν), best.ll
end

# P(|X − μ| > q) under each fitted model.
tail(::Val{:gauss}, p, q) = erfc(q / (p.σ * sqrt(2)))
tail(::Val{:laplace}, p, q) = exp(-q / p.b)
tail(::Val{:t}, p, q) = (t = q / p.s; beta_inc(p.ν / 2, 0.5, p.ν / (p.ν + t^2))[1])

const MODELS = ((:gauss, fit_gauss, 2), (:laplace, fit_laplace, 2), (:t, fit_t, 3))
const NAMES = Dict(:gauss => "Gaussian", :laplace => "Laplace", :t => "Student t")

function fit_all(x)
    fits = Dict(name => f(x) for (name, f, _) in MODELS)
    aic = Dict(name => 2k - 2fits[name][2] for (name, _, k) in MODELS)
    best = argmin(aic)
    μb = fits[best][1].μ
    q = quantile(abs.(x .- μb), 0.999)
    tails = Dict(name => tail(Val(name), fits[name][1], q) for (name, _, _) in MODELS)
    return fits, aic, best, tails
end

g(x; d=3) = string(round(x; sigdigits=d))

tparams(x) = fit_t(x)[1]
t_text(p) = "$(g(p.μ)) | $(g(p.s)) | $(g(p.ν; d=2))"

function main()
    rng = Xoshiro(1)
    Xte, Yte, dte = load_all(TEST_TRACKS)
    driving = .!reduce(vcat, (x["static"] for x in dte))
    Yd = Dict(k => v[:, driving] for (k, v) in Yte)
    unit = Dict(:e => "m", :ψe => "rad", :κ_profile => "1/m", :widths => "m", :sight => "m", :clearance => "m")
    rows = ["| Estimate | Network | μ | s | ν | Unit | μ / σ | s / σ | ν / σ |", "|---|---|---|---|---|---|---|---|---|"]
    wins = Dict(:raw => Dict(n => 0 for (n, _, _) in MODELS), :std => Dict(n => 0 for (n, _, _) in MODELS))
    exceptions = String[]
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
        fits_e, _, best_e, _ = fit_all(err)
        fits_z, _, best_z, _ = fit_all(z)
        wins[:raw][best_e] += 1
        wins[:std][best_z] += 1
        best_e === :t || push!(exceptions, "`$est`, $name (errors): $(NAMES[best_e])")
        best_z === :t || push!(exceptions, "`$est`, $name (errors over σ): $(NAMES[best_z])")
        push!(rows, "| `$est` | $name | $(t_text(fits_e[:t][1])) | $(unit[est]) | $(t_text(fits_z[:t][1])) |")
        println(rows[end]); flush(stdout)
    end
    n = length(ESTIMATES) * length(NETWORKS)
    won = "The Student t had the least AIC of the three for $(wins[:raw][:t]) of $n networks' errors and " *
          "$(wins[:std][:t]) of $n of the errors over σ" *
          (isempty(exceptions) ? "." : " (otherwise: " * join(exceptions, "; ") * ").")
    mkpath(RESULTS_DIR)
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
