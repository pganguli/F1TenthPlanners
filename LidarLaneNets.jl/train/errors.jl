# What the networks' errors look like: the distribution that fits them best.
#
#     julia -t auto --project=. errors.jl      # prints, and writes results/errors.md
#
# For every network (estimate × network set, see LidarLaneNets.NETWORKS): its errors
# (estimate − truth) on the held-out tracks' scans while driving, over the
# same entries as the RMS errors (profiles: the entries the car can see,
# pooled over the distance ahead). Three candidates, each fitted by maximum
# likelihood:
#
# - Gaussian (μ, σ);
# - Laplace (μ, b): heavier tails, exponential;
# - Student t (μ, s, ν): heavier still, polynomial tails; ν is how heavy (a
#   Gaussian as ν → ∞; a Cauchy at ν = 1).
#
# The best by AIC is named. Tail check: how often each fitted model says the
# error is beyond the observed 99.9th percentile of |error − μ| (0.1% if the
# model's tails are right). The same fit is also done on the errors divided by
# the network's own standard deviation (what the selector and the Kalman
# filter treat as a standard Gaussian).

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

function row(est, name, x, unit)
    fits, aic, best, tails = fit_all(x)
    G, L, T = fits[:gauss][1], fits[:laplace][1], fits[:t][1]
    ΔL, ΔT = aic[:gauss] - aic[:laplace], aic[:gauss] - aic[:t]
    return "| `$est` | $name | $(NAMES[best]) | μ $(g(G.μ)), σ $(g(G.σ)) | μ $(g(L.μ)), b $(g(L.b)) | " *
           "μ $(g(T.μ)), s $(g(T.s)), **ν $(g(T.ν; d=2))** | $(g(ΔL)) / $(g(ΔT)) | " *
           "$(g(100tails[:gauss]; d=2)) / $(g(100tails[:laplace]; d=2)) / $(g(100tails[:t]; d=2)) |" *
           (isempty(unit) ? "" : " $unit |")
end

const HEAD = "| Estimate | Network | Best fit (AIC) | Gaussian | Laplace | Student t | AIC better than Gaussian by: " *
             "Laplace / t | Beyond the observed 99.9th percentile, % (Gaussian / Laplace / t; 0.1 is right) |"
const RULE = "|---|---|---|---|---|---|---|---|"

function main()
    rng = Xoshiro(1)
    Xte, Yte, dte = load_all(TEST_TRACKS)
    driving = .!reduce(vcat, (x["static"] for x in dte))
    Yd = Dict(k => v[:, driving] for (k, v) in Yte)
    unit = Dict(:e => "m", :ψe => "rad", :κ_profile => "1/m", :widths => "m", :sight => "m", :clearance => "m")
    raw = [HEAD * " Unit |", RULE * "---|"]
    std = [HEAD, RULE]
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
        push!(raw, row(est, name, err, unit[est]))
        push!(std, row(est, name, z, ""))
        println(raw[end]); flush(stdout)
    end
    mkpath(RESULTS_DIR)
    write(joinpath(RESULTS_DIR, "errors.md"), """
        # How the networks' errors are distributed

        Errors (estimate − truth) on the held-out tracks ($(join(TEST_TRACKS, ", "))) while driving, over the
        same entries as the RMS errors (profiles: the entries the car can see, pooled over the distance
        ahead; at most $(MAX_FIT) errors per fit, a random subset). Each candidate fitted by maximum
        likelihood. Student t: location μ, scale s, degrees of freedom ν (tails heavier the smaller ν; a
        Gaussian as ν → ∞, a Cauchy at ν = 1). AIC differences: how much better than the Gaussian (larger is
        better; differences above ~10 are decisive). Tail check: how often each fitted model puts an error
        beyond the observed 99.9th percentile of |error − μ|; a model whose tails are right says 0.1%.
        From [`errors.jl`](../errors.jl). The networks are those of [networks.md](networks.md).

        ## The errors

        $(join(raw, "\n"))

        ## The errors over the network's own standard deviation

        What the selector and the Kalman filter assume is a standard Gaussian (μ 0, σ 1).

        $(join(std, "\n"))
        """)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
