# The Student t distribution: fitting it by maximum likelihood, and its tail.

using SpecialFunctions: loggamma, beta_inc
using Statistics: median

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

"""
    fit_t(x) -> ((μ, s, ν), log-likelihood)

The Student t that fits `x` best: ν over a grid (log-spaced 0.5 to 500),
then refined around the best.
"""
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

# P(|X − μ| > q) for a Student t of scale s.
t_tail(q, s, ν) = (t = q / s; beta_inc(ν / 2, 0.5, ν / (ν + t^2))[1])
