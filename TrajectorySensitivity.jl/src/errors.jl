# How a network errs, as a model to draw from: what the oracle adds to the
# true estimates.
#
# A network gives, per scan and per entry j of its estimate, a value and its
# own standard deviation σⱼ. The model draws both:
#
#   errorⱼ = sⱼ ⋅ a(fⱼ) ⋅ Q(uⱼ)              (the network's own error distribution)
#   σⱼ = b(fⱼ) ⋅ exp(mⱼ + dⱼ ηⱼ),  ηⱼ = c ⋅ g(uⱼ) + √(1 − c²) ⋅ ξⱼ
#
# fⱼ is the situation (see `situation`): how much the lane bends, how far it
# is clear, ... from the true estimates. The networks err far more in some
# situations than in others (κ_profile's median error is some 70 times
# larger where the lane bends sharply than where it is straight), so a(f)
# and b(f) scale the error and σ with it, as measured.
#
# u and ξ are standard Gaussian vectors, correlated across the entries and
# over time: each is √p ⋅ (a slow part, first-order Gauss-Markov with time
# constant τ) + √(1 − p) ⋅ (a white part), so its correlation k scans apart
# is p ⋅ exp(−k Δt / τ), the form train/fusion.jl fits. g(u) = Φ⁻¹(2Φ(|u|) − 1)
# is the error's size as a standard Gaussian, so c ties the network's σ to
# how wrong it is. Q is the quantile function of the network's errors, each
# over its entry's scale sⱼ (the median |error|), at the Gaussian scores
# `SCORES` (a Gaussian copula): the measured distribution itself. (The
# Student t fitted to the same errors in LidarLaneNets.jl's
# train/results/errors.md matches their bulk, but some have ν < 1, whose
# tails reach far past any error the networks make.) Everything is measured
# by scripts/error_model.jl. A gain k scales both the error and σ: a network
# k times as wrong that knows it.

"""
    GaussProcess(corr, persist, τ)

A standard Gaussian vector process: correlation `corr` across its entries,
and `persist ⋅ exp(−lag / τ)` between scans `lag` seconds apart.
"""
struct GaussProcess
    L::Matrix{Float64}   # lower Cholesky factor of the correlation
    persist::Float64
    τ::Float64
    function GaussProcess(corr::AbstractMatrix, persist::Real, τ::Real)
        C = Symmetric(Matrix{Float64}(corr))
        return new(Matrix(cholesky(C + 1e-9I).L), clamp(Float64(persist), 0.0, 1.0), Float64(τ))
    end
end

"""
    correlation(g::GaussProcess) -> Matrix

The correlation across the entries of `g`.
"""
correlation(g::GaussProcess) = (C = g.L * g.L'; d = sqrt.(diag(C)); C ./ (d * d'))

Base.length(g::GaussProcess) = size(g.L, 1)

"""
    NetworkErrors

How one network (`network`, for `estimate`) errs: its errors' scale per
entry `s`, times `err_scale` in the situations `at` (see [`situation`](@ref);
linear between them, the end values past them), their distribution over it
(`quantiles`, at the Gaussian scores [`SCORES`](@ref)) and their Gaussian
process `u`; log σ's mean and standard deviation per entry, σ times
`σ_scale` in the situations `at`, its coupling `c` to the error's size, and
the process `ξ` of the rest; and
the noise parameters the Kalman filter is given for this network
(`filter`, a LidarLaneNets `EstimateNoise`, as train/fusion.jl measures
them).
"""
struct NetworkErrors
    estimate::Symbol
    network::Symbol
    s::Vector{Float64}
    at::Vector{Float64}
    err_scale::Vector{Float64}
    σ_scale::Vector{Float64}
    quantiles::Vector{Float64}
    u::GaussProcess
    logσ_mean::Vector{Float64}
    logσ_std::Vector{Float64}
    c::Float64
    ξ::GaussProcess
    filter::EstimateNoise
end

"""
    typical_σ(m::NetworkErrors) -> Vector

The network's median σ per entry (over all situations).
"""
typical_σ(m::NetworkErrors) = exp.(m.logσ_mean)

"""
    situation(estimate, labels::LaneLabels) -> Vector

What a network's errors on `estimate` depend on, per entry, from the true
estimates `labels`: for `e` and `ψe`, how much the lane bends beside the
car (|κ_profile[1]|); for `sight`, the sight; for the rest, the size of the
entry's own true value (|κ| ahead, the room to each side, the free distance
along each arc).
"""
function situation(est::Symbol, l)
    est === :e || est === :ψe ? [abs(l.κ_profile[1])] : est === :sight ? copy(l.sight) : abs.(l[est])
end

# Piecewise linear through (x, y), the end values past the ends.
function _pwl(x, y, q)
    q <= x[1] && return y[1]
    q >= x[end] && return y[end]
    i = searchsortedlast(x, q)
    return y[i] + (q - x[i]) / (x[i+1] - x[i]) * (y[i+1] - y[i])
end

"""
    err_factor(m::NetworkErrors, f), σ_factor(m, f)

How much larger the errors, and the σ, are in the situation `f` than
typical.
"""
err_factor(m::NetworkErrors, f::Real) = _pwl(m.at, m.err_scale, f)
σ_factor(m::NetworkErrors, f::Real) = _pwl(m.at, m.σ_scale, f)

# --- the error distribution, through Gaussian scores

"""
    SCORES

The Gaussian scores at which an error distribution's quantiles are kept:
−4 to 4 in steps of 0.02 (probabilities from 3 ⋅ 10⁻⁵ to 1 − 3 ⋅ 10⁻⁵).
"""
const SCORES = range(-4.0, 4.0; length=401)

_Φ(u) = 0.5erfc(-u / sqrt(2))

"""
    from_score(quantiles, u)

The value at Gaussian score `u` of the distribution with `quantiles` at
[`SCORES`](@ref) (linear between them; the end values past them).
"""
function from_score(q::AbstractVector, u::Real)
    u <= first(SCORES) && return q[1]
    u >= last(SCORES) && return q[end]
    x = (u - first(SCORES)) / step(SCORES) + 1
    i = floor(Int, x)
    return q[i] + (x - i) * (q[i+1] - q[i])
end

"""
    to_score(quantiles, x)

The Gaussian score of the value `x` in the distribution with `quantiles` at
[`SCORES`](@ref): the inverse of [`from_score`](@ref) (the middle of a run
of equal quantiles; ±4 past the ends).
"""
function to_score(q::AbstractVector, x::Real)
    x < q[1] && return first(SCORES)
    x > q[end] && return last(SCORES)
    lo, hi = searchsortedfirst(q, x), searchsortedlast(q, x)
    lo > hi && (f = (x - q[hi]) / (q[lo] - q[hi]); return SCORES[hi] + f * step(SCORES))   # between two
    return (SCORES[lo] + SCORES[hi]) / 2                                                   # equal to some
end

# --- drawing

"""
    size_score(u)

How large a standard Gaussian `u` is, as a standard Gaussian: Φ⁻¹(2Φ(|u|) − 1).
"""
size_score(u) = -sqrt(2) * erfcinv(2clamp(erf(abs(u) / sqrt(2)), 1e-15, 1 - 1e-15))

"""
    ErrorDraws(model::NetworkErrors, rng)

Draws a network's errors and σ scan after scan ([`draw!`](@ref)), from its
stationary distribution on.
"""
mutable struct ErrorDraws{R<:AbstractRNG}
    model::NetworkErrors
    rng::R
    u_slow::Vector{Float64}
    ξ_slow::Vector{Float64}
    started::Bool
end

ErrorDraws(m::NetworkErrors, rng::AbstractRNG) =
    ErrorDraws(m, rng, zeros(length(m.u)), zeros(length(m.ξ)), false)

# One step of a GaussProcess `dt` seconds on (the first, stationary).
function _step!(slow, g::GaussProcess, dt, started, rng)
    n = length(slow)
    a = started ? exp(-dt / g.τ) : 0.0
    slow .= a .* slow .+ sqrt(1 - a^2) .* (g.L * randn(rng, n))
    return sqrt(g.persist) .* slow .+ sqrt(1 - g.persist) .* (g.L * randn(rng, n))
end

"""
    draw!(d::ErrorDraws, dt, f) -> (error, σ)

The network's error and its own σ for the next scan, `dt` seconds after
the last, in the situation `f` (per entry, see [`situation`](@ref)).
"""
function draw!(d::ErrorDraws, dt::Real, f::AbstractVector)
    m = d.model
    u = _step!(d.u_slow, m.u, dt, d.started, d.rng)
    ξ = _step!(d.ξ_slow, m.ξ, dt, d.started, d.rng)
    d.started = true
    η = m.c .* size_score.(u) .+ sqrt(1 - m.c^2) .* ξ
    σ = σ_factor.(Ref(m), f) .* exp.(m.logσ_mean .+ m.logσ_std .* η)
    err = [m.s[j] * err_factor(m, f[j]) * from_score(m.quantiles, u[j]) for j in eachindex(u)]
    return err, σ
end

# --- reading and writing

_matrix(rows) = reduce(vcat, (permutedims(Float64.(r)) for r in rows))
_rows(M::AbstractMatrix) = [collect(Float64, r) for r in eachrow(M)]

"""
    load_errors(path; network=:small_weak) -> NamedTuple

The error models of a network set, one [`NetworkErrors`](@ref) per
estimate, from a TOML file written by scripts/error_model.jl.
"""
function load_errors(path::AbstractString; network::Symbol=:small_weak)
    d = TOML.parsefile(path)[String(network)]
    one(est) = begin
        x = d[file_name(est)]
        NetworkErrors(est, network, Float64.(x["s"]), Float64.(x["at"]), Float64.(x["err_scale"]),
            Float64.(x["sigma_scale"]), Float64.(x["quantiles"]),
            GaussProcess(_matrix(x["u_corr"]), x["u_persist"], x["u_tau"]),
            Float64.(x["logsigma_mean"]), Float64.(x["logsigma_std"]), x["c"],
            GaussProcess(_matrix(x["xi_corr"]), x["xi_persist"], x["xi_tau"]),
            EstimateNoise(persist=x["filter_persist"], τ=x["filter_tau"], σ_bias=x["filter_sigma_bias"],
                ν_small=x["filter_nu"], ν_large=x["filter_nu_large"]))
    end
    return NamedTuple{ESTIMATES}(Tuple(one(est) for est in ESTIMATES))
end

"""
    error_entry(m::NetworkErrors) -> Dict

One estimate's section of the TOML file [`load_errors`](@ref) reads.
"""
function error_entry(m::NetworkErrors)
    corr(g) = _rows(correlation(g))
    f = m.filter
    return Dict{String,Any}("s" => m.s, "at" => m.at, "err_scale" => m.err_scale, "sigma_scale" => m.σ_scale,
        "quantiles" => m.quantiles, "u_corr" => corr(m.u), "u_persist" => m.u.persist,
        "u_tau" => m.u.τ, "logsigma_mean" => m.logσ_mean, "logsigma_std" => m.logσ_std, "c" => m.c,
        "xi_corr" => corr(m.ξ), "xi_persist" => m.ξ.persist, "xi_tau" => m.ξ.τ, "filter_persist" => f.persist,
        "filter_tau" => f.τ, "filter_sigma_bias" => f.σ_bias, "filter_nu" => f.ν_small, "filter_nu_large" => f.ν_large)
end
