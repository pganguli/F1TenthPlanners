# Sensitivity without simulation: the closed loop linearized about one
# situation, and the sets its states can reach (zonotopes) when an estimate
# is biased or late.

"""
    Zonotope(center, generators)

The set `{center + G ξ : ξ ∈ [-1, 1]ᵐ}`, `G` the matrix of `generators`
(one per column).
"""
struct Zonotope
    center::Vector{Float64}
    G::Matrix{Float64}
end

Zonotope(c::AbstractVector, G::AbstractMatrix) = Zonotope(collect(Float64, c), Matrix{Float64}(G))

"""
    box(ε) -> Zonotope

The box `[-ε, ε]` (one generator per entry).
"""
box(ε::AbstractVector) = Zonotope(zeros(length(ε)), Diagonal(collect(Float64, ε)))

"""
    linear_map(M, Z) -> Zonotope

`{M x : x ∈ Z}`.
"""
linear_map(M::AbstractMatrix, Z::Zonotope) = Zonotope(M * Z.center, M * Z.G)

"""
    minkowski_sum(Z1, Z2) -> Zonotope

`{x + y : x ∈ Z1, y ∈ Z2}`.
"""
minkowski_sum(a::Zonotope, b::Zonotope) = Zonotope(a.center .+ b.center, hcat(a.G, b.G))

"""
    interval_hull(Z) -> (lo, hi)

The smallest box holding `Z`.
"""
interval_hull(Z::Zonotope) = (r = vec(sum(abs, Z.G; dims=2)); (Z.center .- r, Z.center .+ r))

"""
    widths(Z) -> Vector

How wide `Z` is along each coordinate (its interval hull's sides).
"""
widths(Z::Zonotope) = 2 .* vec(sum(abs, Z.G; dims=2))

"""
    jacobian(f, x; h=1e-6) -> Matrix

The Jacobian of `f` at `x` by central differences, step `h` (relative to
`max(1, |xⱼ|)`).
"""
function jacobian(f, x::AbstractVector; h::Real=1e-6)
    x = collect(Float64, x)
    y0 = f(x)
    J = Matrix{Float64}(undef, length(y0), length(x))
    for j in eachindex(x)
        dj = h * max(1.0, abs(x[j]))
        xp, xm = copy(x), copy(x)
        xp[j] += dj
        xm[j] -= dj
        J[:, j] = (f(xp) .- f(xm)) ./ (2dj)
    end
    return J
end

# ∫₀ᵗ exp(Aη) dη ⋅ B and exp(A t), by one matrix exponential.
function _integrals(A, B, t)
    n, m = size(A, 1), size(B, 2)
    M = exp([A B; zeros(m, n + m)] .* t)
    return M[1:n, 1:n], M[1:n, n+1:n+m]
end

"""
    discretize(A, B, Bw, T, D) -> (Φ, Γ_old, Γ_new, Γw, steps)

The continuous system `ẋ = A x + B u + Bw w`, sampled every `T` seconds,
whose input `u` (computed at a sample) takes effect `D` seconds after it
and holds until the next one takes effect, while `w` holds from sample to
sample. With `D = steps ⋅ T + D′` (`0 ≤ D′ < T`):

    x[k+1] = Φ x[k] + Γ_old u[k-steps-1] + Γ_new u[k-steps] + Γw w[k]
"""
function discretize(A, B, Bw, T, D)
    steps = floor(Int, D / T + 1e-12)
    Dp = D - steps * T
    Φ, _ = _integrals(A, B, T)
    _, Γnew = _integrals(A, B, T - Dp)
    _, Γpart = _integrals(A, B, Dp)
    Γold = exp(A .* (T - Dp)) * Γpart
    _, Γw = _integrals(A, Bw, T)
    return Φ, Γold, Γnew, Γw, steps
end

"""
    stable(A) -> Bool

Whether the discrete-time system `x[k+1] = A x[k]` is stable (every
eigenvalue inside the unit circle).
"""
stable(A::AbstractMatrix) = maximum(abs, eigvals(A)) < 1 - 1e-9

"""
    bias_set(A, Bb, C, ε) -> Zonotope

Where the outputs `C x` of `x[k+1] = A x[k] + Bb b` settle (`A` stable)
for every constant `b` in the box `[-ε, ε]`: the final reachable set.
"""
bias_set(A, Bb, C, ε) = Zonotope(zeros(size(C, 1)), C * ((I - A) \ (Bb * Diagonal(collect(Float64, ε)))))

"""
    disturbance_set(A, Bw, C, ε; tol=1e-9, max_steps=100_000) -> Zonotope

Every output `C x` that `x[k+1] = A x[k] + Bw w[k]` (`A` stable, from
`x = 0`) can reach with each `w[k]` anywhere in the box `[-ε, ε]`, step
after step: the limit of the reachable set (the sum of `C Aᵏ Bw` boxes,
until `Aᵏ Bw` has decayed to `tol` of `Bw`).
"""
function disturbance_set(A, Bw, C, ε; tol::Real=1e-9, max_steps::Integer=100_000)
    G = Matrix{Float64}[]
    X = Bw * Diagonal(collect(Float64, ε))
    first = sum(abs, X)
    for _ in 1:max_steps
        push!(G, C * X)
        X = A * X
        sum(abs, X) < tol * first && break
    end
    return Zonotope(zeros(size(C, 1)), reduce(hcat, G))
end
