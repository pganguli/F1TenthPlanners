# Variance-based (Sobol) sensitivity indices, from Saltelli's sampling
# scheme: two independent samples A and B of the k factors, and for each
# factor i, A with B's column i.

"""
    saltelli_design(rng, N, k) -> (A, B)

Two independent N × k samples of k factors, each uniform on [0, 1] (scale
them to the factors' ranges). Evaluate the model on `A`, on `B`, and on
`mix(A, B, i)` for each factor `i`: N (k + 2) runs.
"""
saltelli_design(rng::AbstractRNG, N::Integer, k::Integer) = (rand(rng, N, k), rand(rng, N, k))

"""
    mix(A, B, i) -> Matrix

`A` with column `i` from `B`.
"""
mix(A::AbstractMatrix, B::AbstractMatrix, i::Integer) = (C = copy(A); C[:, i] = B[:, i]; C)

"""
    sobol_indices(yA, yB, yAB) -> (first, total, variance)

First-order and total Sobol indices of each factor, from the model's
outputs on `A`, on `B`, and on `mix(A, B, i)` (`yAB[:, i]`): Saltelli
(2010)'s estimator of the first-order index, Jansen's of the total.
`first[i]` is the share of the output's variance factor i causes alone,
`total[i]` with all its interactions; `total[i] - first[i]` is the share
of its interactions. Rows with a NaN output are left out.
"""
function sobol_indices(yA::AbstractVector, yB::AbstractVector, yAB::AbstractMatrix)
    ok = isfinite.(yA) .& isfinite.(yB) .& vec(all(isfinite, yAB; dims=2))
    a, b, ab = yA[ok], yB[ok], yAB[ok, :]
    n = length(a)
    n < 2 && return fill(NaN, size(yAB, 2)), fill(NaN, size(yAB, 2)), NaN
    all_y = vcat(a, b)
    μ = sum(all_y) / length(all_y)
    V = sum(abs2, all_y .- μ) / (length(all_y) - 1)
    V <= 0 && return zeros(size(yAB, 2)), zeros(size(yAB, 2)), V
    first = [sum(b .* (ab[:, i] .- a)) / n / V for i in axes(ab, 2)]
    total = [sum(abs2, a .- ab[:, i]) / 2n / V for i in axes(ab, 2)]
    return first, total, V
end
