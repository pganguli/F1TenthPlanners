# The planner as it was before its linear algebra used the matrices' band
# structure: dense matrices throughout. The tests check the banded one
# against it.
module DenseReference

using LinearAlgebra
using MinimumCurvaturePlanner: left_normals, _menger

function dense_offsets(x, y, lower, upper; closed=false, normals=left_normals(x, y; closed), tiebreak=1e-6,
                       iterations=0)
    n = length(x)
    nx, ny = normals
    ks = closed ? (1:n) : (2:n-1)
    A = zeros(length(ks), n)
    b = zeros(length(ks))
    for (row, k) in enumerate(ks), (i, w) in ((k - 1, 1.0), (k, -2.0), (k + 1, 1.0))
        i = mod1(i, n)
        A[row, i] += w * (nx[k] * nx[i] + ny[k] * ny[i])
        b[row] += w * (nx[k] * x[i] + ny[k] * y[i])
    end
    lb, ub = collect(Float64, lower), collect(Float64, upper)
    α = dense_box_qp(A' * A + tiebreak * I, A' * b, lb, ub)
    iterations > 0 && (α = dense_refine(α, x, y, nx, ny, lb, ub, closed, tiebreak, iterations))
    return α
end

function dense_refine(α, x, y, nx, ny, lb, ub, closed, tiebreak, iterations)
    n = length(x)
    ks = closed ? (1:n) : (2:n-1)
    Δs² = (sum(hypot(x[mod1(i + 1, n)] - x[i], y[mod1(i + 1, n)] - y[i]) for i in 1:n-1) / (n - 1))^2
    point(i, a) = (x[i] + a * nx[i], y[i] + a * ny[i])
    neighbors(k) = (mod1(k - 1, n), k, mod1(k + 1, n))
    curvature(α, k) = _menger((point(i, α[i]) for i in neighbors(k))...) * Δs²
    cost(α) = sum(curvature(α, k)^2 for k in ks) + tiebreak * sum(abs2, α)
    r = zeros(length(ks))
    J = zeros(length(ks), n)
    h = 1e-7
    for _ in 1:iterations
        fill!(J, 0.0)
        for (row, k) in enumerate(ks)
            r[row] = curvature(α, k)
            for i in neighbors(k)
                α[i] += h
                J[row, i] = (curvature(α, k) - r[row]) / h
                α[i] -= h
            end
        end
        step = dense_box_qp(J' * J + tiebreak * I, J' * (r - J * α), lb, ub) - α
        now, t = cost(α), 1.0
        while t >= 1 / 64 && cost(α + t * step) >= now
            t /= 2
        end
        t >= 1 / 64 || break
        α += t * step
        t * maximum(abs, step) < 1e-4 && break
    end
    return α
end

function dense_box_qp(H, g, lb, ub; maxiter=10length(g) + 10)
    n = length(g)
    x = zeros(n)
    at = zeros(Int8, n)
    for i in 1:n
        lb[i] == ub[i] && (x[i] = lb[i]; at[i] = -1)
    end
    for _ in 1:maxiter
        free = findall(iszero, at)
        if !isempty(free)
            fixed = findall(!iszero, at)
            rhs = -(g[free] .+ H[free, fixed] * x[fixed])
            target = cholesky(Symmetric(H[free, free])) \ rhs
            τ, block, side = 1.0, 0, Int8(0)
            for (k, i) in enumerate(free)
                d = target[k] - x[i]
                if d > 0 && x[i] + d > ub[i]
                    τi = (ub[i] - x[i]) / d
                    τi < τ && ((τ, block, side) = (τi, i, Int8(1)))
                elseif d < 0 && x[i] + d < lb[i]
                    τi = (lb[i] - x[i]) / d
                    τi < τ && ((τ, block, side) = (τi, i, Int8(-1)))
                end
            end
            for (k, i) in enumerate(free)
                x[i] += τ * (target[k] - x[i])
            end
            if block > 0
                x[block] = side == 1 ? ub[block] : lb[block]
                at[block] = side
                continue
            end
        end
        grad = H * x .+ g
        worst, release = 0.0, 0
        for i in 1:n
            lb[i] == ub[i] && continue
            pull = at[i] == -1 ? -grad[i] : at[i] == 1 ? grad[i] : 0.0
            pull > worst && (worst = pull; release = i)
        end
        release == 0 && return x
        at[release] = 0
    end
    return x
end

# The quadratic program's matrices, for its cost ½ αᵀHα + gᵀα.
function dense_qp(x, y; closed=false, normals=left_normals(x, y; closed), tiebreak=1e-6)
    n = length(x)
    nx, ny = normals
    ks = closed ? (1:n) : (2:n-1)
    A = zeros(length(ks), n)
    b = zeros(length(ks))
    for (row, k) in enumerate(ks), (i, w) in ((k - 1, 1.0), (k, -2.0), (k + 1, 1.0))
        i = mod1(i, n)
        A[row, i] += w * (nx[k] * nx[i] + ny[k] * ny[i])
        b[row] += w * (nx[k] * x[i] + ny[k] * y[i])
    end
    return A' * A + tiebreak * I, A' * b, A, b
end

end # module
