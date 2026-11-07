"""
    MinimumCurvaturePlanner

Minimum-curvature path planning (Heilmeier, Wischnewski, Hermansdorfer,
Betz, Lienkamp and Lohmann, "Minimum curvature trajectory planning and
control for an autonomous race car", Vehicle System Dynamics 58(10), 2019),
with nothing tied to a particular vehicle, sensor or track format.

Given a reference line (a track's or lane's center, say) and how far each of
its points may move sideways, it finds the path that bends least: in wide
before a corner, close to the inside at its apex, wide again after. Such a
path lets a vehicle corner faster, and turns less sharply than the reference
where the reference is tighter than the vehicle can steer.

The path is the reference shifted along its left normals, `p_i = r_i + α_i
n_i`, with `lower[i] ≤ α[i] ≤ upper[i]`. It minimizes the summed squared
curvature, linearized as Heilmeier et al. do: with the tangents held at the
reference's, `κ_i ≈ n_i ⋅ (p_{i-1} - 2p_i + p_{i+1}) / Δs²`, which is linear in
`α`. That makes a convex quadratic program with box constraints, solved here
exactly by an active-set method.

Holding the tangents fixed favors the inside of a corner (shifting a
corner's points outward spreads them apart, which the linearization counts as
more bending). On a narrow corridor that only keeps the path a little closer
to the reference; on a wide one, around long corners, it can even make the
corners tighter. `iterations > 0` then refines the path, as the iterative
variant of the method does (`mincurv_iqp` in TUM's
global_racetrajectory_optimization): the exact curvature, that of the circle
through each point and its neighbors, is linearized around the current path
and the quadratic program solved again (Gauss-Newton), each step shortened
until the summed squared curvature falls.

Works on a stretch of road (an open line, whose ends are free) or a whole lap
(a closed line). See [`min_curvature_path`](@ref).
"""
module MinimumCurvaturePlanner

using LinearAlgebra

export left_normals, min_curvature_offsets, min_curvature_path

"""
    left_normals(x, y; closed=false) -> (nx, ny)

Unit normals pointing left of the line through the points `(x[i], y[i])`,
each perpendicular to the direction from the point before to the point after
(from the point itself at the ends of an open line).
"""
function left_normals(x::AbstractVector{<:Real}, y::AbstractVector{<:Real}; closed::Bool=false)
    n = length(x)
    length(y) == n || throw(DimensionMismatch("x and y have different lengths"))
    n >= 2 || throw(ArgumentError("need at least 2 points"))
    nx, ny = zeros(n), zeros(n)
    for i in 1:n
        a, b = closed ? (mod1(i - 1, n), mod1(i + 1, n)) : (max(1, i - 1), min(n, i + 1))
        h = atan(y[b] - y[a], x[b] - x[a])
        nx[i], ny[i] = -sin(h), cos(h)
    end
    return nx, ny
end

"""
    min_curvature_offsets(x, y, lower, upper; closed=false, normals=left_normals(x, y; closed),
                          tiebreak=1e-6, iterations=0) -> α

How far to shift each point of the reference line `(x[i], y[i])` along its
left normal (`normals = (nx, ny)`) for the path that bends least, within
`lower[i] ≤ α[i] ≤ upper[i]` [m]. The reference must be feasible
(`lower[i] ≤ 0 ≤ upper[i]`) and its points about evenly spaced.

`tiebreak` weighs `Σ α²` against `Σ (κ_i Δs²)²`, only to choose among paths
that bend equally little (on a straight, every parallel path does): the one
nearest the reference. Keep it small enough not to pull a curved path in.
`iterations` is the number of refinement steps on the exact curvature (see
[`MinimumCurvaturePlanner`](@ref)); 0 gives Heilmeier et al.'s quadratic
program alone.
"""
function min_curvature_offsets(x::AbstractVector{<:Real}, y::AbstractVector{<:Real},
                               lower::AbstractVector{<:Real}, upper::AbstractVector{<:Real};
                               closed::Bool=false, normals=left_normals(x, y; closed), tiebreak::Real=1e-6,
                               iterations::Integer=0)
    n = length(x)
    all(==(n), length.((y, lower, upper, normals[1], normals[2]))) ||
        throw(DimensionMismatch("x, y, lower, upper and the normals must have the same length"))
    n >= 3 || throw(ArgumentError("need at least 3 points"))
    all(lower[i] <= 0 <= upper[i] for i in 1:n) ||
        throw(ArgumentError("the reference line must lie within the bounds (lower ≤ 0 ≤ upper)"))
    tiebreak > 0 || throw(ArgumentError("tiebreak must be positive, got $tiebreak"))
    nx, ny = normals
    # κ_k Δs² ≈ n_k ⋅ (p_{k-1} - 2p_k + p_{k+1}) = (A α + b)_k, over the points
    # with a neighbor on both sides
    ks = closed ? (1:n) : (2:n-1)
    A = zeros(length(ks), n)
    b = zeros(length(ks))
    for (row, k) in enumerate(ks), (i, w) in ((k - 1, 1.0), (k, -2.0), (k + 1, 1.0))
        i = mod1(i, n)
        A[row, i] += w * (nx[k] * nx[i] + ny[k] * ny[i])
        b[row] += w * (nx[k] * x[i] + ny[k] * y[i])
    end
    lb, ub = collect(Float64, lower), collect(Float64, upper)
    α = _box_qp(A' * A + tiebreak * I, A' * b, lb, ub)
    iterations > 0 && (α = _refine(α, x, y, nx, ny, lb, ub, closed, tiebreak, iterations))
    return α
end

# Signed curvature of the circle through the points a, b and c.
function _menger(a, b, c)
    cross = (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1])
    return 2cross / (hypot(b[1] - a[1], b[2] - a[2]) * hypot(c[1] - b[1], c[2] - b[2]) * hypot(c[1] - a[1], c[2] - a[2]))
end

# Gauss-Newton on the exact curvature κ_k (the circle through each point and
# its neighbors), scaled like the linearized one by Δs², within the same box.
function _refine(α, x, y, nx, ny, lb, ub, closed, tiebreak, iterations)
    n = length(x)
    ks = closed ? (1:n) : (2:n-1)
    Δs² = (sum(hypot(x[mod1(i + 1, n)] - x[i], y[mod1(i + 1, n)] - y[i]) for i in 1:n-1) / (n - 1))^2
    point(i, a) = (x[i] + a * nx[i], y[i] + a * ny[i])
    neighbors(k) = (mod1(k - 1, n), k, mod1(k + 1, n))
    curvature(α, k) = _menger((point(i, α[i]) for i in neighbors(k))...) * Δs²
    cost(α) = sum(curvature(α, k)^2 for k in ks) + tiebreak * sum(abs2, α)
    r = zeros(length(ks))
    J = zeros(length(ks), n)
    h = 1e-7   # step for the derivatives [m]
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
        # κ(α_new) ≈ r + J (α_new - α)
        step = _box_qp(J' * J + tiebreak * I, J' * (r - J * α), lb, ub) - α
        now, t = cost(α), 1.0
        while t >= 1 / 64 && cost(α + t * step) >= now
            t /= 2
        end
        t >= 1 / 64 || break   # no step lowers the cost: done
        α += t * step
        t * maximum(abs, step) < 1e-4 && break
    end
    return α
end

"""
    min_curvature_path(x, y, lower, upper; closed=false, normals, tiebreak) -> (px, py)

The path that bends least through the corridor around the reference line
`(x[i], y[i])`: each point moved by `α[i]` along its left normal, with
`lower[i] ≤ α[i] ≤ upper[i]`. See [`min_curvature_offsets`](@ref) for the
keywords.
"""
function min_curvature_path(x::AbstractVector{<:Real}, y::AbstractVector{<:Real},
                            lower::AbstractVector{<:Real}, upper::AbstractVector{<:Real};
                            closed::Bool=false, normals=left_normals(x, y; closed), tiebreak::Real=1e-6,
                            iterations::Integer=0)
    α = min_curvature_offsets(x, y, lower, upper; closed, normals, tiebreak, iterations)
    nx, ny = normals
    return x .+ α .* nx, y .+ α .* ny
end

# Minimizer of ½ xᵀHx + gᵀx subject to lb ≤ x ≤ ub, for H positive definite
# and lb ≤ 0 ≤ ub. Primal active-set method (Nocedal and Wright, "Numerical
# Optimization", 2006, Algorithm 16.3): fix some variables at their bounds,
# minimize over the rest, step to the first bound that gets in the way, and
# release a bound whose multiplier has the wrong sign. Each step lowers the
# cost, so it terminates; for safety it gives up after `maxiter` steps.
function _box_qp(H, g, lb, ub; maxiter=10length(g) + 10)
    n = length(g)
    x = zeros(n)
    at = zeros(Int8, n)   # 0 free, -1 held at lb, +1 held at ub
    for i in 1:n
        lb[i] == ub[i] && (x[i] = lb[i]; at[i] = -1)
    end
    for _ in 1:maxiter
        free = findall(iszero, at)
        if !isempty(free)
            fixed = findall(!iszero, at)
            rhs = -(g[free] .+ H[free, fixed] * x[fixed])
            target = cholesky(Symmetric(H[free, free])) \ rhs
            τ, block, side = 1.0, 0, Int8(0)   # the first bound in the way, and which
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
        # optimal for this set of bounds; release the bound that most wants to go
        grad = H * x .+ g
        worst, release = 0.0, 0
        for i in 1:n
            lb[i] == ub[i] && continue
            pull = at[i] == -1 ? -grad[i] : at[i] == 1 ? grad[i] : 0.0   # > 0: the bound holds x back
            pull > worst && (worst = pull; release = i)
        end
        release == 0 && return x
        at[release] = 0
    end
    return x
end

end # module
