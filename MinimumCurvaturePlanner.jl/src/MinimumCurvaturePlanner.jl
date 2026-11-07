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
exactly by an active-set method. Each κ_i involves only three neighboring
points, so the program's matrix is zero beyond two places from its diagonal
(and, on a closed line, outside its corners): each step of the method costs
time linear in the number of points.

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
    # with a neighbor on both sides. Row k of A has nonzeros only in columns
    # k-1, k and k+1: C holds them, in that order.
    ks = closed ? (1:n) : (2:n-1)
    C = zeros(length(ks), 3)
    b = zeros(length(ks))
    for (row, k) in enumerate(ks), (p, w) in enumerate((1.0, -2.0, 1.0))
        i = mod1(k + p - 2, n)
        C[row, p] = w * (nx[k] * nx[i] + ny[k] * ny[i])
        b[row] += w * (nx[k] * x[i] + ny[k] * y[i])
    end
    lb, ub = collect(Float64, lower), collect(Float64, upper)
    α = _box_qp(_normal_equations(ks, C, b, n, closed, tiebreak)..., lb, ub)
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
    J = zeros(length(ks), 3)   # κ_k's derivatives by α[k-1], α[k] and α[k+1]
    v = zeros(length(ks))
    h = 1e-7   # step for the derivatives [m]
    for _ in 1:iterations
        for (row, k) in enumerate(ks)
            r = curvature(α, k)
            v[row] = r
            for (p, i) in enumerate(neighbors(k))
                α[i] += h
                J[row, p] = (curvature(α, k) - r) / h
                α[i] -= h
                v[row] -= J[row, p] * α[i]
            end
        end
        # κ(α_new) ≈ κ(α) + J (α_new - α) = J α_new + v
        step = _box_qp(_normal_equations(ks, J, v, n, closed, tiebreak)..., lb, ub) - α
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
# H is a dense matrix or a `_Penta`; only the linear algebra differs.
function _box_qp(H, g, lb, ub; maxiter=10length(g) + 10)
    n = length(g)
    x = zeros(n)
    at = zeros(Int8, n)   # 0 free, -1 held at lb, +1 held at ub
    for i in 1:n
        lb[i] == ub[i] && (x[i] = lb[i]; at[i] = -1)
    end
    ws = _Workspace(n)
    free = Vector{Int}(undef, n)
    grad = zeros(n)
    for _ in 1:maxiter
        m = 0
        for i in 1:n
            iszero(at[i]) && (m += 1; free[m] = i)
        end
        if m > 0
            f = view(free, 1:m)
            target = _solve_free!(ws, H, f, at, x, g)
            τ, block, side = 1.0, 0, Int8(0)   # the first bound in the way, and which
            for (k, i) in enumerate(f)
                d = target[k] - x[i]
                if d > 0 && x[i] + d > ub[i]
                    τi = (ub[i] - x[i]) / d
                    τi < τ && ((τ, block, side) = (τi, i, Int8(1)))
                elseif d < 0 && x[i] + d < lb[i]
                    τi = (lb[i] - x[i]) / d
                    τi < τ && ((τ, block, side) = (τi, i, Int8(-1)))
                end
            end
            for (k, i) in enumerate(f)
                x[i] += τ * (target[k] - x[i])
            end
            if block > 0
                x[block] = side == 1 ? ub[block] : lb[block]
                at[block] = side
                continue
            end
        end
        # optimal for this set of bounds; release the bound that most wants to go
        _gradient!(grad, H, x, g)
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

# A symmetric n×n matrix whose nonzeros lie within 2 of the diagonal,
# counting cyclically when `closed` (a closed line's ends are neighbors, which
# puts entries in the corners): d0[i] = H[i, i], d1[i] = H[i, i+1] and
# d2[i] = H[i, i+2], indices mod n when closed. A closed one needs n ≥ 5, so
# that the offsets -2:2 between two indices are told apart.
struct _Penta
    d0::Vector{Float64}
    d1::Vector{Float64}
    d2::Vector{Float64}
    closed::Bool
end

# H[i, j]; 0 unless i and j are within 2 of each other.
function _entry(H::_Penta, i, j)
    δ = j - i
    H.closed && (δ = mod(δ + 2, length(H.d0)) - 2)
    return δ == 0 ? H.d0[i] : δ == 1 ? H.d1[i] : δ == -1 ? H.d1[j] :
           δ == 2 ? H.d2[i] : δ == -2 ? H.d2[j] : 0.0
end

# The index δ away from i, or 0 past an open line's ends.
_neighbor(H::_Penta, i, δ) = H.closed ? mod1(i + δ, length(H.d0)) : (1 <= i + δ <= length(H.d0) ? i + δ : 0)

# H = CᵀC + tiebreak·I and g = Cᵀv, for the matrix C whose row for point k
# has the nonzeros C[row, :] in columns k-1, k and k+1 (mod n when closed).
# H is a `_Penta`, or dense for a closed line too short for one.
function _normal_equations(ks, C, v, n, closed, tiebreak)
    g = zeros(n)
    if closed && n < 5
        H = zeros(n, n)
        for (row, k) in enumerate(ks), p in 1:3
            i = mod1(k + p - 2, n)
            g[i] += C[row, p] * v[row]
            for q in 1:3
                H[i, mod1(k + q - 2, n)] += C[row, p] * C[row, q]
            end
        end
        return H + tiebreak * I, g
    end
    d0, d1, d2 = zeros(n), zeros(n), zeros(n)
    for (row, k) in enumerate(ks)
        a, c = mod1(k - 1, n), mod1(k + 1, n)
        c1, c2, c3 = C[row, 1], C[row, 2], C[row, 3]
        d0[a] += c1 * c1; d0[k] += c2 * c2; d0[c] += c3 * c3
        d1[a] += c1 * c2; d1[k] += c2 * c3; d2[a] += c1 * c3
        g[a] += c1 * v[row]; g[k] += c2 * v[row]; g[c] += c3 * v[row]
    end
    d0 .+= tiebreak
    return _Penta(d0, d1, d2, closed), g
end

# Scratch for `_solve_free!`, reused across the active-set steps.
struct _Workspace
    e0::Vector{Float64}   # the band of the free variables' matrix, then its Cholesky factor
    e1::Vector{Float64}
    e2::Vector{Float64}
    rhs::Vector{Float64}  # the right-hand side, then the solution
    c1::Vector{Float64}   # closed lines: the coupling to the last two free variables,
    c2::Vector{Float64}
    w1::Vector{Float64}   # and the band's inverse times it
    w2::Vector{Float64}
end
_Workspace(n) = _Workspace((zeros(n) for _ in 1:8)...)

_gradient!(grad, H::AbstractMatrix, x, g) = (grad .= H * x .+ g)

function _gradient!(grad, H::_Penta, x, g)
    for i in eachindex(x)
        s = H.d0[i] * x[i]
        for δ in (-2, -1, 1, 2)
            j = _neighbor(H, i, δ)
            j > 0 && (s += _entry(H, i, j) * x[j])
        end
        grad[i] = s + g[i]
    end
    return grad
end

# The minimizer over the variables `free`, with the others held at x.
function _solve_free!(ws, H::AbstractMatrix, free, at, x, g)
    fixed = findall(!iszero, at)
    rhs = -(g[free] .+ H[free, fixed] * x[fixed])
    return cholesky(Symmetric(H[free, free])) \ rhs
end

# Dropping the fixed variables keeps the rest banded, in their own order:
# consecutive free variables are no further apart than in the original. A
# closed line's corners couple its first two free variables to its last two;
# those last two are eliminated by their 2×2 Schur complement (block
# elimination), which leaves a banded solve with three right-hand sides.
function _solve_free!(ws, H::_Penta, free, at, x, g)
    m = length(free)
    rhs = view(ws.rhs, 1:m)
    for (a, i) in enumerate(free)
        s = 0.0
        for δ in (-2, -1, 1, 2)
            j = _neighbor(H, i, δ)
            j > 0 && !iszero(at[j]) && (s += _entry(H, i, j) * x[j])
        end
        rhs[a] = -(g[i] + s)
    end
    if !H.closed
        _band_cholesky!(ws, H, free, m)
        _band_solve!(rhs, ws, m)
    elseif m <= 4   # too few for the band and the corners to be apart
        rhs .= cholesky(Symmetric([_entry(H, i, j) for i in free, j in free])) \ rhs
    else
        p = m - 2   # [B C; Cᵀ D] [y; z] = [r; t], with D the last two's 2×2 block
        _band_cholesky!(ws, H, free, p)
        q1, q2 = free[m-1], free[m]
        for a in 1:p
            ws.w1[a] = ws.c1[a] = _entry(H, free[a], q1)
            ws.w2[a] = ws.c2[a] = _entry(H, free[a], q2)
        end
        y = view(rhs, 1:p)
        _band_solve!(y, ws, p)        # B⁻¹ r
        _band_solve!(ws.w1, ws, p)    # B⁻¹ C
        _band_solve!(ws.w2, ws, p)
        # S z = t - Cᵀ B⁻¹ r, with S = D - Cᵀ B⁻¹ C positive definite
        s11, s12, s22 = _entry(H, q1, q1), _entry(H, q1, q2), _entry(H, q2, q2)
        t1, t2 = rhs[m-1], rhs[m]
        for a in 1:p
            s11 -= ws.c1[a] * ws.w1[a]
            s12 -= ws.c1[a] * ws.w2[a]
            s22 -= ws.c2[a] * ws.w2[a]
            t1 -= ws.c1[a] * y[a]
            t2 -= ws.c2[a] * y[a]
        end
        z1, z2 = cholesky(Symmetric([s11 s12; s12 s22])) \ [t1, t2]
        for a in 1:p
            y[a] -= ws.w1[a] * z1 + ws.w2[a] * z2
        end
        rhs[m-1], rhs[m] = z1, z2
    end
    return rhs
end

# Cholesky factor L of the first m free variables' matrix, a pentadiagonal
# one, into ws: L[a, a] = e0[a], L[a+1, a] = e1[a], L[a+2, a] = e2[a].
function _band_cholesky!(ws, H, free, m)
    e0, e1, e2 = ws.e0, ws.e1, ws.e2
    for a in 1:m
        e0[a] = _entry(H, free[a], free[a])
        e1[a] = a + 1 <= m ? _entry(H, free[a], free[a+1]) : 0.0
        e2[a] = a + 2 <= m ? _entry(H, free[a], free[a+2]) : 0.0
    end
    for a in 1:m
        d = e0[a]
        if a > 2
            e2[a-2] /= e0[a-2]
            d -= e2[a-2]^2
        end
        if a > 1
            a > 2 && (e1[a-1] -= e2[a-2] * e1[a-2])
            e1[a-1] /= e0[a-1]
            d -= e1[a-1]^2
        end
        d > 0 || throw(PosDefException(a))
        e0[a] = sqrt(d)
    end
    return ws
end

# b ← (L Lᵀ)⁻¹ b, with L from `_band_cholesky!`, for b's first m entries.
function _band_solve!(b, ws, m)
    e0, e1, e2 = ws.e0, ws.e1, ws.e2
    for a in 1:m
        a > 1 && (b[a] -= e1[a-1] * b[a-1])
        a > 2 && (b[a] -= e2[a-2] * b[a-2])
        b[a] /= e0[a]
    end
    for a in m:-1:1
        a < m && (b[a] -= e1[a] * b[a+1])
        a < m - 1 && (b[a] -= e2[a] * b[a+2])
        b[a] /= e0[a]
    end
    return b
end

end # module
