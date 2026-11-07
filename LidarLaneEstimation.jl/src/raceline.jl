# The minimum-curvature line through the lane in view
#
# A lane center can turn more sharply than a vehicle can steer, while a line
# that uses the lane's width (in wide before a corner, close to the inside at
# its apex, out wide after) turns far less. This finds the line through the
# lane seen in one scan that bends least, as in the minimum-curvature method
# of Heilmeier et al. (2019, "Minimum curvature trajectory planning and
# control for an autonomous race car", Vehicle System Dynamics 58(10)), but
# over the stretch of lane in view instead of a whole lap:
#
# 1. Resample the lane center to points c_i, LINE_STEP apart, with unit
#    normals n_i pointing left.
# 2. The line is p_i = c_i + α_i n_i. Bound each α_i so that p_i keeps at
#    least `wall_margin` from every return in the scan.
# 3. Minimize Σ κ_i², the line's curvature at each point linearized as
#    Heilmeier et al. do: with the tangents held at the center line's,
#    κ_i ≈ n_i ⋅ (p_{i-1} - 2p_i + p_{i+1}) / LINE_STEP², linear in α. That
#    is a convex quadratic program with box constraints, solved exactly by an
#    active-set method.
#
# Holding the tangents fixed favors the inside of a corner a little (shifting
# a corner's points outward spreads them apart, which this counts as more
# bending), so the line keeps closer to the center than the exact
# minimum-curvature line. On the f1tenth_racetracks maps that suits a car
# with steering lag better: re-solving the exact curvature to convergence
# (Gauss-Newton) made the line hug its wall margin and brought the car within
# 0.1 m of the walls, and so did making the line end on the center at the
# end of the view. The line's end is left free.

const LINE_STEP = 0.2   # spacing of the line's points [m]

# Weight of Σα² against Σ (κ_i LINE_STEP²)², only to choose among lines that
# bend equally little (on a straight, every parallel line does): the one
# nearest the center. Small enough not to pull a curved line in measurably.
const LINE_TIEBREAK = 1e-6

# Points of the polyline (x, y) every `step` meters, from its start to its end.
function _resample(x, y, step)
    line = _Polyline(x, y)
    n = max(2, round(Int, line.s[end] / step) + 1)
    pts = [_point_at(line, s) for s in range(0.0, line.s[end]; length=n)]
    return first.(pts), last.(pts)
end

# Minimizer of ½ xᵀHx + gᵀx subject to lb ≤ x ≤ ub, for H positive definite
# and lb ≤ 0 ≤ ub. Primal active-set method (Nocedal and Wright, 2006,
# Algorithm 16.3): fix some variables at their bounds, minimize over the
# rest, step to the first bound that gets in the way, and release a bound
# whose multiplier has the wrong sign. Each step lowers the cost, so it
# terminates; for safety it gives up after `maxiter` steps.
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

# The minimum-curvature line through the lane around the center line (cx, cy),
# keeping `wall_margin` from every return of the scan. Where no return
# limits it, the line stays within half the lane width of the center (the
# lane continues past what the scan shows).
function _min_curvature_line(p::LanePerception, cx, cy)
    cfg = p.cfg
    x, y = _resample(cx, cy, LINE_STEP)
    n = length(x)
    n < 3 && return cx, cy
    nx, ny = similar(x), similar(y)   # unit normals, to the left
    for i in 1:n
        a, b = max(1, i - 1), min(n, i + 1)
        h = atan(y[b] - y[a], x[b] - x[a])
        nx[i], ny[i] = -sin(h), cos(h)
    end

    m = cfg.wall_margin
    cap = isnan(p.width) ? 0.0 : max(p.width / 2 - m, 0.0)
    ub, lb = fill(Inf, n), fill(-Inf, n)
    blocked = falses(n)
    @inbounds for j in 1:cfg.stride:length(p.xs)
        hypot(p.xs[j], p.ys[j]) > cfg.r_max && continue
        for i in 1:n
            dx, dy = p.xs[j] - x[i], p.ys[j] - y[i]
            t = dx * nx[i] + dy * ny[i]          # along the normal
            off = abs(dx * ny[i] - dy * nx[i])   # across it
            off >= m && continue
            s = sqrt(m^2 - off^2)
            # the return rules out α in (t - s, t + s)
            if t - s >= 0
                ub[i] = min(ub[i], t - s)
            elseif t + s <= 0
                lb[i] = max(lb[i], t + s)
            else
                blocked[i] = true                # the center itself is too close
            end
        end
    end
    for i in 1:n
        ub[i] = blocked[i] ? 0.0 : isinf(ub[i]) ? cap : ub[i]
        lb[i] = blocked[i] ? 0.0 : isinf(lb[i]) ? -cap : lb[i]
    end

    # κ_k LINE_STEP² ≈ n_k ⋅ (p_{k-1} - 2p_k + p_{k+1}) = (A α + b)_k, k = 2..n-1
    A = zeros(n - 2, n)
    b = zeros(n - 2)
    for k in 2:n-1, (i, w) in ((k - 1, 1.0), (k, -2.0), (k + 1, 1.0))
        A[k-1, i] = w * (nx[k] * nx[i] + ny[k] * ny[i])
        b[k-1] += w * (nx[k] * x[i] + ny[k] * y[i])
    end
    α = _box_qp(A' * A + LINE_TIEBREAK * I, A' * b, lb, ub)
    return x .+ α .* nx, y .+ α .* ny
end
