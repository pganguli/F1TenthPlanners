# The sensitivity of the trajectory to each estimate's bias and delay,
# without simulation and without a map: the closed loop (F1Tenth.jl's
# single-track car, servo and VESC speed loop; the LiDAR's timing;
# LidarRacer.jl's planners and Stanley steering, with no Kalman filter and
# no age limit on late results) linearized about one situation, and the
# sets its states can reach (zonotopes). Writes results/analytic.md.
#
#     julia --project=. analytic.jl
#
# The situation: the car in steady state on a lane of constant curvature κ0
# and width W, everything about it (its center line, its walls, what the
# LiDAR can see) computed from the geometry, not from a map. Linearized
# there:
#
# - the car: vehicle_dynamics_st (Magic Formula tires), the servo's lag, the
#   VESC's speed PID and motor, in the lane's frame (lateral offset d,
#   heading against the lane ψ̃); continuous, then sampled every scan
#   (25 ms) with the command taking effect 26.25 ms after its scan (the
#   sweep and the computing latency). The command limiter's and VESC's
#   ramps, the steering limits and the tires' grip limit are not in a
#   linear model: it holds for small deviations;
# - the controller: LidarRacer.jl's `drive` (path planner, Stanley, speed
#   planner) by its exact Jacobian at the situation, fed the estimates
#   directly (no Kalman filter) and the car's measured speed and yaw rate;
# - the estimates: their true values as functions of the car's pose and the
#   road ahead (their Jacobians, from the geometry).
#
# Accuracy: one estimate gets a constant bias, anywhere in the box ± the car
# network's median error there (each entry on its own); its sensitivity is
# the size of the final reachable set of the car's lateral offset (and
# speed): every steady state those biases lead to.
#
# Latency: one estimate is n scans late (no age limit), the others on time,
# while the road's curvature varies (a first-order process, its bound and
# correlation length measured on the 23 racetracks' center lines); the
# reachable set of the difference it makes (against the same loop with the
# estimate on time, on the same road), and its growth per scan of delay; and
# the delay margin: the shortest delay at which the loop goes unstable.

const TRAIN = joinpath(@__DIR__, "..", "..", "LidarLaneNets.jl", "train")
include(joinpath(TRAIN, "tracks.jl"))   # centerline (the racetracks' statistics only), F1Tenth

using LidarRacer
using LinearAlgebra
using Printf
using Statistics
using LidarLaneNets
const SVector = F1Tenth.SVector
using StanleyController: Path, tracking_error, point_at, menger_curvature
using TrajectorySensitivity

const OUT = joinpath(@__DIR__, "results")

# --- the situation: a lane of constant curvature 0.1 1/m (radius 10 m;
# between the racetracks' 75th and 90th percentile |κ|, 0.07 and 0.19 1/m)
# and their median width. The tightest such lane tried (0.02, 0.05, 0.1,
# 0.15, 0.2 1/m) on which the linearized loop is stable: from 0.15 1/m the
# speed is set by the yaw-rate cap (a_lat / |r|) and the speed loop through
# it is unstable in small signal (in the simulator, the command limiter's
# ramps hold it to a chatter of about ±2 cm/s). Another curvature:
# KAPPA0=0.05 julia --project=. analytic.jl
const KAPPA0 = parse(Float64, get(ENV, "KAPPA0", "0.1"))
const WIDTH = 2.26
const T = 1 / 40          # scan period [s]

# where each estimate's entries are in a flat vector of all six
const SIZES = (e=1, ψe=1, κ_profile=length(PROFILE_S), widths=2length(PROFILE_S), sight=1, clearance=length(CLEARANCE_K))
const RANGES = let o = 0
    NamedTuple{ESTIMATES}(Tuple((r = o+1:o+SIZES[est]; o += SIZES[est]; r) for est in ESTIMATES))
end
const NLAB = sum(SIZES)

flatten(l) = reduce(vcat, (l[est] for est in ESTIMATES))
unflatten(v) = NamedTuple{ESTIMATES}(Tuple(collect(Float64, v[RANGES[est]]) for est in ESTIMATES))

# =====================================================================
# The lane's geometry, from its curvature: what the estimates truly are

struct Lane
    s::Vector{Float64}
    x::Vector{Float64}
    y::Vector{Float64}
    h::Vector{Float64}
    walls::NTuple{2,Matrix{Float64}}   # left, right: 2 × n polylines
    κ::Function
    path::Path
end

"""
    Lane(κ; s_lo, s_hi, ds) -> Lane

A lane of width WIDTH whose center line has curvature `κ(s)`, with `s = 0`
at the origin heading along x.
"""
function Lane(κ; s_lo=-4.0, s_hi=18.0, ds=0.01)
    s = collect(s_lo:ds:s_hi)
    i0 = round(Int, -s_lo / ds) + 1
    n = length(s)
    x, y, h = zeros(n), zeros(n), zeros(n)
    for i in i0:n-1
        h[i+1] = h[i] + ds * (κ(s[i]) + κ(s[i+1])) / 2
        x[i+1] = x[i] + ds * cos((h[i] + h[i+1]) / 2)
        y[i+1] = y[i] + ds * sin((h[i] + h[i+1]) / 2)
    end
    for i in i0:-1:2
        h[i-1] = h[i] - ds * (κ(s[i]) + κ(s[i-1])) / 2
        x[i-1] = x[i] - ds * cos((h[i] + h[i-1]) / 2)
        y[i-1] = y[i] - ds * sin((h[i] + h[i-1]) / 2)
    end
    w = WIDTH / 2
    left = vcat((x .- w .* sin.(h))', (y .+ w .* cos.(h))')
    right = vcat((x .+ w .* sin.(h))', (y .- w .* cos.(h))')
    return Lane(s, x, y, h, (left, right), κ, Path(x, y))
end

_cross(ax, ay, bx, by) = ax * by - ay * bx

# Whether segments p1p2 and q1q2 cross.
function _crosses(p1x, p1y, p2x, p2y, q1x, q1y, q2x, q2y)
    d1 = _cross(q2x - q1x, q2y - q1y, p1x - q1x, p1y - q1y)
    d2 = _cross(q2x - q1x, q2y - q1y, p2x - q1x, p2y - q1y)
    d3 = _cross(p2x - p1x, p2y - p1y, q1x - p1x, q1y - p1y)
    d4 = _cross(p2x - p1x, p2y - p1y, q2x - p1x, q2y - p1y)
    return (d1 > 0) != (d2 > 0) && (d3 > 0) != (d4 > 0)
end

# Whether the straight line from a to b crosses no wall.
function visible(L::Lane, ax, ay, bx, by)
    for W in L.walls, i in 1:size(W, 2)-1
        _crosses(ax, ay, bx, by, W[1, i], W[2, i], W[1, i+1], W[2, i+1]) && return false
    end
    return true
end

# Distance from (px, py) to the nearest wall.
function wall_distance(L::Lane, px, py)
    best = Inf
    for W in L.walls, i in 1:size(W, 2)-1
        ax, ay, bx, by = W[1, i], W[2, i], W[1, i+1], W[2, i+1]
        dx, dy = bx - ax, by - ay
        t = clamp(((px - ax) * dx + (py - ay) * dy) / (dx^2 + dy^2), 0.0, 1.0)
        best = min(best, hypot(px - ax - t * dx, py - ay - t * dy))
    end
    return best
end

center_at(L::Lane, s) = point_at(L.path, s - L.s[1])

# The last point of [lo, hi] where `ok` holds, `ok(lo)` holding: hi if it
# holds all the way (checked every `step`), else the boundary, by bisection.
function last_ok(ok, lo, hi; step=0.05, tol=1e-10)
    a = lo
    while a < hi
        b = min(a + step, hi)
        if !ok(b)
            while b - a > tol
                m = (a + b) / 2
                ok(m) ? (a = m) : (b = m)
            end
            return a
        end
        a = b
    end
    return hi
end

"""
    true_estimates(L, d, ψ, p) -> flat vector

The six estimates' true values for the car with its center of gravity `d`
to the left of the lane's center at s = 0 and its heading `ψ` from the
lane's (car parameters `p`): as LidarLaneNets.jl's truth.jl defines them,
except that sight and clearance are where the view or the arc truly ends
(not to the next sample), so that they change smoothly.
"""
function true_estimates(L::Lane, d, ψ, p)
    θ = ψ   # the lane heads along x at s = 0
    cx, cy = -d * sin(0.0), d * cos(0.0)
    fx, fy = cx + p.lf * cos(θ), cy + p.lf * sin(θ)
    lx, ly = cx + p.lidar_dist * cos(θ), cy + p.lidar_dist * sin(θ)
    err = tracking_error(L.path, fx, fy, θ)
    s0 = L.s[1] + err.s
    κp = [L.κ(s0 + q) for q in PROFILE_S]
    wl, wr = fill(WIDTH / 2, length(PROFILE_S)), fill(WIDTH / 2, length(PROFILE_S))
    ok(q) = begin
        px, py = center_at(L, s0 + q)
        hypot(px - lx, py - ly) <= p.scan_range && visible(L, lx, ly, px, py)
    end
    sight = last_ok(ok, 0.0, SIGHT_MAX)
    clear = map(CLEARANCE_K) do κ
        at(s) = abs(κ) < 1e-9 ? (fx + s * cos(θ), fy + s * sin(θ)) :
                (fx - sin(θ) / κ + sin(θ + κ * s) / κ, fy + cos(θ) / κ - cos(θ + κ * s) / κ)
        limit = abs(κ) < 1e-6 ? CLEARANCE_MAX : min(CLEARANCE_MAX, π / abs(κ))
        last_ok(s -> wall_distance(L, at(s)...) > p.hw, 0.0, limit; step=0.05)
    end
    return flatten(lane_labels(; e=err.e, ψe=err.ψe, κ_profile=κp, w_left=wl, w_right=wr, sight, clearance=clear))
end

# =====================================================================
# The car, its actuators, the controller

const VP = F1Tenth.VehicleParams()
const CAR = CarParams(VP)
const BEAMS = range(-3π / 4, 3π / 4; length=1081)
const SWEEP = scan_sweep(CAR, BEAMS)
const PLANNER = Planner(CAR; sweep=SWEEP)
const DELAY = SWEEP + LidarRacer.default_latency()   # from a scan to its command taking effect
const GEOM = (lf=VP.lf, lidar_dist=VP.lidar_x - VP.lr, scan_range=CAR.scan_range, hw=CAR.width / 2)
const KA = F1Tenth.drive_force_per_amp(VP) * VP.current_max / VP.m   # acceleration per unit of VESC output
const TAU_F = (1 / VP.vesc_rate) * (1 - VP.vesc_kd_filter) / VP.vesc_kd_filter   # the D term's filter

"""
    controller(labels, v, r) -> [steer, speed]

LidarRacer.jl's `drive` on the estimates as they are (no Kalman filter:
zero standard deviations).
"""
function controller(lab::AbstractVector, v, r)
    steer, speed, _ = drive(PLANNER, LaneState(unflatten(lab)), v, r)
    return [steer, speed]
end

# The continuous dynamics: x = (d, ψ̃, v, r, β, δ, i, ξ), u = (steer, speed)
# commands, w = the road's curvature at the car less κ0.
function plant(x, u, w)
    d, ψ, v, r, β, δ, i, ξ = x
    κ = KAPPA0 + w
    err = u[2] - v * cos(β)                                   # the VESC's speed error, in m/s
    out = VP.vesc_kp / 20 * VP.erpm_gain * err + i + VP.vesc_kd / 20 * VP.erpm_gain / TAU_F * (err - ξ)
    sv = (u[1] - δ) / VP.tau_steer                            # the servo
    f = F1Tenth.vehicle_dynamics_st(SVector(0.0, 0.0, δ, v, 0.0, r, β), SVector(sv, KA * out), VP)
    return [v * sin(ψ + β), r - κ * v * cos(ψ + β) / (1 - κ * d), f[4], f[6], f[7], f[3],
        VP.vesc_ki / 20 * VP.erpm_gain * err, (err - ξ) / TAU_F]
end

"""
    equilibrium() -> NamedTuple

The steady state on the nominal lane: the car's offset, slip, steering,
drive, speed (Newton's method on the body's steady state, the controller's
steering and speed commands), and the estimates' true values there.
"""
function equilibrium()
    L = Lane(_ -> KAPPA0)
    function residual(y)
        d, β, δ, a, v = y
        r = KAPPA0 * v / (1 - KAPPA0 * d)
        f = F1Tenth.vehicle_dynamics_st(SVector(0.0, 0.0, δ, v, 0.0, r, β), SVector(0.0, a), VP)
        u = controller(true_estimates(L, d, -β, GEOM), v, r)
        return [f[4], f[6], f[7], u[1] - δ, u[2] - v * cos(β)]
    end
    y = [0.0, 0.0, atan(KAPPA0 * (VP.lf + VP.lr)), 0.1, sqrt(PLANNER.speed.a_lat / KAPPA0)]
    for it in 1:40
        R = residual(y)
        norm(R) < 1e-11 && break
        y = y .- TrajectorySensitivity.jacobian(residual, y; h=1e-7) \ R
        it == 40 && @warn "equilibrium: residual $(norm(R))"
    end
    d, β, δ, a, v = y
    r = KAPPA0 * v / (1 - KAPPA0 * d)
    lab = true_estimates(L, d, -β, GEOM)
    x = [d, -β, v, r, β, δ, a / KA, 0.0]
    return (; x, u=[δ, v * cos(β)], lab, d, β, δ, a, v, r, vesc_out=a / KA)
end

# =====================================================================
# The road: a first-order process of curvature along the lane, its bound
# and correlation length from the racetracks

"""
    road_statistics() -> (bound, length)

The racetracks' center-line curvature (every 0.25 m, through points 1 m
apart, as truth.jl measures it): its 95th percentile |κ| (the bound), and
the distance over which it decorrelates (an exponential fitted to its
autocorrelation up to 10 m).
"""
function road_statistics()
    ks = Vector{Float64}[]
    for name in sort(collect(keys(F1Tenth.RACETRACKS)))
        c = centerline(name)
        len = c.s[end] + c.seg_length[end]
        push!(ks, [menger_curvature(point_at(c, mod(s - 0.5, len)), point_at(c, s), point_at(c, mod(s + 0.5, len)))
                   for s in 0.0:0.25:len-0.25])
    end
    bound = quantile(abs.(reduce(vcat, ks)), 0.95)
    lags = 1:40
    ρ = map(lags) do k
        num = sum(sum((x[1:end-k] .- mean(x)) .* (x[1+k:end] .- mean(x))) for x in ks)
        den = sum(sum(abs2, x .- mean(x)) for x in ks)
        num / den
    end
    ℓs = range(0.2, 20.0; length=2000)
    ℓ = ℓs[argmin([sum(abs2, ρ .- exp.(-0.25 .* lags ./ l)) for l in ℓs])]
    return bound, ℓ
end

# =====================================================================
# The linear closed loop

"""
    linearize(eq) -> NamedTuple

Everything linear about the equilibrium `eq`: the car sampled every scan
(Φ, Γold, Γnew, Γw, with `steps` whole scans of command delay), the
controller's Jacobians (KL: the estimates; Kv, Kr: speed, yaw rate), the
estimates' Jacobians (Hx: offset and heading; Hk: the road's curvature at
the knots, Δ apart, `k0` the car's).
"""
function linearize(eq)
    x0, u0 = eq.x, eq.u
    @assert norm(plant(x0, u0, 0.0)) < 1e-6 "not an equilibrium: $(plant(x0, u0, 0.0))"
    A = TrajectorySensitivity.jacobian(x -> plant(x, u0, 0.0), x0; h=1e-7)
    B = TrajectorySensitivity.jacobian(u -> plant(x0, u, 0.0), u0; h=1e-7)
    Bw = TrajectorySensitivity.jacobian(w -> plant(x0, u0, w[1]), [0.0]; h=1e-7)
    Φ, Γold, Γnew, Γw, steps = discretize(A, B, Bw, T, DELAY)
    steps == 1 || error("the analysis assumes a command delay of one to two scans, not $(steps)")
    J = TrajectorySensitivity.jacobian(z -> controller(z[1:NLAB], z[NLAB+1], z[NLAB+2]), vcat(eq.lab, eq.v, eq.r); h=1e-6)
    KL, Kv, Kr = J[:, 1:NLAB], J[:, NLAB+1], J[:, NLAB+2]
    # the estimates against the car's pose
    L0 = Lane(_ -> KAPPA0)
    Hx = TrajectorySensitivity.jacobian(q -> true_estimates(L0, q[1], q[2], GEOM), [eq.d, -eq.β]; h=1e-6)
    # ... and against the road: knots Δ apart (one scan's travel), from 3 m behind to 13 m ahead
    Δ = eq.v * T
    knots = collect(-round(Int, 3 / Δ):round(Int, 13 / Δ)) .* Δ
    hat(s, j) = max(0.0, 1 - abs(s - knots[j]) / Δ)
    road(δ) = s -> KAPPA0 + sum(δ[j] * hat(s, j) for j in eachindex(knots) if abs(s - knots[j]) < Δ; init=0.0)
    Hk = TrajectorySensitivity.jacobian(δ -> true_estimates(Lane(road(δ)), eq.d, -eq.β, GEOM), zeros(length(knots));
        h=1e-4)
    k0 = findfirst(s -> abs(s) < Δ / 2, knots)
    return (; A, B, Bw, Φ, Γold, Γnew, Γw, KL, Kv, Kr, Hx, Hk, Δ, knots, k0)
end

"""
    closed_loop(M; channel=nothing, delay=0, road=false) -> (A, C, Bbias, Broad)

The sampled closed loop: the car (8), the commands of the last two scans
(2 + 2), with `road` the road's knots, and with a `delay` (scans) on the
estimate `channel` a register of its last `delay` contributions to the
command. `C` picks the lateral offset and the speed. `Bbias`: a constant
bias on `channel`'s entries; `Broad`: the change of the road's curvature
entering the far knot.
"""
function closed_loop(M; channel=nothing, delay::Integer=0, road::Bool=false, a_road=1.0)
    nk = road ? length(M.knots) : 0
    n = 12 + nk + 2delay
    X, U1, U2 = 1:8, 9:10, 11:12
    K = 12 .+ (1:nk)
    R(j) = 12 + nk + 2(j - 1) .+ (1:2)   # register: R(1) one scan old ... R(delay) delay scans
    A = zeros(n, n)
    A[X, X] = M.Φ
    A[X, U1] = M.Γnew
    A[X, U2] = M.Γold
    road && (A[X, K[M.k0]] = M.Γw[:, 1])
    idx = channel === nothing ? Int[] : collect(RANGES[channel])
    live = setdiff(1:NLAB, delay > 0 ? idx : Int[])
    # the command now: the estimates (the late one from the register), speed and yaw rate
    A[U1, 1:2] = M.KL[:, live] * M.Hx[live, :]
    road && (A[U1, K] = M.KL[:, live] * M.Hk[live, :])
    A[U1, 3] .+= M.Kv
    A[U1, 4] .+= M.Kr
    if delay > 0
        A[U1, R(delay)] = I(2)
        A[R(1), 1:2] = M.KL[:, idx] * M.Hx[idx, :]
        road && (A[R(1), K] = M.KL[:, idx] * M.Hk[idx, :])
        for j in 2:delay
            A[R(j), R(j - 1)] = I(2)
        end
    end
    A[U2, U1] = I(2)
    if road   # the knots move one back each scan; the farthest follows the process
        for j in 1:nk-1
            A[K[j], K[j+1]] = 1.0
        end
        A[K[end], K[end]] = a_road
    end
    C = zeros(2, n)
    C[1, 1] = 1.0
    C[2, 3] = 1.0
    Bbias = zeros(n, length(idx))
    Bbias[U1, :] = M.KL[:, idx]
    Broad = zeros(n, 1)
    road && (Broad[K[end], 1] = 1.0)
    return A, C, Bbias, Broad
end

"""
    latency_set(M, channel, delay, a_road, c) -> Zonotope

Every difference in lateral offset and speed between the loop with
`channel` `delay` scans late and the loop with it on time, both on the same
road, over every road whose curvature's far end changes by at most `c` a
scan (the first-order process `a_road`): the reachable set of the deviation
the delay causes.
"""
function latency_set(M, channel, delay, a_road, c)
    A1, C1, _, B1 = closed_loop(M; channel, delay, road=true, a_road)
    A0, C0, _, B0 = closed_loop(M; road=true, a_road)
    n1, n0 = size(A1, 1), size(A0, 1)
    A = [A1 zeros(n1, n0); zeros(n0, n1) A0]
    stable(A) || return Zonotope(zeros(2), fill(Inf, 2, 1))
    return disturbance_set(A, vcat(B1, B0), hcat(C1, -C0), [c])
end

"""
    delay_margin(M, channel; max_delay=400) -> scans or nothing

The fewest scans of delay on `channel` that make the loop unstable
(`nothing`: none up to `max_delay`).
"""
function delay_margin(M, channel; max_delay=400)
    unstable(n) = !stable(closed_loop(M; channel, delay=n)[1])
    unstable(max_delay) || return nothing
    lo, hi = 0, max_delay   # stable at lo, unstable at hi
    while hi - lo > 1
        m = (lo + hi) ÷ 2
        unstable(m) ? (hi = m) : (lo = m)
    end
    return hi
end

# =====================================================================

fmt(x; d=3) = x === nothing ? "none" : isfinite(x) ? string(round(x; sigdigits=d)) : "–"

function main()
    eq = equilibrium()
    @printf("equilibrium: v %.3f m/s, d %.4f m, β %.4f rad, δ %.4f rad, r %.4f rad/s, VESC output %.3f\n",
        eq.v, eq.d, eq.β, eq.δ, eq.r, eq.vesc_out)
    M = linearize(eq)
    A0, C0, _, _ = closed_loop(M)
    stable(A0) || error("the nominal closed loop is unstable")
    ρ0 = maximum(abs, eigvals(A0))
    println("closed loop stable, spectral radius ", round(ρ0; digits=4))

    # accuracy: a bias up to the car network's median error at the situation
    models = load_errors(joinpath(OUT, "error_model.toml"); network=:small_weak)
    labs = unflatten(eq.lab)
    acc = Dict{Symbol,Any}()
    for est in ESTIMATES
        m = models[est]
        ε = m.s .* err_factor.(Ref(m), situation(est, labs))
        A, C, Bb, _ = closed_loop(M; channel=est)
        Z = bias_set(A, Bb, C, ε)
        acc[est] = (; ε, Z, w=widths(Z), per_unit=widths(bias_set(A, Bb, C, ones(length(ε)))))
        @printf("accuracy %-10s lateral %.4g m, speed %.4g m/s\n", est, acc[est].w[1], acc[est].w[2])
    end

    # latency: the road varies; one estimate late
    K, ℓ = road_statistics()
    a_road = exp(-M.Δ / ℓ)
    c = (1 - a_road) * K
    @printf("road: |κ| ≤ %.3f 1/m (95th percentile), correlation length %.2f m\n", K, ℓ)
    lat = Dict{Symbol,Any}()
    delays = 0:4
    for est in ESTIMATES
        w = map(n -> widths(latency_set(M, est, n, a_road, c)), delays)
        lat_w, spd_w = first.(w), last.(w)
        slope = sum((delays) .* (lat_w .- lat_w[1])) / sum(abs2, delays)
        margin = delay_margin(M, est)
        lat[est] = (; lat_w, spd_w, slope, margin)
        @printf("latency  %-10s %s m;  %.4g m per scan;  margin %s scans\n", est,
            join((@sprintf("%.4g", x) for x in lat_w), " / "), slope, fmt(margin))
    end

    A, C, _, Br = closed_loop(M; road=true, a_road)
    lat_base = widths(disturbance_set(A, Br, C, [c]))[1]
    write_report(eq, M, ρ0, acc, lat, K, ℓ, delays, lat_base)
end

function write_report(eq, M, ρ0, acc, lat, K, ℓ, delays, lat_base)
    rows_acc = join((@sprintf("| `%s` | %s | %s | %s |", est, fmt(acc[est].w[1]), fmt(acc[est].w[2]),
        length(acc[est].ε) == 1 ? @sprintf("± %s", fmt(acc[est].ε[1])) :
        @sprintf("± %s to %s per entry", fmt(minimum(acc[est].ε)), fmt(maximum(acc[est].ε)))) for est in ESTIMATES), "\n")
    rows_lat = join((@sprintf("| `%s` | %s | %s | %s |", est, fmt(lat[est].slope),
        join((fmt(x) for x in lat[est].lat_w), " / "),
        lat[est].margin === nothing ? "none up to 10 s" : @sprintf("%d scans (%s ms)", lat[est].margin, fmt(1000T * lat[est].margin)))
                     for est in ESTIMATES), "\n")
    write(joinpath(OUT, "analytic.md"), """
        # Sensitivity from the linearized closed loop

        No simulation and no map: the closed loop linearized about one situation, and the sets it can reach
        (zonotopes). From [`analytic.jl`](../analytic.jl), which says how each part is modeled. Unlike the
        simulation ([sensitivity.md](sensitivity.md)): no Kalman filter (the estimates go to the planners as
        they are), and no age limit on late results.

        ## The situation

        The car in steady state on a lane of constant curvature $(KAPPA0) 1/m (radius $(fmt(1 / KAPPA0)) m; the
        racetracks' center lines bend more than this between 10% and 25% of the way) and width $(WIDTH) m (their
        median), its center line, walls and what the LiDAR sees computed from the geometry. (On a tighter lane,
        from 0.15 1/m, the speed planner's yaw-rate cap sets the speed, and the speed loop through it is unstable
        in small signal: in the simulator the command limiter's ramps hold it to a chatter of about ±2 cm/s, so a
        linear model doesn't hold there.) There, the controller holds it at $(fmt(eq.v; d=4)) m/s, $(fmt(eq.d; d=3)) m
        to the left of the lane's center (toward the inside), steering $(fmt(eq.δ; d=3)) rad; the estimates' true
        values: sight $(fmt(eq.lab[RANGES.sight][1]; d=3)) m, clearance $(fmt(minimum(eq.lab[RANGES.clearance]); d=3)) to
        $(fmt(maximum(eq.lab[RANGES.clearance]); d=3)) m along the fan. Linearized there, sampled every scan (25 ms,
        commands taking effect $(fmt(1000DELAY; d=4)) ms after their scan), the closed loop is stable (spectral
        radius $(fmt(ρ0; d=4))).

        ## Accuracy

        Each estimate with a constant bias anywhere in the box ± the car network's (small_weak) median error
        in this situation, each entry on its own (from [error_model.toml](error_model.toml)); the others true.
        The sensitivity: the final reachable set, every steady state such biases lead to, by its width in the
        car's lateral offset and in its speed. The loop is linear, so a bias k times as large gives a set k times
        as wide.

        | Estimate | Lateral offset: width of the final set [m] | Speed: width [m/s] | Bias box |
        |---|---|---|---|
        $(rows_acc)

        ## Latency

        Each estimate late by whole scans, as a late result is used at the first scan after it arrives (no age
        limit), the others on time, while the road's curvature varies: a first-order process along the lane,
        within ± $(fmt(K)) 1/m (the racetracks' 95th percentile |κ|), decorrelating over $(fmt(ℓ)) m (fitted to their
        center lines' autocorrelation). The reachable set of the difference the delay makes to the lateral offset
        (against the same loop with the estimate on time, on the same road), over all such roads, by its width, at
        delays of $(join(delays, ", ")) scans; the sensitivity: its growth per scan (least squares). The delay
        margin: the shortest delay at which the loop goes unstable. (Over the same roads, the lateral offset
        itself, with every estimate on time, spans $(fmt(lat_base)) m: mostly the car settling to a different offset
        in a tighter or wider bend.)

        | Estimate | Sensitivity [m per scan of delay] | Width at $(join(delays, " / ")) scans [m] | Delay margin |
        |---|---|---|---|
        $(rows_lat)

        ## Why some are zero

        At this situation, a small change in `sight` or the `widths` changes no command: the path planner
        rebuilds the lane in $(PROFILE_STEP) m steps up to the sight (a small change crosses no step), the
        sight's braking limit on the speed is not the active one, and the planned line doesn't touch the lane's
        bounds. Their sensitivity is exactly zero here: they matter only through larger errors (crossing a step,
        reaching a bound), which a linear model can't see. `clearance` acts on the speed only (it sets the speed
        here, through the free distance ahead), and on the lateral offset only through the speed.

        ## What a linear model leaves out

        The command limiter's and the VESC's ramps, the steering limits and the tires' grip limit (it holds for
        small deviations); the speed planner's switches between its limits (only the one active here counts);
        sight and clearance at their caps (there, their derivatives are zero). Sight and clearance are where the
        view and the arc truly end, not rounded to the samples truth.jl uses, so that they change smoothly.
        """)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
