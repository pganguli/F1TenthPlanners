# The same sensitivities as the simulation campaigns measure (the RMS
# lateral deviation from the reference run with the car network's errors on
# one estimate; its growth per 25 ms the estimate comes late from the
# cloud), without simulation and without a map: the linearized closed loop
# of analytic.jl, now with LidarLaneNets.jl's Kalman filter in it, driven by
# random roads and random network errors, and its stationary covariance
# (the discrete Lyapunov equation) instead of worst-case sets. Writes
# results/matched.md, against the simulation's values (results/empirical.toml).
#
#     julia --project=. matched.jl
#
# Every situation the racetracks have (a lane of constant curvature and
# width, the car in steady state on it; or, within 10 m, a straight with a
# bend ahead or a bend with a straight ahead, the scene held as it is about
# the car; in tight bends, the speed command through the command limiter's
# ramp's describing function), side by side two loops on the same
# random road: the reference (every estimate true) and the experiment; the
# sensitivity is the RMS of the difference of their lateral offsets (its
# mean squared, plus its variance), and the grid's values are weighted by
# how much of the 23 racetracks' length is like each situation.
#
# The filter, as LidarLaneNets.jl's LaneTracker works:
#
# - e, ψe and the curvature beside the car: a Kalman filter with the
#   tracker's motion model and noise (TrackerConfig's defaults; the error
#   model's EstimateNoise for the car network: its persisting bias, its
#   white share), at its steady-state gain (the Riccati equation). The
#   motion model is not the car's: the mismatch (slip, the offset path)
#   enters as a constant;
# - every other entry: the latest measurement as it is (without the cloud
#   there is no bias to take off); sight and clearance taken k_σ σ short of
#   it by the planner, σ the tracker's (the network's white share and its
#   bias's);
# - a late cloud result (true, a delay of n scans): counted at its scan,
#   then carried to now. For e and ψe by the motion model through the
#   on-time estimates (the late value plus the model's predicted change, so
#   the model's mismatch accumulates); for the curvature beside the car not
#   at all (the model holds it constant); the profiles shifted by the
#   distance driven (exact on the ground, but their far end is lost: the
#   planned line gets shorter), sight and clearance as they were, their σ
#   grown with the distance driven since (TrackerConfig's q_sight,
#   q_clearance).
#
# The network errors, from the error library (scripts/error_library.jl on
# the simulation branch): at each situation, the car network's recorded
# errors on the scans whose true lane is most like it (mirrored where it
# bends the other way): their mean (a constant), their covariance across
# the estimate's entries and their correlation from one scan to the next
# (a first-order process), how they change with the entry's true value (a
# change of gain), and the network's own σ (what the filter is told).
#
# What a linear model leaves out: crashes, the controller's steps beyond
# the ones computed exactly here (the shorter planned line, σ's offsets),
# the speed planner's switches between its limits, the robust (Student t)
# updates, the tracker's restarts.

include(joinpath(@__DIR__, "analytic.jl"))
using JLD2
using TOML

const LIBRARY = get(ENV, "ERROR_LIBRARY", joinpath(OUT, "error_library.jld2"))
const NEAR = 3000          # library scans per situation
const SURE = 0.01          # a true estimate's σ, against its network's (as the oracle's)
const TRACKER = TrackerConfig()
const REF = CAR.lf + CAR.lr        # the tracker's reference: the front axle, ahead of the rear
const LANE_IDX = [RANGES.e[1], RANGES.ψe[1], RANGES.κ_profile[1]]   # e, ψe, κ in the flat vector
const LANE_COMP = (e=1, ψe=2, κ_profile=3)
const DELAYS = 0:4
# the measured entries' errors through the planner exactly (statistical
# linearization; `command_noise`), or by its derivative: STATISTICAL=0
const STATISTICAL = get(ENV, "STATISTICAL", "1") != "0"

# =====================================================================
# The error library, by situation

const FEATURE_S = (0.0, 1.0, 2.0, 3.0, 5.0)
_at(s) = round(Int, s / PROFILE_STEP) + 1

function features(lab::AbstractVector)
    l = unflatten(lab)
    np = length(PROFILE_S)
    return [l.e[1], l.ψe[1], (l.κ_profile[_at(s)] for s in FEATURE_S)..., l.widths[_at(0.0)], l.widths[_at(2.0)],
        l.widths[np+_at(0.0)], l.widths[np+_at(2.0)], l.sight[1]]
end

mirror_features(f) = [-f[1]; -f[2]; -f[3:7]; f[10:11]; f[8:9]; f[12]]

function mirror(est::Symbol, v::AbstractVector; sign::Bool)
    est === :widths && return [v[length(PROFILE_S)+1:end]; v[1:length(PROFILE_S)]]
    est === :clearance && return reverse(v)
    est in (:e, :ψe, :κ_profile) && sign && return -v
    return v
end

struct Library
    F::Matrix{Float64}        # features × scans, scaled
    M::Matrix{Float64}        # mirrored
    scale::Vector{Float64}
    follows::BitVector
    truth::Dict{Symbol,Matrix{Float32}}
    err::Dict{Symbol,Matrix{Float32}}     # the car's (small_weak)
    σ::Dict{Symbol,Matrix{Float32}}
end

function load_library(path)
    d = JLD2.load(path)
    f = d["features"]
    scale = [max(std(r), 1e-6) for r in eachrow(f)]
    M = reduce(hcat, (mirror_features(c) for c in eachcol(f)))
    err = Dict(Symbol(e) => v for ((e, r), v) in d["err"] if r == "car")
    σ = Dict(Symbol(e) => v for ((e, r), v) in d["sigma"] if r == "car")
    truth = Dict(Symbol(e) => v for (e, v) in d["truth"])
    return Library(f ./ scale, M ./ scale, scale, BitVector(d["follows"]), truth, err, σ)
end

"""
    error_stats(lib, lab, est) -> NamedTuple

The car network's errors on `est` at the situation whose true estimates are
`lab`, from the `NEAR` recorded scans most like it: mean, covariance across
entries, correlation from one scan to the next (one number, pooled over the
entries), slope against each entry's true value, and the network's mean σ².
"""
function error_stats(lib::Library, lab, est)
    f = features(lab) ./ lib.scale
    n = size(lib.F, 2)
    d1 = [sum(abs2, view(lib.F, :, k) .- f) for k in 1:n]
    d2 = [sum(abs2, view(lib.M, :, k) .- f) for k in 1:n]
    near = partialsortperm(vcat(d1, d2), 1:NEAR)
    pick(k) = k <= n ? (k, false) : (k - n, true)
    E = lib.err[est]
    col(X, k, flip; sign=true) = (v = Float64.(X[:, k]); flip ? mirror(est, v; sign) : v)
    errs = [col(E, pick(k)...) for k in near]
    tru = [col(lib.truth[est], pick(k)...) for k in near]
    σ2 = [col(lib.σ[est], pick(k)...; sign=false) .^ 2 for k in near]
    Y, X = reduce(hcat, errs), reduce(hcat, tru)
    μ = vec(mean(Y; dims=2))
    Σ = cov(Y; dims=2)
    # one scan apart, within a drive
    num, den = 0.0, 0.0
    for k in near
        j, flip = pick(k)
        (j < n && lib.follows[j]) || continue
        a, b = col(E, j, flip), col(E, j + 1, flip)
        num += sum((a .- μ) .* (b .- μ))
        den += sum(abs2, a .- μ)
    end
    ρ = den > 0 ? clamp(num / den, 0.0, 0.995) : 0.0
    slope = map(axes(Y, 1)) do j
        x, y = X[j, :], Y[j, :]
        v = var(x)
        v > 1e-12 ? clamp(cov(x, y) / v, -1.0, 1.0) : 0.0
    end
    return (; μ, Σ=(Σ + Σ') / 2, ρ, slope, σ2=vec(mean(reduce(hcat, σ2); dims=2)), scans=[pick(k) for k in near])
end

"""
    command_noise(eq, lib, models, est, scans; samples=600) -> NamedTuple

The planner's own response to the recorded errors on `est`'s measured
entries (not the filtered one: e, ψe, κ beside the car), each recorded scan's
error and σ put into the steady state's lane and the commands computed
exactly (the controller's steps, saturations and k_σ σ offsets included):
the mean change of the commands, their covariance, and their correlation
from one scan to the next (from the scans that follow a used one in the
same drive). Statistical linearization: what the loop then sees of these
errors is a noise on its commands with these statistics.
"""
function command_noise(eq, lib::Library, models, est, scans; samples::Integer=600)
    idx = collect(RANGES[est])
    lane = haskey(LANE_COMP, est) ? LANE_COMP[est] : 0
    use = [i for (i, j) in enumerate(idx) if lane == 0 || j != LANE_IDX[lane]]
    f = models[est].filter
    σ_tracker(σ) = sqrt.(max.((1 - f.persist) .* σ .^ 2, 0.01 .* σ .^ 2) .+ f.σ_bias^2)
    base = drive(PLANNER, lane_state(eq.lab), eq.v, eq.r)
    n = size(lib.F, 2)
    function du(j, flip)
        e = Float64.(lib.err[est][:, j])
        σ = Float64.(lib.σ[est][:, j])
        flip && (e = mirror(est, e; sign=true); σ = mirror(est, σ; sign=false))
        lab = copy(eq.lab)
        lab[idx[use]] .+= e[use]
        st = σ_tracker(σ)
        ls = est === :sight ? lane_state(lab; σ_sight=st[1]) : est === :clearance ? lane_state(lab; σ_clear=st) :
             lane_state(lab)
        u = drive(PLANNER, ls, eq.v, eq.r)
        return [u[1] - base[1], u[2] - base[2]]
    end
    pairs_ = [(j, flip) for (j, flip) in scans if j < n && lib.follows[j]]
    pairs_ = pairs_[1:min(samples, length(pairs_))]
    a = reduce(hcat, (du(j, flip) for (j, flip) in pairs_))
    b = reduce(hcat, (du(j + 1, flip) for (j, flip) in pairs_))
    m = vec(mean(a; dims=2))
    S = cov(a; dims=2)
    v = sum(abs2, a .- m)
    ρ = v > 0 ? clamp(sum((a .- m) .* (b .- m)) / v, 0.0, 0.995) : 0.0
    return (; m, S=(S + S') / 2, ρ)
end

# =====================================================================
# The tracker, linearized

# The motion model of e, ψe, κ (Euler, one scan), at the true estimates
# ŷ0 and the measured speed and yaw rate: ŷ' = ŷ + T f(ŷ, v, r).
motion(y, v, r) = [v * sin(y[2]) - REF * r * cos(y[2]), y[3] * v - r, 0.0]

function tracker_model(eq)
    y0 = eq.lab[LANE_IDX]
    Jy = TrajectorySensitivity.jacobian(y -> motion(y, eq.v, eq.r), y0; h=1e-7)
    Ju = TrajectorySensitivity.jacobian(u -> motion(y0, u[1], u[2]), [eq.v, eq.r]; h=1e-7)
    # what the model predicts per scan where truly nothing changes: its mismatch
    return (F3=I + T .* Jy, G3=T .* Ju, c3=T .* motion(y0, eq.v, eq.r), Jy, Ju)
end

"""
    lane_filter(eq, models, erring, σ2) -> (K, F6, G6, c6)

The tracker's filter on e, ψe, κ and the car network's biases on them, at
its steady-state gain, when `erring` (1, 2, 3 or 0 for none) is measured by
the car network with σ² `σ2`, the others true (SURE times their network's
σ).
"""
function lane_filter(eq, models, erring, σ2)
    tm = tracker_model(eq)
    ns = (models.e.filter, models.ψe.filter, models.κ_profile.filter)
    typ = (typical_σ(models.e)[1], typical_σ(models.ψe)[1], typical_σ(models.κ_profile)[1])
    gain(k) = k == erring ? 1.0 : SURE
    a = [exp(-T / n.τ) for n in ns]
    σb = [gain(k) * ns[k].σ_bias for k in 1:3]
    F6 = [tm.F3 zeros(3, 3); zeros(3, 3) Diagonal(a)]
    G6 = [tm.G3; zeros(3, 2)]
    c6 = [tm.c3; zeros(3)]
    Q = Diagonal([TRACKER.q_e^2 * T, TRACKER.q_ψ^2 * T, (TRACKER.q_κ * eq.v * T)^2, (σb .^ 2 .* (1 .- a .^ 2))...])
    σ = [k == erring ? σ2 : (SURE * typ[k])^2 for k in 1:3]
    R = Diagonal([max((1 - ns[k].persist) * σ[k], 0.01σ[k]) for k in 1:3])
    H = [I(3) I(3)]
    K, _ = kalman_gain(F6, H, Matrix(Q), Matrix(R))
    return (; K, F6, G6, c6, H)
end

# =====================================================================
# The controller's exact response to what a linear model can't take

function lane_state(lab; σ_sight=0.0, σ_clear=0.0, cut_profile=0, cut_widths=0)
    l = unflatten(lab)
    np = length(PROFILE_S)
    κp, wl, wr = copy(l.κ_profile), l.widths[1:np], l.widths[np+1:end]
    cut_profile > 0 && (κp[end-cut_profile+1:end] .= NaN)
    if cut_widths > 0
        wl[end-cut_widths+1:end] .= NaN
        wr[end-cut_widths+1:end] .= NaN
    end
    nk = length(CLEARANCE_K)
    return LaneState(l.e[1], l.ψe[1], κp[1], κp, wl, wr, l.sight[1], σ_sight, l.clearance,
        σ_clear isa AbstractVector ? collect(Float64, σ_clear) : fill(Float64(σ_clear), nk),
        NamedTuple{ESTIMATES}(ntuple(_ -> :truth, 6)), :ok)
end

# The change of the commands, at the steady state, from a lane state.
command_change(eq; kw...) = (s1 = drive(PLANNER, lane_state(eq.lab; kw...), eq.v, eq.r);
    s0 = drive(PLANNER, lane_state(eq.lab), eq.v, eq.r); [s1[1] - s0[1], s1[2] - s0[2]])

# The entries of a profile lost to a delay of n scans (shifted by the
# distance driven; past its end, unknown).
lost_entries(eq, n) = n == 0 ? 0 : count(j -> j + eq.v * n * T / PROFILE_STEP > length(PROFILE_S), 1:length(PROFILE_S))

# =====================================================================
# Two loops side by side, on the same random road

mutable struct Alloc
    n::Int
end
grab!(a::Alloc, k) = (r = a.n+1:a.n+k; a.n += k; r)

"""
    experiment(M, eq, kind, est; stats, filt, delay, Δu, road) -> (rms, mean, sd)

The RMS of the difference in lateral offset between the loop where `est`
errs (`kind` :accuracy: the car network's errors, `stats`; the filter
`filt` for e, ψe, κ) or comes late (`kind` :latency: `delay` scans, true),
and the reference loop, on the same random road (`road`: (a, σ²) of the
curvature's first-order process per scan's travel). `Δu`: a constant
change of the commands (what the controller does exactly to a shorter
profile or a larger σ).
"""
function experiment(M, eq, kind, est; stats=nothing, filt=nothing, delay=0, Δu=zeros(2), road, cmd=nothing, lowpass=0.0)
    nk = length(M.knots)
    idx = collect(RANGES[est])
    lane = haskey(LANE_COMP, est) ? LANE_COMP[est] : 0
    ℓ = lane > 0 ? LANE_IDX[lane] : 0           # its entry in the filter, if any
    raw = setdiff(idx, ℓ == 0 ? Int[] : [ℓ])    # its entries used as measured
    al = Alloc(0)
    Kn = grab!(al, nk)
    Xr, U1r, U2r = grab!(al, 8), grab!(al, 2), grab!(al, 2)
    Xe, U1e, U2e = grab!(al, 8), grab!(al, 2), grab!(al, 2)
    E = kind === :accuracy ? grab!(al, length(idx)) : 1:0
    Eu = cmd === nothing ? (1:0) : grab!(al, 2)   # the commands' noise from the measured entries' errors
    Fh = kind === :accuracy && lane > 0 ? grab!(al, 6) : 1:0
    Ry = kind === :latency && lane > 0 ? [grab!(al, 1)[1] for _ in 1:delay] : Int[]
    Rg = kind === :latency && lane > 0 ? [grab!(al, 1)[1] for _ in 1:delay] : Int[]
    Rc = kind === :latency && est in (:sight, :clearance) ? [grab!(al, 2) for _ in 1:delay] : UnitRange{Int}[]
    n = al.n
    nw = 1 + length(E) + length(Eu)
    A, B, c = zeros(n, n), zeros(n, nw), zeros(n)

    # the road: knots one scan's travel apart, moving back each scan; the farthest new
    for j in 1:nk-1
        A[Kn[j], Kn[j+1]] = 1.0
    end
    A[Kn[end], Kn[end]] = road.a
    B[Kn[end], 1] = 1.0

    # the car and its commands, both loops; the commands from the true estimates
    for (X, U1, U2) in ((Xr, U1r, U2r), (Xe, U1e, U2e))
        A[X, X] = M.Φ
        A[X, U1] = M.Γnew
        A[X, U2] = M.Γold
        A[X, Kn[M.k0]] = M.Γw[:, 1]
        A[U2, U1] = I(2)
        A[U1, X[1:2]] = M.KL * M.Hx
        A[U1, Kn] = M.KL * M.Hk
        A[U1, X[3]] .+= M.Kv
        A[U1, X[4]] .+= M.Kr
    end
    frozen_speed = all(iszero, M.KL[2, :]) && M.Kv[2] == 0 && M.Kr[2] == 0
    c[U1e] .+= frozen_speed ? [Δu[1], 0.0] : Δu

    truth_rows(rows) = (Hx=M.Hx[rows, :], Hk=M.Hk[rows, :])
    if kind === :accuracy
        s = stats
        # the error: a first-order process with the measured covariance and correlation
        A[E, E] = s.ρ * I(length(idx))
        B[E, 2:1+length(E)] = I(length(idx))
        b = s.slope
        # the measured entries: true + slope × (true's change) + error + mean
        if !isempty(raw)
            r = [findfirst(==(j), idx) for j in raw]
            KLr = M.KL[:, raw]
            A[U1e, Xe[1:2]] .+= KLr * Diagonal(b[r]) * M.Hx[raw, :]
            A[U1e, Kn] .+= KLr * Diagonal(b[r]) * M.Hk[raw, :]
            if cmd === nothing
                A[U1e, E[r]] .+= KLr
                c[U1e] .+= KLr * s.μ[r]
            else
                # the planner's exact response, as a noise on the commands
                keep = frozen_speed ? [1.0, 0.0] : [1.0, 1.0]
                A[Eu, Eu] = cmd.ρ * I(2)
                B[Eu, 1+length(E)+1:end] = I(2)
                A[U1e, Eu] .+= I(2)
                c[U1e] .+= keep .* cmd.m
            end
        end
        if lane > 0
            # the filter's estimate in place of the true one
            A[U1e, Fh[lane]] .+= M.KL[:, ℓ]
            A[U1e, Xe[1:2]] .-= M.KL[:, ℓ] * M.Hx[ℓ, :]'
            A[U1e, Kn] .-= M.KL[:, ℓ] * M.Hk[ℓ, :]'
            # the filter, one scan on: predicted with the next speed and yaw rate, updated with
            # the next measurements (true for the others; true + slope + error + mean for this one)
            Kg, F6, G6, c6, H = filt.K, filt.F6, filt.G6, filt.c6, filt.H
            IKH = I - Kg * H
            Hz = truth_rows(LANE_IDX)
            r1 = findfirst(==(ℓ), idx)
            zx = Hz.Hx .+ (b[r1] .* (1:3 .== lane)) .* Hz.Hx    # 3 × 2: the slope on this one's row
            zk = Hz.Hk .+ (b[r1] .* (1:3 .== lane)) .* Hz.Hk
            ez = Float64.(1:3 .== lane)
            A[Fh, Fh] .+= IKH * F6
            A[Fh, :] .+= IKH * G6 * A[Xe[3:4], :]
            B[Fh, :] .+= IKH * G6 * B[Xe[3:4], :]
            A[Fh, :] .+= Kg * (zx * A[Xe[1:2], :] + zk * A[Kn, :] + ez * A[E[r1:r1], :])
            B[Fh, :] .+= Kg * (zx * B[Xe[1:2], :] + zk * B[Kn, :] + ez * B[E[r1:r1], :])
            c[Fh] .+= IKH * (c6 + G6 * c[Xe[3:4]]) + Kg * (zx * c[Xe[1:2]] + zk * c[Kn] + ez * s.μ[r1])
        end
    else
        if lane > 0 && delay > 0
            # e, ψe, κ: the late true value, carried forward by the motion model
            tm = tracker_model(eq)
            others = setdiff(1:3, lane)
            Hz = truth_rows(LANE_IDX)
            A[Ry[1], Xe[1:2]] = Hz.Hx[lane, :]
            A[Ry[1], Kn] = Hz.Hk[lane, :]
            for j in 2:delay
                A[Ry[j], Ry[j-1]] = 1.0
            end
            # the model's change per scan: through the other (on-time, true) estimates and v, r
            gy = T .* tm.Jy[lane, others]
            gu = T .* tm.Ju[lane, :]
            A[Rg[1], Xe[1:2]] = gy' * Hz.Hx[others, :]
            A[Rg[1], Kn] = gy' * Hz.Hk[others, :]
            A[Rg[1], Xe[3]] += gu[1]
            A[Rg[1], Xe[4]] += gu[2]
            c[Rg[1]] = tm.c3[lane]
            for j in 2:delay
                A[Rg[j], Rg[j-1]] = 1.0
            end
            A[U1e, Ry[delay]] .+= M.KL[:, ℓ]
            for j in 1:delay
                A[U1e, Rg[j]] .+= M.KL[:, ℓ]
            end
            A[U1e, Xe[1:2]] .-= M.KL[:, ℓ] * Hz.Hx[lane, :]'
            A[U1e, Kn] .-= M.KL[:, ℓ] * Hz.Hk[lane, :]'
        end
        if est in (:sight, :clearance) && delay > 0
            # as they were at the scan
            KLi = M.KL[:, idx]
            A[Rc[1], Xe[1:2]] = KLi * M.Hx[idx, :]
            A[Rc[1], Kn] = KLi * M.Hk[idx, :]
            for j in 2:delay
                A[Rc[j], Rc[j-1]] = I(2)
            end
            A[U1e, Rc[delay]] .+= I(2)
            A[U1e, Xe[1:2]] .-= KLi * M.Hx[idx, :]
            A[U1e, Kn] .-= KLi * M.Hk[idx, :]
        end
    end

    # the speed command through the ramp's equivalent low-pass, in both loops
    lowpass > 0 && ramp_lowpass!(A, B, c, (U1r[2], U1e[2]), lowpass)
    stable(A) || return (rms=Inf, mean=NaN, sd=NaN)
    C = zeros(1, n)
    C[1, Xe[1]] = 1.0
    C[1, Xr[1]] = -1.0
    m = (C*((I-A)\c))[1]
    W = zeros(nw, nw)
    W[1, 1] = road.σ2
    kind === :accuracy && (W[2:1+length(E), 2:1+length(E)] = (1 - stats.ρ^2) .* stats.Σ)
    if cmd !== nothing
        keep = frozen_speed ? Diagonal([1.0, 0.0]) : Diagonal([1.0, 1.0])
        W[end-1:end, end-1:end] = (1 - cmd.ρ^2) .* (keep * cmd.S * keep)
    end
    P = stationary_covariance(A, B * W * B')
    v = max((C*P*C')[1], 0.0)
    return (rms=sqrt(m^2 + v), mean=m, sd=sqrt(v))
end

# =====================================================================
# One situation

"""
    ramp_lowpass!(A, B, c, rows, a)

Make the command rows `rows` (each the speed command of a loop) first-order
low-passes of what they were, `a` the pole: what the command limiter's
ramp does to the speed command in the loop (its describing function: fast
swings clipped, the mean passed). The steady state stays the same.
"""
function ramp_lowpass!(A, B, c, rows, a)
    for row in rows
        A[row, :] .*= (1 - a)
        A[row, row] += a
        B[row, :] .*= (1 - a)
        c[row] *= (1 - a)
    end
    return A
end

"""
    ramp_pole(M) -> pole or nothing

Where the speed loop is unstable in small signal (the command limiter's
ramps then hold the speed to a chatter about its steady value): the pole of
the shortest low-pass on the speed command (time constants 25 ms to 5 s)
that makes the loop stable, with a spectral radius under 0.995. The limit
cycle settles where the loop, with the ramp's describing function, is just
stable. `nothing` if none does.
"""
function ramp_pole(M)
    A0, _, _, _ = closed_loop(M)
    for τ in exp.(range(log(0.025), log(5.0); length=60))
        a = exp(-T / τ)
        A = copy(A0)
        ramp_lowpass!(A, zeros(size(A, 1), 1), zeros(size(A, 1)), (10,), a)
        maximum(abs, eigvals(A)) < 0.995 && return a
    end
    return nothing
end

"""
    situation_lane(key) -> κfun or nothing

The lane of a situation `(κ0, width, κ1, D)`: constant curvature κ0 (`D`
infinite), or κ0 at the car and κ1 from `D` meters ahead (over a 1 m ramp):
a bend ahead of a straight, or a straight ahead of a bend.
"""
situation_lane((κ0, _, κ1, D)) = isinf(D) ? nothing :
    (s -> s < D ? κ0 : s < D + 1 ? κ0 + (κ1 - κ0) * (s - D) : κ1)

describe_situation((κ0, W, κ1, D)) = isinf(D) ? "κ $(κ0)" :
    κ0 == 0 ? "straight, κ $(κ1) bend $(Int(D)) m ahead" : "κ $(κ0) bend, straight $(Int(D)) m ahead"

"""
    analyze_matched(key, models, lib, road) -> NamedTuple or nothing

Accuracy (RMS of the difference with the car network's errors) and latency
(the same at each delay, true but late) for every estimate at a situation.
"""
function analyze_matched(key, models, lib, (σκ, ℓ))
    κ0, width = key[1], key[2]
    eq = equilibrium(κ0, width; κfun=situation_lane(key))
    eq === nothing && return nothing
    M = linearize(eq)
    A0, _, _, _ = closed_loop(M)
    # an unstable speed loop: the ramp's low-pass; failing that, the speed held
    lowpass, held = 0.0, false
    if !stable(A0)
        a = ramp_pole(M)
        a === nothing ? (M = frozen(M); held = true) : (lowpass = a)
    end
    a = exp(-M.Δ / ℓ)
    road = (a=a, σ2=(1 - a^2) * σκ^2)
    acc, lat = Dict{Symbol,Any}(), Dict{Symbol,Any}()
    for est in ESTIMATES
        s = error_stats(lib, eq.lab, est)
        lane = haskey(LANE_COMP, est) ? LANE_COMP[est] : 0
        filt = lane > 0 ? lane_filter(eq, models, lane, s.σ2[1]) : nothing
        # sight and clearance: taken k_σ σ short, σ the tracker's (the white share and the bias)
        f = models[est].filter
        σt = sqrt(max((1 - f.persist), 0.01) * mean(s.σ2) + f.σ_bias^2)
        if STATISTICAL
            # the measured entries through the planner exactly (their σ's offsets included)
            cmd = command_noise(eq, lib, models, est, s.scans)
            acc[est] = experiment(M, eq, :accuracy, est; stats=s, filt, road, cmd, lowpass)
        else
            Δu = est === :sight ? command_change(eq; σ_sight=σt) : est === :clearance ? command_change(eq; σ_clear=σt) :
                 zeros(2)
            acc[est] = experiment(M, eq, :accuracy, est; stats=s, filt, Δu, road, lowpass)
        end
        lat[est] = map(DELAYS) do d
            driven = eq.v * d * T
            Δl = est === :κ_profile ? command_change(eq; cut_profile=lost_entries(eq, d)) :
                 est === :widths ? command_change(eq; cut_widths=lost_entries(eq, d)) :
                 est === :sight ? command_change(eq; σ_sight=TRACKER.q_sight * driven) :
                 est === :clearance ? command_change(eq; σ_clear=TRACKER.q_clearance * driven) : zeros(2)
            experiment(M, eq, :latency, est; delay=d, Δu=Δl, road, lowpass)
        end
    end
    return (; eq, held, lowpass, acc, lat)
end

# per 25 ms (one scan), least squares through the value on time, over 0 to 100 ms
latency_slope(rs) = (r = [x.rms for x in rs]; d = collect(DELAYS);
    all(isfinite, r) ? sum(d .* (r .- r[1])) / sum(abs2, d) : Inf)

# =====================================================================
# How much of the racetracks is like each situation

const APPROACH_KAPPA = (0.1, 0.2, 0.3)    # the bend ahead of a straight
const EXIT_KAPPA = (0.1, 0.2, 0.3)        # the bend a straight is ahead of
const AHEAD = (2.0, 5.0, 8.0)             # how far ahead [m]

nearest(xs, x) = xs[argmin(abs.(collect(xs) .- x))]

"""
    track_situations(ℓ) -> (weights, spread)

How much of the 23 racetracks' center lines (every 0.25 m) is like each
situation `(κ0, width, κ1, D)`, by the nearest |κ| (through points 1 m
apart) and width (the two sides' room, from their center-line files), and
what is within 10 m ahead (the range of clearance and sight): on a straight
with a bend (|κ| > 0.05 1/m) ahead, that bend (its greatest |κ| over 3 m)
and how far; in a bend (κ ≥ 0.1 1/m) with a straight (|κ| < 0.03) ahead,
how far; else a lane of constant curvature (`D` infinite).
`weights[key]`, the share; `spread[key]`, how much the curvature varies
ahead of the car there (the RMS over those points of |κ|'s standard
deviation over the next `ℓ` meters, the road's correlation length): the
road each situation's loops are driven by.
"""
function track_situations(ℓ)
    len_w, sq = Dict{NTuple{4,Float64},Float64}(), Dict{NTuple{4,Float64},Float64}()
    for name in sort(collect(keys(F1Tenth.RACETRACKS)))
        dir = dirname(F1Tenth.racetrack_yaml(name))
        rows = [parse.(Float64, split(l, ",")) for l in eachline(joinpath(dir, name * "_centerline.csv"))
                if !startswith(l, "#") && !isempty(strip(l))]
        c = Path(getindex.(rows, 1), getindex.(rows, 2); closed=true)
        width = [r[3] + r[4] for r in rows]
        len = c.s[end] + c.seg_length[end]
        ss = collect(0.0:0.25:len-0.25)
        ks = [abs(menger_curvature(point_at(c, mod(s - 0.5, len)), point_at(c, s), point_at(c, mod(s + 0.5, len))))
              for s in ss]
        n, ahead = length(ks), round(Int, ℓ / 0.25)
        k(j) = ks[mod1(j, n)]
        for i in 1:n
            W = nearest(GRID_WIDTH, width[clamp(searchsortedlast(c.s, ss[i]), 1, length(width))])
            κc = nearest(GRID_KAPPA, ks[i])
            key = (κc, W, κc, Inf)
            if κc == 0.0
                j = findfirst(j -> k(i + j) > 0.05, 1:40)
                j === nothing || (key = (0.0, W, nearest(APPROACH_KAPPA, maximum(k(i + j + q) for q in 0:12)),
                    nearest(AHEAD, 0.25j)))
            elseif κc >= 0.1
                j = findfirst(j -> k(i + j) < 0.03, 1:40)
                j === nothing || (key = (nearest(EXIT_KAPPA, ks[i]), W, 0.0, nearest(AHEAD, 0.25j)))
            end
            len_w[key] = get(len_w, key, 0.0) + 0.25
            sq[key] = get(sq, key, 0.0) + 0.25 * var([k(i + j) for j in 0:ahead])
        end
    end
    tot = sum(values(len_w))
    return Dict(k => v / tot for (k, v) in len_w), Dict(k => sqrt(sq[k] / len_w[k]) for k in keys(len_w))
end

# =====================================================================

function matched_main()
    models = load_errors(joinpath(OUT, "error_model.toml"); network=:small_weak)
    lib = load_library(LIBRARY)
    # the road: the racetracks' curvature's correlation length, and how much it varies ahead in each situation
    _, ℓ = road_statistics()
    weights, spread = track_situations(ℓ)
    @printf("road: correlation length %.2f m\n", ℓ)
    grid = Dict{NTuple{4,Float64},Any}()
    for key in sort(collect(keys(weights)); by=k -> (isinf(k[4]) ? 0 : 1, k))
        t = time()
        r = analyze_matched(key, models, lib, (spread[key], ℓ))
        grid[key] = r
        r === nothing ? @printf("%s, width %.2f: no steady state\n", describe_situation(key), key[2]) :
        @printf("%s, width %.2f (%.1f%% of the tracks, σκ %.3f, %s; %.0f s): %s\n", describe_situation(key), key[2],
            100weights[key], spread[key], r.held ? "speed held" : r.lowpass > 0 ? "ramp" : "stable", time() - t,
            join(("$(est) $(fmt(r.acc[est].rms)) / $(fmt(latency_slope(r.lat[est])))" for est in ESTIMATES), ", "))
        flush(stdout)
    end
    write_matched(grid, weights, spread, ℓ)
end

function write_matched(grid, weights, spread, ℓ)
    # where the grid has no steady state, its share goes to the nearest curvature that has one
    # a transition without a steady state counts as its lane of constant curvature; one of those without,
    # as the nearest curvature that has one
    ok = [k for (k, r) in grid if r !== nothing && isinf(k[4])]
    moved = 0.0
    wts = Dict{NTuple{4,Float64},Float64}()
    for (key, w) in weights
        if grid[key] !== nothing
            wts[key] = get(wts, key, 0.0) + w
        else
            k2 = (key[1], key[2], key[1], Inf)
            near = k2 in ok ? k2 : ok[argmin([abs(k[1] - key[1]) + 0.01abs(k[2] - key[2]) for k in ok])]
            wts[near] = get(wts, near, 0.0) + w
            moved += w
        end
    end
    pooled(f) = sqrt(sum(w * f(grid[k])^2 for (k, w) in wts))
    acc = Dict(est => pooled(r -> r.acc[est].rms) for est in ESTIMATES)
    # latency: the weighted RMS at each delay, then its growth per scan
    lat = Dict(est => latency_slope([(rms=pooled(r -> r.lat[est][i].rms),) for i in eachindex(DELAYS)]) for est in ESTIMATES)
    emp = TOML.parsefile(joinpath(OUT, "empirical.toml"))
    ratio(a, e) = isfinite(a) && e > 0 ? @sprintf("%.2g", a / e) : "–"
    rows = map(ESTIMATES) do est
        e = emp[String(est)]
        "| `$est` | $(fmt(acc[est])) | $(fmt(e["accuracy"])) ($(fmt(e["accuracy_lo"]))–$(fmt(e["accuracy_hi"]))) | " *
        "$(ratio(acc[est], e["accuracy"])) | $(fmt(lat[est])) | $(fmt(e["latency"])) ($(fmt(e["latency_lo"]))–" *
        "$(fmt(e["latency_hi"]))) | $(ratio(lat[est], e["latency"])) |"
    end
    sits = sort(collect(keys(wts)); by=k -> (isinf(k[4]) ? 0 : 1, k))
    sit_rows = map(sits) do k
        r = grid[k]
        loop = r.held ? "speed held" : r.lowpass > 0 ? "ramp ($(fmt(-T / log(r.lowpass); d=2)) s)" : "stable"
        "| $(describe_situation(k)) | $(k[2]) | $(fmt(100wts[k]; d=2)) | $(fmt(spread[k]; d=2)) | $(fmt(r.eq.v)) | $loop | " *
        join(("$(fmt(r.acc[est].rms)) / $(fmt(latency_slope(r.lat[est])))" for est in ESTIMATES), " | ") * " |"
    end
    write(joinpath(OUT, "matched.md"), """
        # The simulation's sensitivities, from the linearized closed loop

        No simulation and no map, but the same questions as the simulation campaigns
        ([empirical.toml](empirical.toml), from the simulation branch's sensitivity.md): the RMS lateral
        deviation from the reference run with the car network's errors on one estimate (accuracy), and its
        growth per 25 ms the estimate comes late from the cloud, true (latency). From
        [`matched.jl`](../matched.jl), which says how each part is modeled: analytic.jl's linearized closed
        loop with LidarLaneNets.jl's Kalman filter in it (as the car runs it), driven by random roads (each
        situation's lane bending about its curvature as much as the racetracks' do ahead of the car in that
        situation, decorrelating over $(fmt(ℓ)) m) and by the network's recorded errors' statistics at each
        situation, and its stationary covariance (the Lyapunov equation).
        Each situation weighted by how much of the 23 racetracks is like it: a lane of constant curvature, or,
        within 10 m (the range of clearance and sight), a straight with a bend ahead or a bend with a straight
        ahead (the scene held as it is about the car)$(moved > 0 ? @sprintf("; %.1f%% of the tracks' length, in situations without a steady state, counted as the nearest that has one", 100moved) : ""). Where the speed loop
        is unstable in small signal (tight bends), the speed command goes through the command limiter's ramp's
        describing function (a low-pass, the shortest that makes the loop just stable; "ramp" below), as on the
        car, where the ramp holds the speed to a chatter of a few cm/s about its steady value.

        ## Against the simulation

        | Estimate | Accuracy: analytic [m] | simulation [m] | ratio | Latency: analytic [m per 25 ms] | simulation | ratio |
        |---|---|---|---|---|---|---|
        $(join(rows, "\n"))

        ## By situation

        Accuracy [m] / latency [m per 25 ms], per estimate.

        | Situation | width [m] | share of the tracks [%] | road: σκ [1/m] | speed [m/s] | loop | $(join(("`$e`" for e in ESTIMATES), " | ")) |
        |---|---|---|---|---|---|$(repeat("---|", length(ESTIMATES)))
        $(join(sit_rows, "\n"))

        ## What it leaves out

        Crashes (a linear model has none), the speed planner's switches between its limits, the robust
        (Student t) updates and the restarts of the tracker, the controller's steps other than the two computed
        exactly (a profile's far end lost to a delay; sight and clearance taken k_σ σ short), errors that depend
        on more than the lane (the matching sees the true lane only), and bends that tighten or open (every
        situation is a lane of constant curvature, its road varying about it).
        """)
    println(joinpath(OUT, "matched.md"))
end

if abspath(PROGRAM_FILE) == @__FILE__
    matched_main()
end
