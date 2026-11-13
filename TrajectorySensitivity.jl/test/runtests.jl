using LidarLaneNets
using LidarRacer
using LinearAlgebra
using Random
using StanleyController: Path
using Test
using TrajectorySensitivity
using TrajectorySensitivity: _Φ, _step!

# NUC5, as in F1Tenth.jl's defaults
const CAR = (lf=0.130, lr=0.127, width=0.228, s_max=0.4189, steer_cmd_min=-0.3271441202,
    steer_cmd_max=0.2917771883, speed_cmd_max=14.375, decel_cmd=2.5, scan_rate=40.0, scan_range=10.0)
const BEAMS = range(-3π / 4, 3π / 4; length=1081)
const NP = length(PROFILE_S)
const NK = length(CLEARANCE_K)

# A network whose errors are Gaussian with standard deviation `s` (the
# quantiles of a standard Gaussian), persisting with (persist, τ), σ = s.
function gaussian_errors(est, s; persist=1.0, τ=0.1, c=0.0, at=[0.0], scale=[1.0])
    n = output_size(est)
    return NetworkErrors(est, :test, fill(s, n), at, scale, scale, collect(SCORES), GaussProcess(Matrix(1.0I, n, n), persist, τ),
        fill(log(s), n), fill(1e-6, n), c, GaussProcess(Matrix(1.0I, n, n), 1.0, 1.0),
        EstimateNoise(persist=persist, τ=τ, σ_bias=s, ν_small=Inf, ν_large=Inf))
end

models_for(s) = NamedTuple{ESTIMATES}(Tuple(gaussian_errors(est, s) for est in ESTIMATES))
const MODELS = Dict(:small_weak => models_for(0.01), :large => models_for(0.005))

straight_lane(; e=0.0, ψe=0.0) = lane_labels(; e, ψe, κ_profile=zeros(NP), w_left=fill(1.0, NP), w_right=fill(1.0, NP),
    sight=SIGHT_MAX, clearance=fill(CLEARANCE_MAX, NK))

@testset "TrajectorySensitivity" begin
    @testset "scores" begin
        q = collect(2.0 .* SCORES .+ 1.0)
        for u in (-3.9, -1.0, 0.0, 0.37, 2.5)
            @test from_score(q, u) ≈ 2u + 1
            @test to_score(q, from_score(q, u)) ≈ u atol = 1e-9
        end
        @test from_score(q, 10.0) == q[end] && to_score(q, 1e6) == last(SCORES)
        @test to_score([0.0, 0.0, 0.0, 1.0][[1, 2, 3, 4, 4]], 0.0) isa Float64
    end

    @testset "error draws" begin
        rng = Xoshiro(1)
        g = GaussProcess([1.0 0.6; 0.6 1.0], 0.8, 0.1)
        slow, xs = zeros(2), Matrix{Float64}(undef, 2, 40_000)
        for k in axes(xs, 2)
            xs[:, k] = _step!(slow, g, 0.025, k > 1, rng)
        end
        @test correlation(g) ≈ [1.0 0.6; 0.6 1.0] atol = 1e-6
        @test all(abs.(sum(abs2, xs; dims=2) ./ size(xs, 2) .- 1) .< 0.05)            # unit variance
        @test abs(sum(xs[1, :] .* xs[2, :]) / size(xs, 2) - 0.6) < 0.05               # across entries
        ρ1 = sum(xs[1, 1:end-1] .* xs[1, 2:end]) / (size(xs, 2) - 1)
        @test abs(ρ1 - 0.8exp(-0.025 / 0.1)) < 0.05                                    # over time

        m = gaussian_errors(:e, 0.02)
        d = ErrorDraws(m, Xoshiro(2))
        es = [draw!(d, 0.025, [0.0]) for _ in 1:20_000]
        err = [e[1][1] for e in es]
        @test abs(sqrt(sum(abs2, err) / length(err)) - 0.02) < 0.002
        @test all(isapprox(e[2][1], 0.02; rtol=1e-4) for e in es)
        @test typical_σ(m) ≈ [0.02]

        # errors and σ scale with the situation: 1 on a straight, 10 times larger at κ = 0.5
        hard = gaussian_errors(:κ_profile, 0.01; at=[0.0, 0.5], scale=[1.0, 10.0])
        @test err_factor(hard, 0.0) == 1.0 && err_factor(hard, 0.25) ≈ 5.5 && err_factor(hard, 2.0) == 10.0
        @test σ_factor(hard, 0.5) == 10.0
        d = ErrorDraws(hard, Xoshiro(4))
        f = [j <= 10 ? 0.0 : 0.5 for j in 1:NP]
        es = [draw!(d, 0.025, f) for _ in 1:5000]
        rms(j) = sqrt(sum(e[1][j]^2 for e in es) / length(es))
        @test 7 < rms(20) / rms(5) < 13
        @test isapprox(es[1][2][20], 0.1; rtol=1e-3)
        lab = lane_labels(; e=0.1, ψe=0.0, κ_profile=fill(-0.3, NP), w_left=fill(1.0, NP), w_right=fill(2.0, NP),
            sight=4.0, clearance=fill(3.0, NK))
        @test situation(:e, lab) == [0.3] && situation(:ψe, lab) == [0.3] && situation(:sight, lab) == [4.0]
        @test situation(:κ_profile, lab) == fill(0.3, NP) && situation(:widths, lab)[end] == 2.0
    end

    @testset "oracle nets" begin
        o = Oracle()
        lab = straight_lane(e=0.3)
        @test_throws ArgumentError LidarLaneNets.predict(OracleNet(o, :e; σ_min=[0.1]), zeros(3))
        set_truth!(o, lab, 0.0)
        μ, σ = LidarLaneNets.predict(OracleNet(o, :e; σ_min=[0.1]), zeros(3))
        @test μ == [0.3] && σ == [0.1]
        μ, σ = LidarLaneNets.predict(OracleNet(o, :clearance; σ_min=fill(0.1, NK), silent=true), zeros(3))
        @test all(isnan, μ) && all(isinf, σ)
        net = OracleNet(o, :ψe; draws=ErrorDraws(gaussian_errors(:ψe, 0.01), Xoshiro(3)), gain=3.0, σ_min=[1e-4])
        μ, σ = LidarLaneNets.predict(net, zeros(3))
        @test μ[1] != 0.0 && isapprox(σ, [0.03]; rtol=1e-4)
        net0 = OracleNet(o, :ψe; draws=ErrorDraws(gaussian_errors(:ψe, 0.01), Xoshiro(3)), gain=0.0, σ_min=[1e-4])
        μ, σ = LidarLaneNets.predict(net0, zeros(3))
        @test μ == [0.0] && σ == [1e-4]
    end

    @testset "experiments" begin
        @test reference().kind === :none
        @test accuracy(:sight, 2.0).gain == 2.0
        @test latency(:e, 0.05).delay == 0.05
        @test_throws ArgumentError accuracy(:speed, 1.0)
        @test occursin("latency", describe(latency(:e, 0.05)))

        # the controller each experiment runs, driving on a straight lane
        for p in (reference(), accuracy(:e, 2.0), accuracy(:ψe, 1.0; network=:large), latency(:e, 0.05))
            racer, oracle = oracle_racer(CAR, BEAMS, p, MODELS)
            @test racer isa Racer
            steer, speed = 0.0, 0.0
            for k in 1:12
                set_truth!(oracle, straight_lane(e=0.1), 0.025k)
                steer, speed, lane = control!(racer, zeros(length(BEAMS)), 3.0, 0.0, 0.025)
            end
            @test isfinite(steer) && speed > 0
            sel, trk = racer.estimator.cfg, racer.tracker.cfg
            if p.kind === :latency
                @test racer.estimator.schedule isa FixedCalls && sel.remote_latency == 0.05
                @test racer.estimator.calls[:e] == 12 && racer.estimator.calls[:ψe] == 0
            else
                @test racer.estimator.schedule isa NoCalls
            end
            for est in ESTIMATES   # what the filter is told: as the network making those errors would
                acc = p.kind === :accuracy && p.estimate === est
                m = MODELS[acc ? p.network : :small_weak][est]
                @test trk.noise[est].σ_bias ≈ (acc ? p.gain : 0.01) * m.filter.σ_bias
            end
        end
        # a true lane, sure: the filter holds it
        racer, oracle = oracle_racer(CAR, BEAMS, reference(), MODELS)
        lane = nothing
        for k in 1:20
            set_truth!(oracle, straight_lane(e=0.2), 0.025k)
            _, _, lane = control!(racer, zeros(length(BEAMS)), 0.0, 0.0, 0.025)
        end
        @test lane.e ≈ 0.2 atol = 1e-3
        @test lane.status === :ok
    end

    @testset "trajectories" begin
        θs = range(0, 2π; length=401)[1:end-1]
        center = Path(10cos.(θs), 10sin.(θs); closed=true)   # a circle of radius 10, counterclockwise
        len = 2π * 10
        drive(off; speed=1.0, upto=2.2π) = begin
            lg = TrajectoryLog(center)
            for (k, a) in enumerate(range(0, upto; step=0.01))
                r = 10 - off   # off > 0: to the left, inside
                log!(lg, k * 0.1 / speed, r * cos(a), r * sin(a), a + π / 2, speed, 1.0 - off)
            end
            lg
        end
        ref, run = drive(0.0), drive(0.3)
        @test distance_driven(ref) > len   # counted on past the start line
        c = compare(run, ref)
        @test c.max_Δd ≈ 0.3 atol = 0.01
        @test c.rms_Δd ≈ 0.3 atol = 0.01
        @test c.min_gap ≈ 0.7
        @test !c.crashed
        slow = compare(drive(0.0; speed=0.5), ref)
        @test slow.Δlap > 0 && slow.max_Δd < 1e-6
        tb = tube([ref, run, drive(-0.1)]; distance=len)
        @test maximum(filter(isfinite, tb.width)) ≈ 0.4 atol = 0.01
        @test all(==(3), tb.n[2:end-5])
        behind = TrajectoryLog(center)   # starting just behind the start line
        log!(behind, 0.0, 10cos(-0.01), 10sin(-0.01), π / 2, 1.0, 1.0)
        log!(behind, 0.1, 10cos(0.01), 10sin(0.01), π / 2, 1.0, 1.0)
        @test behind.s[1] < 0 < behind.s[2] < 1
        segs = [(0.0, len / 2, "a"), (len / 2, len, "b")]
        seg = by_segment(c.s, abs.(c.Δd), segs)
        @test seg["a"] ≈ 0.3 atol = 0.01
        @test seg["b"] ≈ 0.3 atol = 0.01
    end

    @testset "linear analysis" begin
        Z = box([1.0, 2.0])
        @test widths(Z) == [2.0, 4.0]
        @test widths(linear_map([1.0 1.0], Z)) == [6.0]
        lo, hi = interval_hull(minkowski_sum(Z, Zonotope([1.0, 0.0], [0.5 0.0; 0.0 0.0])))
        @test lo == [-0.5, -2.0] && hi == [2.5, 2.0]
        @test jacobian(x -> [x[1]^2, x[1] * x[2]], [3.0, 2.0]) ≈ [6.0 0.0; 2.0 3.0] atol = 1e-6

        # ẋ = −x + u, sampled every 0.1 s, the input 0.15 s late: one whole step, then 0.05 s
        Φ, Γold, Γnew, Γw, steps = discretize(fill(-1.0, 1, 1), fill(1.0, 1, 1), fill(2.0, 1, 1), 0.1, 0.15)
        @test steps == 1
        @test Φ[1] ≈ exp(-0.1)
        @test Γnew[1] ≈ 1 - exp(-0.05)
        @test Γold[1] ≈ exp(-0.05) * (1 - exp(-0.05))
        @test Γold[1] + Γnew[1] ≈ 1 - exp(-0.1)
        @test Γw[1] ≈ 2(1 - exp(-0.1))

        @test stable(fill(0.5, 1, 1)) && !stable(fill(1.01, 1, 1))
        # a constant bias in [-1, 1] settles x' = 0.5x + b within ±2; x' = -0.5x + b within ±2/3
        @test widths(bias_set(fill(0.5, 1, 1), fill(1.0, 1, 1), fill(1.0, 1, 1), [1.0])) ≈ [4.0]
        @test widths(bias_set(fill(-0.5, 1, 1), fill(1.0, 1, 1), fill(1.0, 1, 1), [1.0])) ≈ [4 / 3]
        # a disturbance changing every step reaches ±Σ|aᵏ| = ±2 either way
        @test widths(disturbance_set(fill(0.5, 1, 1), fill(1.0, 1, 1), fill(1.0, 1, 1), [1.0])) ≈ [4.0] rtol = 1e-6
        @test widths(disturbance_set(fill(-0.5, 1, 1), fill(1.0, 1, 1), fill(1.0, 1, 1), [1.0])) ≈ [4.0] rtol = 1e-6
        # a delayed input: reaching the output only after a few steps
        A = [0.0 0.0 0.0; 1.0 0.0 0.0; 0.0 1.0 0.5]
        @test widths(disturbance_set(A, [1.0, 0.0, 0.0][:, :], [0.0 0.0 1.0], [1.0])) ≈ [4.0] rtol = 1e-6

        # x' = a x + w: variance q / (1 − a²)
        @test stationary_covariance(fill(0.9, 1, 1), fill(2.0, 1, 1))[1] ≈ 2 / (1 - 0.81)
        A2 = [0.5 0.2; -0.1 0.7]
        Q2 = [1.0 0.3; 0.3 2.0]
        P2 = stationary_covariance(A2, Q2)
        @test P2 ≈ A2 * P2 * A2' + Q2
        # a random walk measured with noise: the scalar Riccati equation's fixed point
        K, P = kalman_gain(fill(1.0, 1, 1), fill(1.0, 1, 1), fill(1.0, 1, 1), fill(1.0, 1, 1))
        @test P[1] ≈ (1 + sqrt(5)) / 2 && K[1] ≈ P[1] / (P[1] + 1)
    end

    @testset "tracks" begin
        # a rounded rectangle: two free straights, one free corner
        p = vcat(straight(NaN; width=2.0, label="long straight"), corner(0.5, π / 2; width=2.0, label="corner"),
            straight(NaN; width=1.5), corner(0.5, π / 2; width=1.5, label="corner"), straight(20.0; width=2.0),
            corner(0.5, π / 2; width=2.0, label="corner"), straight(15.0; width=2.0),
            bends([0.5], [NaN]; width=2.0, label="last corner"))
        t = build_track(TrackDesign(p))
        @test hypot(t.x[end] - t.x[1], t.y[end] - t.y[1]) < 0.1   # closes
        @test t.heading[end] ≈ 2π atol = 0.01
        @test t.length ≈ t.s[end] + (t.s[2] - t.s[1]) atol = 0.06
        @test isempty(track_problems(t))
        @test t.segments[1][3] == "long straight" && t.segments[end][3] == "last corner"
        @test t.segments[end][2] ≈ t.length
        @test minimum(t.width) ≈ 1.5 && maximum(t.width) ≈ 2.0
        img, origin = track_map(t; resolution=0.1)
        @test any(==(0x00), img) && count(==(0x00), img) < length(img) / 5
        dir = mktempdir()
        base = write_track(dir, "Test", t)
        @test isfile(base * ".pgm") && isfile(base * ".yaml")
        @test read_segments(joinpath(dir, "Test_segments.csv")) == [(Float64(a), Float64(b), l) for (a, b, l) in t.segments]
        @test_throws ArgumentError build_track(TrackDesign(straight(NaN; width=2.0)))
        @test_throws ArgumentError corner(0.5, 0.1; width=2.0, label="too short")
        # parts of the track too close: an inner wall folding over
        wide(w) = vcat(straight(NaN; width=w), corner(0.6, π / 2; ramp=1.0, width=w, label="c"), straight(NaN; width=w),
            corner(0.6, π / 2; ramp=1.0, width=w, label="c"), straight(20.0; width=w),
            corner(0.6, π / 2; ramp=1.0, width=w, label="c"), straight(15.0; width=w),
            bends([0.6], [NaN]; ramp=1.0, width=w, label="c"))
        @test isempty(track_problems(build_track(TrackDesign(wide(2.0)))))
        tight = build_track(TrackDesign(wide(4.0)))   # its inner walls fold over
        @test !isempty(track_problems(tight))
    end
end
