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
        for p in (reference(), accuracy(:e, 2.0; source=:parametric), accuracy(:ψe, 1.0; network=:large, source=:parametric),
                  latency(:e, 0.05))
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

    @testset "situations and replay" begin
        bend(κ_here, κ_ahead) = lane_labels(; e=0.0, ψe=0.0,
            κ_profile=[s < 0.5 ? κ_here : κ_ahead for s in PROFILE_S], w_left=fill(1.0, NP), w_right=fill(1.0, NP),
            sight=SIGHT_MAX, clearance=fill(CLEARANCE_MAX, NK))
        @test SITUATIONS[situation_class(bend(0.0, 0.01))] == "straight"
        @test SITUATIONS[situation_class(bend(0.0, 0.3))] == "medium entry"
        @test SITUATIONS[situation_class(bend(0.1, 0.1))] == "gentle middle"
        @test SITUATIONS[situation_class(bend(-0.8, 0.1))] == "tight exit"

        # a library of 2 drives × 6 scans on 2 tracks: drive 1 bends left (κ 0.3), drive 2 is straight;
        # e's car error = the scan's number, the cloud's its negative
        lanes = [k <= 6 ? bend(0.3, 0.3) : bend(0.0, 0.0) for k in 1:12]
        features = reduce(hcat, situation_features.(lanes))
        follows = BitVector([1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0])
        err = Dict((est, r) => zeros(Float32, output_size(est), 12) for est in ESTIMATES for r in (:car, :cloud))
        σ = Dict((est, r) => fill(Float32(0.5), output_size(est), 12) for est in ESTIMATES for r in (:car, :cloud))
        err[(:e, :car)][1, :] = 1:12
        err[(:e, :cloud)][1, :] = -(1:12)
        err[(:widths, :car)][1, :] .= 0.1    # the left side's
        truth = Dict(est => Float32.(reduce(hcat, (l[est] for l in lanes))) for est in ESTIMATES)
        lib = TrajectorySensitivity.ErrorLibrary((:small_weak, :large), ["A", "B"], [fill(1, 6); fill(2, 6)], follows,
            features, truth, err, σ)
        d = ReplayDraws(lib, :e, Xoshiro(1); block=3, near=2, keep=0.5)
        straight = straight_lane()
        # (a straight lane is its own mirror image: either way round)
        seen = [abs(TrajectorySensitivity.oracle_errors!(d, :car, 0.025k, 0.025, :e, straight)[1][1]) for k in 1:30]
        @test all(x -> 7 <= x <= 12, seen)                                # replayed from a straight lane
        @test count(k -> seen[k+1] == seen[k] + 1, 1:29) >= 10            # in consecutive scans
        e1, _ = TrajectorySensitivity.oracle_errors!(d, :car, 99.0, 0.025, :e, straight)
        e2, s2 = TrajectorySensitivity.oracle_errors!(d, :cloud, 99.0, 0.025, :e, straight)
        @test e2 == -e1 && s2 == [0.5]                                    # the car's and the cloud's: the same scan
        # a right bend replays the left bend's errors, mirrored
        dr = ReplayDraws(lib, :widths, Xoshiro(1); near=1)
        w, _ = TrajectorySensitivity.oracle_errors!(dr, :car, 0.025, 0.025, :widths, bend(-0.3, -0.3))
        @test w[NP+1] ≈ 0.1 && w[1] == 0 && dr.flip
        de = ReplayDraws(lib, :e, Xoshiro(1); near=1)
        @test TrajectorySensitivity.oracle_errors!(de, :car, 0.025, 0.025, :e, bend(-0.3, -0.3))[1][1] < 0
        # carried over to the car's lane by the mean error: κ's error = -0.5 × the true κ, the recorded
        # scans' true κ spread from 0.025 to 0.3
        truth2 = deepcopy(truth)
        truth2[:κ_profile] .= Float32.(0.025 .* (1:12)')
        err[(:κ_profile, :car)] .= -0.5f0 .* truth2[:κ_profile]
        lib2 = TrajectorySensitivity.ErrorLibrary((:small_weak, :large), ["A", "B"], [fill(1, 6); fill(2, 6)], follows,
            features, truth2, err, σ; min_bin=1)
        m = lib2.mean_err[(:κ_profile, :car)]
        @test isapprox(m(1, 0.2), -0.1; atol=0.01) && isapprox(m(1, -0.2), 0.1; atol=0.01)
        dk = ReplayDraws(lib2, :κ_profile, Xoshiro(1); near=1)
        @test TrajectorySensitivity.oracle_errors!(dk, :car, 0.025, 0.025, :κ_profile, bend(0.3, 0.3))[1][1] ≈ -0.15 atol = 0.01
        # leaving a track out
        dw = ReplayDraws(without(lib, "B"), :e, Xoshiro(1); near=3)
        @test all(k -> abs(TrajectorySensitivity.oracle_errors!(dw, :car, 0.025k, 0.025, :e, straight)[1][1]) <= 6, 1:10)
        @test without(lib, "C") === lib

        @test cloud_call(:sight, 0.05).kind === :cloud && real_network(:e; network=:large).network === :large
        @test_throws ArgumentError accuracy(:e, 1.0; source=:other)
        racer, oracle = oracle_racer(CAR, BEAMS, cloud_call(:e, 0.05), MODELS; library=lib)
        @test racer.estimator.schedule isa FixedCalls && racer.estimator.cfg.remote_latency == 0.05
        for k in 1:6
            set_truth!(oracle, straight_lane(e=0.1), 0.025k)
            control!(racer, zeros(length(BEAMS)), 3.0, 0.0, 0.025)
        end
        @test racer.estimator.calls[:e] == 6
        @test_throws ArgumentError oracle_racer(CAR, BEAMS, cloud_call(:e, 0.05), MODELS)   # no library
        @test_throws ArgumentError oracle_racer(CAR, BEAMS, real_network(:e), MODELS)         # no nets
    end

    @testset "sobol indices" begin
        # Ishigami: known indices (Sobol' & Levitan 1999)
        ish(x) = sin(x[1]) + 7sin(x[2])^2 + 0.1x[3]^4 * sin(x[1])
        A, B = saltelli_design(Xoshiro(3), 20_000, 3)
        A, B = π .* (2A .- 1), π .* (2B .- 1)
        f(M) = [ish(r) for r in eachrow(M)]
        S, ST, _ = sobol_indices(f(A), f(B), reduce(hcat, (f(mix(A, B, i)) for i in 1:3)))
        @test isapprox(S, [0.314, 0.442, 0.0]; atol=0.03)
        @test isapprox(ST, [0.558, 0.442, 0.244]; atol=0.03)
        @test mix(A, B, 2)[:, 2] == B[:, 2] && mix(A, B, 2)[:, 1] == A[:, 1]

        # a joint experiment: every estimate errs, from the same recorded scans
        lanes = [straight_lane() for _ in 1:4]
        lib = TrajectorySensitivity.ErrorLibrary((:small_weak, :large), ["A"], fill(1, 4), BitVector([1, 1, 1, 0]),
            reduce(hcat, situation_features.(lanes)),
            Dict(est => Float32.(reduce(hcat, (l[est] for l in lanes))) for est in ESTIMATES),
            Dict((est, r) => fill(0.01f0, output_size(est), 4) for est in ESTIMATES for r in (:car, :cloud)),
            Dict((est, r) => fill(0.05f0, output_size(est), 4) for est in ESTIMATES for r in (:car, :cloud)); min_bin=1)
        p = joint_accuracy((0.0, 1.0, 2.0, 0.0, 0.0, 0.5); seed=2)
        @test p.gains.ψe == 1.0 && p.gains.κ_profile == 2.0 && occursin("joint", describe(p))
        racer, oracle = oracle_racer(CAR, BEAMS, p, MODELS; library=lib)
        @test racer.estimator.schedule isa NoCalls
        @test racer.estimator.nets.small.ψe.draws === racer.estimator.nets.small.κ_profile.draws    # one replay for all
        set_truth!(oracle, straight_lane(), 0.025)
        μ, _ = predict(racer.estimator.nets.small.κ_profile, zeros(length(BEAMS)))
        @test all(x -> isapprox(x, 0.02; atol=1e-6), μ)              # 2 × the recorded 0.01
        @test predict(racer.estimator.nets.small.e, zeros(length(BEAMS)))[1] == straight_lane().e   # gain 0: true
        @test_throws ArgumentError oracle_racer(CAR, BEAMS, p, MODELS)
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
