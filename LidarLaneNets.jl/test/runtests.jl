using LidarLaneNets
using Random
using Test

const NB = 1081
const NP = length(PROFILE_S)
const NK = length(CLEARANCE_K)

tier_nets(t; rng=Xoshiro(1)) = NamedTuple{ESTIMATES}(Tuple(EstimatorNet(est, t, NB; rng) for est in ESTIMATES))

meas(est, t, tier, value; σ=0.05) = Measurement(est, t, tier, collect(Float64, value), fill(σ, length(value)))

# e and ψe measured every scan from t = 0 at 40 Hz, with `extra(k, t)` added at scan k.
function run_tracker(n; v=3.0, extra=(k, t) -> Measurement[], e=0.1, ψe=0.0)
    tr = LaneTracker(0.257)
    local lane
    for k in 0:n-1
        t = 0.025k
        m = vcat([meas(:e, t, :small, [e]), meas(:ψe, t, :small, [ψe])], extra(k, t))
        lane = track!(tr, m, v, 0.0, t)
    end
    return tr, lane
end

@testset "LidarLaneNets" begin
    @testset "estimates" begin
        @test [output_size(est) for est in ESTIMATES] == [1, 1, NP, 2NP, 1, NK]
        @test_throws ArgumentError output_size(:speed)
        l = lane_labels(e=0.1, ψe=-0.2, κ_profile=fill(0.3, NP), w_left=fill(1.0, NP), w_right=fill(2.0, NP),
            sight=7.0, clearance=fill(4.0, NK))
        @test l.e == [0.1] && l.ψe == [-0.2] && l.sight == [7.0]
        @test w_left(l.widths) == fill(1.0, NP) && w_right(l.widths) == fill(2.0, NP)
        s = LaneState(l)
        @test s.e == 0.1 && s.w_right == fill(2.0, NP) && s.status === :ok && s.σ_sight == 0
    end

    @testset "scan features" begin
        r = fill(5.0, NB)
        r[1], r[5], r[9] = 65.533, NaN, 0.004
        f = scan_features(r, :small)
        @test length(f) == input_size(:small, NB) == 271
        @test f[1] == 1.0f0 && f[2] == 1.0f0 && f[3] ≈ 0.0004f0 && f[4] == 0.5f0
        @test length(scan_features(r, :large)) == NB
        X = zeros(Float32, 271, 2)
        scan_features!(X, r, :small, 2)
        @test X[:, 2] == f && all(iszero, X[:, 1])
    end

    @testset "networks" begin
        for tier in TIERS, est in (:e, :widths)
            net = EstimatorNet(est, tier, NB; rng=Xoshiro(2), y_mean=fill(1.0f0, output_size(est)),
                y_std=fill(2.0f0, output_size(est)))
            μ, σ = predict(net, rand(Xoshiro(3), NB) .* 8)
            @test length(μ) == length(σ) == output_size(est)
            @test all(>(0), σ)
            X = rand(Xoshiro(4), Float32, input_size(tier, NB), 3)
            M, S = predict(net, X)
            @test size(M) == size(S) == (output_size(est), 3)
        end
        dir = mktempdir()
        for tier in TIERS, est in ESTIMATES
            save_net(weights_file(dir, est, tier), EstimatorNet(est, tier, NB; rng=Xoshiro(5)))
        end
        nets = load_nets(dir)
        @test keys(nets) == TIERS && keys(nets.small) == ESTIMATES
        x = rand(Xoshiro(6), NB) .* 8
        @test predict(nets.large.ψe, x) == predict(EstimatorNet(:ψe, :large, NB; rng=Xoshiro(5)), x)
        @test keys(load_nets(dir; tiers=(:small,))) == (:small,)
    end

    @testset "cloud schedules" begin
        ctx = (; t=0.0, lane=nothing, v=0.0, yaw_rate=0.0)
        @test isempty(cloud_calls(NoCalls(), ctx))
        @test collect(cloud_calls(AllCalls(), ctx)) == collect(ESTIMATES)
        @test cloud_calls(FixedCalls(:sight, :e), ctx) == [:sight, :e]
        rr = RoundRobin()
        @test [only(cloud_calls(rr, ctx)) for _ in 1:8] == [ESTIMATES..., :e, :ψe]
        rr2 = RoundRobin([:e, :ψe, :sight]; per_scan=2)
        @test [cloud_calls(rr2, ctx) for _ in 1:3] == [[:e, :ψe], [:sight, :e], [:ψe, :sight]]
        @test isempty(cloud_calls(RoundRobin(Symbol[]), ctx))
    end

    @testset "selector" begin
        nets = (small=tier_nets(:small), large=tier_nets(:large))
        trusting = (e=Inf, ψe=Inf, κ_profile=Inf, widths=Inf, sight=Inf, clearance=Inf)
        cfg = SelectorConfig(max_σ=trusting, agree=Inf, remote_latency=0.05)
        te = TieredEstimator(nets; schedule=FixedCalls(:κ_profile, :sight), cfg)
        r = fill(3.0, NB)
        m0 = observe!(te, r, 0.0)
        @test Set(m.estimate for m in m0) == Set(ESTIMATES) && all(m -> m.tier === :small, m0)
        @test isempty(filter(m -> m.tier === :large, observe!(te, r, 0.025)))
        late = filter(m -> m.tier === :large, observe!(te, r, 0.05))   # the scan at t = 0 comes back
        @test Set(m.estimate for m in late) == Set([:κ_profile, :sight])
        @test all(m -> m.scan_time == 0.0, late)
        @test te.calls[:κ_profile] == 3 && te.calls[:e] == 0
        set_schedule!(te, NoCalls())
        observe!(te, r, 0.075)
        @test te.calls[:κ_profile] == 3

        # the large tier alone: no small results are used
        alone = TieredEstimator(nets; schedule=AllCalls(), cfg=SelectorConfig(max_σ=trusting, use_small=false))
        @test isempty(observe!(alone, r, 0.0)) && length(alone.pending) == length(ESTIMATES)
        @test length(observe!(alone, r, 0.025)) == length(ESTIMATES)
        # dropped calls still count as calls
        lossy = TieredEstimator(nets; schedule=AllCalls(), cfg=SelectorConfig(remote_drop=1.0))
        observe!(lossy, r, 0.0)
        @test isempty(lossy.pending) && lossy.calls[:e] == 1

        # results older than max_age are dropped
        old = TieredEstimator(nets; schedule=AllCalls(), cfg=SelectorConfig(max_σ=trusting, agree=Inf, remote_latency=0.2))
        observe!(old, r, 0.0)
        @test isempty(filter(m -> m.tier === :large, observe!(old, r, 0.2)))

        # unsure small results are dropped; without large nets, the cloud is never called
        unsure = TieredEstimator((small=tier_nets(:small),); schedule=AllCalls(),
            cfg=SelectorConfig(max_σ=(e=0.0, ψe=Inf, κ_profile=Inf, widths=Inf, sight=Inf, clearance=Inf)))
        m = observe!(unsure, r, 0.0)
        @test !any(x -> x.estimate === :e, m) && length(m) == 5
        @test isempty(unsure.pending) && unsure.calls[:e] == 0

        # a large result that disagrees with the small one is dropped
        strict = TieredEstimator(nets; schedule=AllCalls(),
            cfg=SelectorConfig(max_σ=trusting, agree=0.0, remote_latency=0.0))
        @test isempty(filter(m -> m.tier === :large, observe!(strict, r, 0.0)))
    end

    @testset "tracker: follows the measurements" begin
        _, lane = run_tracker(80; e=0.2, ψe=0.0, v=0.0)
        @test lane.status === :ok
        @test lane.e ≈ 0.2 atol = 1e-3
        @test abs(lane.ψe) < 1e-3
        @test lane.tiers.e === :small && lane.tiers.widths === :none
        @test lane.sight == 0 && isinf(lane.σ_sight)
    end

    @testset "tracker: a late result counts at its scan" begin
        κ(k, t) = [meas(:κ_profile, t, :small, fill(0.1, NP))]
        # A: the large result for scan 5 is there on time
        a(k, t) = k == 5 ? [κ(k, t); meas(:ψe, t, :large, [0.05])] : κ(k, t)
        # B: scan 5 first gets the small result, and the large one two scans later
        b(k, t) = k == 5 ? [κ(k, t); meas(:ψe, t, :small, [-0.3])] :
                  k == 7 ? [κ(k, t); meas(:ψe, 0.025 * 5, :large, [0.05])] : κ(k, t)
        ta, la = run_tracker(10; extra=a)
        tb, lb = run_tracker(10; extra=b)
        @test la.e ≈ lb.e && la.ψe ≈ lb.ψe && la.κ ≈ lb.κ
        @test ta.P ≈ tb.P
        @test lb.tiers.ψe === :small   # the latest result came from the small tier
        # the other way round: a small result never replaces a large one
        c(k, t) = k == 5 ? [κ(k, t); meas(:ψe, t, :large, [0.05])] :
                  k == 7 ? [κ(k, t); meas(:ψe, 0.025 * 5, :small, [-0.3])] : κ(k, t)
        _, lc = run_tracker(10; extra=c)
        @test lc.ψe ≈ la.ψe
    end

    @testset "tracker: profiles and distances move with the car" begin
        prof = collect(Float64, PROFILE_S)   # 0, 0.25, 0.5, …
        first(k, t) = k == 0 ? [meas(:κ_profile, t, :small, prof), meas(:sight, t, :large, [6.0]),
            meas(:clearance, t, :small, fill(5.0, NK))] : Measurement[]
        _, lane = run_tracker(5; v=2.5, extra=first)   # 4 scans × 2.5 m/s × 25 ms = 0.25 m driven
        # moved by one entry (the first is the filter's κ; the far end is no longer seen)
        @test lane.κ_profile[2:end-4] ≈ prof[3:end-3]
        @test isnan(lane.κ_profile[end])
        @test lane.sight ≈ 5.75 && lane.clearance ≈ fill(4.75, NK)
        @test lane.tiers.sight === :large
        # a late profile is shifted by the distance driven since its scan
        late(k, t) = k == 4 ? [meas(:widths, 0.0, :large, vcat(prof, prof))] : Measurement[]
        _, lane = run_tracker(5; v=2.5, extra=late)
        @test lane.w_left[1:end-4] ≈ prof[2:end-3] && lane.w_right[1:end-4] ≈ prof[2:end-3]
        # a late large profile replaces the small one for its scan instead of being blended on top
        both(k, t) = k == 0 ? [meas(:κ_profile, t, :small, fill(1.0, NP))] :
                     k == 2 ? [meas(:κ_profile, 0.0, :large, fill(0.2, NP))] : Measurement[]
        only(k, t) = k == 0 ? [meas(:κ_profile, t, :large, fill(0.2, NP))] : Measurement[]
        _, lb = run_tracker(4; v=0.0, extra=both)
        _, lo = run_tracker(4; v=0.0, extra=only)
        @test lb.κ_profile[2:end] ≈ lo.κ_profile[2:end] ≈ fill(0.2, NP - 1)
    end

    @testset "tracker: lost, and restart after outliers" begin
        tr = LaneTracker(0.257)
        @test track!(tr, Measurement[], 3.0, 0.0, 0.0).status === :lost   # nothing yet
        _, lane = run_tracker(3)
        @test lane.status === :ok
        tr, _ = run_tracker(10; v=1.0)
        local s
        for k in 10:21
            s = track!(tr, Measurement[], 1.0, 0.0, 0.025k)
        end
        @test s.status === :lost
        tr, _ = run_tracker(10; v=1.0, e=0.0)
        for k in 10:13   # the lane jumps 1 m: outliers until the filter restarts
            s = track!(tr, [meas(:e, 0.025k, :small, [1.0]), meas(:ψe, 0.025k, :small, [0.0])], 1.0, 0.0, 0.025k)
        end
        @test s.e ≈ 1.0 atol = 0.05
    end
end
