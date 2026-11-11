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
        @test file_name(:ψe) == "psi_e" && UNITS.κ_profile == "1/m"
        @test_throws ArgumentError file_name(:speed)
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
        f = scan_features(r, 4)
        @test length(f) == input_size(:small, NB) == 271
        @test f[1] == 1.0f0 && f[2] == 1.0f0 && f[3] ≈ 0.0004f0 && f[4] == 0.5f0
        @test length(scan_features(r, 1)) == NB
        X = zeros(Float32, 271, 2)
        scan_features!(X, r, 4, 2)
        @test X[:, 2] == f && all(iszero, X[:, 1])
        @test scan_features(hcat(fill(2.0, NB), r), 4) == hcat(fill(0.2f0, 271), f)
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
        # a large network starts out exactly as its small one
        small = EstimatorNet(:widths, :small, NB; rng=Xoshiro(7), y_mean=fill(0.5f0, output_size(:widths)),
            y_std=fill(3.0f0, output_size(:widths)))
        big = large_from_small(small; rng=Xoshiro(8))
        x = rand(Xoshiro(9), NB) .* 8
        @test big.tier === :large && big.y_std == small.y_std
        @test predict(big, x) == predict(small, x)
        @test_throws ArgumentError large_from_small(big)
        # other small shapes: the weak one takes every 8th beam, and survives saving
        weak = EstimatorNet(:e, :small, NB; spec=SMALL_WEAK_SPEC, rng=Xoshiro(10))
        @test input_stride(weak) == 8 && input_size(weak) == 136
        wl = large_from_small(weak; rng=Xoshiro(11))
        @test input_stride(wl) == 1 && wl.spec == SMALL_WEAK_SPEC && predict(wl, x) == predict(weak, x)
        wl2 = load_net(save_net(tempname() * ".jld2", wl))
        @test wl2.spec == SMALL_WEAK_SPEC && predict(wl2, x) == predict(wl, x)

        dir = mktempdir()
        for name in NETWORKS, est in ESTIMATES
            save_net(weights_file(dir, est, name),
                EstimatorNet(est, network_tier(name), NB; spec=network_spec(name), rng=Xoshiro(5)))
        end
        nets = load_nets(dir)   # small_weak on the car, large in the cloud
        @test nets.small.e.spec == SMALL_WEAK_SPEC && nets.large.e.spec == SMALL_STRONG_SPEC
        @test load_nets(dir; small=:small_strong).small.sight.spec == SMALL_STRONG_SPEC
        @test_throws ArgumentError load_nets(dir; small=:large)
        @test_throws ArgumentError network_spec(:medium)
        @test keys(nets) == TIERS && keys(nets.small) == ESTIMATES
        x = rand(Xoshiro(6), NB) .* 8
        @test predict(nets.large.ψe, x) == predict(EstimatorNet(:ψe, :large, NB; rng=Xoshiro(5)), x)
        @test keys(load_nets(dir; large=nothing)) == (:small,)
        @test basename(weights_file(dir, :ψe, :small_weak)) == "small_weak_psi_e.jld2"
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
        nets = (small=tier_nets(:small), large=tier_nets(:large; rng=Xoshiro(2)))
        trusting = (e=Inf, ψe=Inf, κ_profile=Inf, widths=Inf, sight=Inf, clearance=Inf)
        cfg = SelectorConfig(max_σ=trusting, remote_latency=0.05)
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
        old = TieredEstimator(nets; schedule=AllCalls(), cfg=SelectorConfig(max_σ=trusting, remote_latency=0.2))
        observe!(old, r, 0.0)
        @test isempty(filter(m -> m.tier === :large, observe!(old, r, 0.2)))

        # unsure small results are dropped; without large nets, the cloud is never called
        unsure = TieredEstimator((small=tier_nets(:small),); schedule=AllCalls(),
            cfg=SelectorConfig(max_σ=(e=0.0, ψe=Inf, κ_profile=Inf, widths=Inf, sight=Inf, clearance=Inf)))
        m = observe!(unsure, r, 0.0)
        @test !any(x -> x.estimate === :e, m) && length(m) == 5
        @test isempty(unsure.pending) && unsure.calls[:e] == 0

        # a large result that disagrees with the small one is kept (the tracker learns the bias from it)
        both = TieredEstimator(nets; schedule=AllCalls(), cfg=SelectorConfig(max_σ=trusting, remote_latency=0.0))
        @test length(filter(m -> m.tier === :large, observe!(both, r, 0.0))) == length(ESTIMATES)
    end

    @testset "tracker: follows the measurements" begin
        _, lane = run_tracker(80; e=0.2, ψe=0.0, v=0.0)
        @test lane.status === :ok
        @test lane.e ≈ 0.2 atol = 0.01
        @test abs(lane.ψe) < 0.01
        @test lane.tiers.e === :small && lane.tiers.widths === :none
        @test lane.sight == 0 && isinf(lane.σ_sight)
    end

    # The car's network is biased (e 0.3 and sight 6 m, where the truth is 0
    # and 5 m); the cloud's isn't, and its results come `lag` scans late.
    biased = TrackerConfig(noise=merge(default_noise(),
        (e=EstimateNoise(persist=0.9, τ=2.0, σ_bias=0.3), sight=EstimateNoise(persist=0.9, τ=2.0, σ_bias=1.0))))
    function biased_run(lag; cloud=true, n=60, cfg=biased)
        tr = LaneTracker(0.257; cfg)
        local lane
        for k in 0:n-1
            t = 0.025k
            m = [meas(:e, t, :small, [0.3]), meas(:ψe, t, :small, [0.0]), meas(:sight, t, :small, [6.0])]
            if cloud && k >= lag
                tc = 0.025(k - lag)
                push!(m, meas(:e, tc, :large, [0.0]), meas(:sight, tc, :large, [5.0]))
            end
            lane = track!(tr, m, 0.0, 0.0, t)
        end
        return tr, lane
    end

    @testset "tracker: the cloud corrects the car's bias" begin
        _, alone = biased_run(0; cloud=false)
        @test alone.e ≈ 0.3 atol = 0.02
        @test alone.sight ≈ 6.0 atol = 0.05
        errs = [begin
            _, l = biased_run(lag)
            (abs(l.e), abs(l.sight - 5))
        end for lag in (0, 1, 4, 8)]
        @test errs[1][1] < 0.05 && errs[1][2] < 0.15   # on time: close to the cloud
        # later results correct less, but always more than none
        @test issorted(first.(errs)) && issorted(last.(errs))
        @test all(e -> e[1] < 0.3 - 0.05 && e[2] < 1.0 - 0.05, errs)
        # the cloud's sight is used even though the car's results are newer
        _, l = biased_run(1)
        @test l.tiers.sight === :small && abs(l.sight - 5) < abs(alone.sight - 5) / 2
    end

    @testset "tracker: the cloud corrects a bias far beyond its usual size" begin
        # the car's bias is 0.3, where its typical size is 0.01: the difference shows it anyway
        small_bias = TrackerConfig(noise=merge(default_noise(),
            (e=EstimateNoise(persist=0.9, τ=2.0, σ_bias=0.01, ν_small=3.0, ν_large=3.0),
                sight=EstimateNoise(persist=0.9, τ=2.0, σ_bias=0.05, ν_small=3.0, ν_large=3.0))))
        _, l = biased_run(1; cfg=small_bias)
        @test abs(l.e) < 0.05
        @test abs(l.sight - 5) < 0.15
    end

    @testset "tracker: a late result gives the same state as on time" begin
        κ(k, t) = [meas(:κ_profile, t, :small, fill(0.1, NP))]
        on_time(k, t) = k == 5 ? [κ(k, t); meas(:ψe, t, :large, [0.05])] : κ(k, t)
        late(k, t) = k == 7 ? [κ(k, t); meas(:ψe, 0.025 * 5, :large, [0.05])] : κ(k, t)
        ta, la = run_tracker(10; extra=on_time)
        tb, lb = run_tracker(10; extra=late)
        @test la.e ≈ lb.e && la.ψe ≈ lb.ψe && la.κ ≈ lb.κ
        @test ta.state.lane.P ≈ tb.state.lane.P
        @test la.κ_profile[2:end] ≈ lb.κ_profile[2:end]
    end

    @testset "tracker: profiles and distances move with the car" begin
        prof = collect(Float64, PROFILE_S)   # 0, 0.25, 0.5, …
        first(k, t) = k == 0 ? [meas(:κ_profile, t, :small, prof), meas(:sight, t, :large, [6.0]),
            meas(:clearance, t, :small, fill(5.0, NK))] : Measurement[]
        _, lane = run_tracker(5; v=2.5, extra=first)   # 4 scans × 2.5 m/s × 25 ms = 0.25 m driven
        # moved by one entry (the first is the filter's κ; the far end is no longer seen)
        @test lane.κ_profile[2:end-4] ≈ prof[3:end-3]
        @test isnan(lane.κ_profile[end])
        # sight and clearance are from the car: they hold, growing uncertain
        @test lane.sight ≈ 6.0 && lane.clearance ≈ fill(5.0, NK)
        @test lane.σ_sight > 0.05   # more than the measurement's own
        @test lane.tiers.sight === :large
    end

    @testset "tracker: robust to a wild measurement" begin
        wild(k, t) = k == 30 ? [meas(:e, t, :small, [3.0])] : Measurement[]
        heavy = TrackerConfig(noise=merge(default_noise(), (e=EstimateNoise(σ_bias=0.005, ν_small=3.0),)))
        gaussian = TrackerConfig(noise=heavy.noise, robust=false)
        function jumped(cfg)
            tr = LaneTracker(0.257; cfg)
            local lane
            for k in 0:30
                t = 0.025k
                m = k == 30 ? [meas(:ψe, t, :small, [0.0]); wild(k, t)] :
                    [meas(:e, t, :small, [0.0]), meas(:ψe, t, :small, [0.0])]
                lane = track!(tr, m, 1.0, 0.0, t)
            end
            return abs(lane.e)
        end
        @test jumped(heavy) < jumped(gaussian) / 5
    end

    @testset "tracker: starts from late measurements alone" begin
        # e and ψe only ever come one scan late (a car on the cloud alone)
        tr = LaneTracker(0.257)
        local lane
        for k in 0:12
            t = 0.025k
            m = k == 0 ? Measurement[] : [meas(:e, t - 0.025, :large, [0.2]), meas(:ψe, t - 0.025, :large, [0.0])]
            lane = track!(tr, m, 0.0, 0.0, t)
            k == 0 && @test lane.status === :lost
            k == 1 && @test lane.status === :predicting   # started, from the scan before
        end
        @test lane.status === :predicting
        @test lane.e ≈ 0.2 atol = 1e-3
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

        # the same from the cloud alone, its results two scans late
        tr = LaneTracker(0.257)
        cloud(k, e) = [meas(:e, 0.025k, :large, [e]), meas(:ψe, 0.025k, :large, [0.0])]
        for k in 0:14
            s = track!(tr, k >= 2 ? cloud(k - 2, k - 2 < 10 ? 0.0 : 1.0) : Measurement[], 1.0, 0.0, 0.025k)
        end
        @test s.status !== :lost
        @test s.e ≈ 1.0 atol = 0.05
    end
end
