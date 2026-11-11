using LidarLaneNets
using LidarRacer
using Random
using Test

# NUC5, as in F1Tenth.jl's defaults
const CAR = (lf=0.130, lr=0.127, width=0.228, s_max=0.4189, steer_cmd_min=-0.3271441202,
    steer_cmd_max=0.2917771883, speed_cmd_max=14.375, decel_cmd=2.5, scan_rate=40.0, scan_range=10.0,
    mass=3.9)                                       # an extra field, ignored
const BEAMS = range(-3π / 4, 3π / 4; length=1081)  # a Hokuyo UST-10LX
const NP = length(PROFILE_S)
const NK = length(CLEARANCE_K)

# A lane 2 m wide, its center `e` to the left of the front axle, at `ψe`,
# bending by κ, seen `sight` meters ahead, `free` meters clear on every arc.
lane(; e=0.0, ψe=0.0, κ=0.0, sight=SIGHT_MAX, free=CLEARANCE_MAX) =
    LaneState(lane_labels(; e, ψe, κ_profile=fill(κ, NP), w_left=fill(1.0, NP), w_right=fill(1.0, NP), sight,
        clearance=fill(free, NK)))

# untrained networks, for running the whole controller
tier_nets(t) = NamedTuple{ESTIMATES}(Tuple(EstimatorNet(est, t, length(BEAMS); rng=Xoshiro(1)) for est in ESTIMATES))
const NETS = (small=tier_nets(:small), large=tier_nets(:large))

planner(; kwargs...) = Planner(CarParams(CAR); sweep=0.75 / CAR.scan_rate, kwargs...)

@testset "LidarRacer" begin
    @testset "CarParams" begin
        car = CarParams(CAR)
        @test car isa CarParams
        @test car.lf == CAR.lf && car.scan_range == CAR.scan_range
        @test CarParams(car) == car
        @test_throws Exception CarParams((lf=0.13,))
    end

    @testset "SpeedConfig from the car" begin
        cfg = SpeedConfig(CarParams(CAR); sweep=0.01875, latency=0.0075)
        @test cfg.v_max == CAR.speed_cmd_max
        @test cfg.a_brake == CAR.decel_cmd
        @test cfg.r_min ≈ (CAR.lf + CAR.lr) / tan(CAR.steer_cmd_max)   # the smaller steering limit
        @test cfg.delay ≈ 0.01875 + 0.0075 + 1 / CAR.scan_rate
        @test SpeedConfig(CarParams(CAR); sweep=0.0, latency=0.0, v_max=4.0, a_lat=3.0).a_lat == 3.0
        @test planner(a_lat=3.0).speed.a_lat == 3.0
    end

    @testset "Racer timing" begin
        r = Racer(CAR, BEAMS, NETS)
        @test r.sweep ≈ 0.75 / CAR.scan_rate   # 270° of a 360° sweep
        @test r.latency == 0.0075
        @test r.planner.speed.v_max == CAR.speed_cmd_max
        @test Racer(CAR, BEAMS, NETS; v_max=3.0, latency=0.02).planner.speed.v_max == 3.0
    end

    @testset "lane center from the estimates" begin
        x, y, h, wl, wr = lane_center(lane(e=0.2), 4.0)
        @test x[2] ≈ 0 && y[2] ≈ 0.2          # beside the front axle, to the left
        @test x[1] ≈ -0.5 && all(≈(0.2), y)   # half a meter behind, then straight on
        @test x[end] ≈ 4.0 && length(x) == 2 + 16
        @test all(==(1.0), wl) && all(==(1.0), wr)
        x, y, h, _, _ = lane_center(lane(κ=0.5), 3.0)
        @test h[end] ≈ 0.5 * 3.0                 # turned by κ times the length
        @test hypot(x[end], y[end] - 2) ≈ 2 atol = 1e-3   # on the circle of radius 2
        # stops at the first unknown point
        s = lane()
        s.κ_profile[6] = NaN
        @test length(lane_center(s, 8.0)[1]) == 2 + 4
    end

    @testset "straight lane, centered" begin
        steer, speed, plan = drive(planner(), lane(), 3.0, 0.0)
        @test abs(steer) < 0.02
        @test abs(plan.e) < 0.02 && abs(plan.ψe) < 0.02
        @test maximum(abs, plan.y) < 0.05   # the planned line stays in the middle
        @test 1.0 < speed <= CAR.speed_cmd_max
        @test isinf(plan.free)
    end

    @testset "steers back to the line" begin
        steer_l, _, plan_l = drive(planner(), lane(e=0.3), 3.0, 0.0)
        steer_r, _, plan_r = drive(planner(), lane(e=-0.3), 3.0, 0.0)
        @test plan_l.e > 0 && steer_l > 0    # line to the left: steer left
        @test plan_r.e < 0 && steer_r < 0
        steer, _, _ = drive(planner(), lane(ψe=0.2), 3.0, 0.0)
        @test steer > 0                      # line pointing left: steer left
    end

    @testset "slower in a turn, for a wall ahead, and when unsure" begin
        _, v_straight, _ = drive(planner(), lane(), 3.0, 0.0)
        steer, v_turn, plan = drive(planner(), lane(κ=0.6), 3.0, 0.0)
        @test steer > 0 && plan.κ > 0
        @test v_turn < v_straight
        # the planned line bends less than the lane (it cuts the corner)
        @test 0 < plan.κ < 0.6

        p = planner()
        _, v_wall, plan = drive(p, lane(free=2.0), 3.0, 0.0)
        cfg = p.speed
        d = plan.free - cfg.margin - cfg.delay * 3.0
        @test plan.free ≈ 2.0
        @test v_wall <= sqrt(cfg.v_min^2 + 2cfg.a_brake * d) + 1e-9
        @test v_wall < v_straight

        # sight is taken k_σ standard deviations short
        unsure = lane(sight=6.0)
        unsure = LaneState(unsure.e, unsure.ψe, unsure.κ, unsure.κ_profile, unsure.w_left, unsure.w_right, 6.0, 2.0,
            unsure.clearance, unsure.σ_clearance, unsure.tiers, :ok)
        _, v_sure, plan_sure = drive(p, lane(sight=6.0), 3.0, 0.0)
        _, v_unsure, plan_unsure = drive(p, unsure, 3.0, 0.0)
        @test plan_unsure.sight ≈ 4.0 && plan_sure.sight ≈ 6.0
        @test v_unsure < v_sure
    end

    @testset "the whole controller" begin
        ranges = [abs(sin(a)) < 1e-3 || 1 / abs(sin(a)) > 10 ? 65.533 : 1 / abs(sin(a)) for a in BEAMS]
        trusting = (e=Inf, ψe=Inf, κ_profile=Inf, widths=Inf, sight=Inf, clearance=Inf)
        r = Racer(CAR, BEAMS, NETS; selector=SelectorConfig(max_σ=trusting))
        local steer, speed, s
        for _ in 1:5
            steer, speed, s = control!(r, ranges, 3.0, 0.0, 0.025)
        end
        @test s.status === :ok && isfinite(steer) && speed >= 0
        @test r.plan !== nothing
        @test r.t ≈ 0.125
        # nothing it trusts: the lane is never found, so it stops
        never = (e=0.0, ψe=0.0, κ_profile=0.0, widths=0.0, sight=0.0, clearance=0.0)
        r = Racer(CAR, BEAMS, NETS; selector=SelectorConfig(max_σ=never))
        steer, speed, s = control!(r, ranges, 3.0, 0.0, 0.025)
        @test s.status === :lost && speed == 0 && steer == 0
    end
end
