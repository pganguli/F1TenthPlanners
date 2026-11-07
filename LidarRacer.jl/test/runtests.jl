using LidarRacer
using Test

# NUC5, as in F1Tenth.jl's defaults
const CAR = (lf=0.130, lr=0.127, width=0.228, s_max=0.4189, steer_cmd_min=-0.3271441202,
    steer_cmd_max=0.2917771883, speed_cmd_max=14.375, decel_cmd=2.5, scan_rate=40.0, scan_range=10.0,
    mass=3.9)                                       # an extra field, ignored
const BEAMS = range(-3π / 4, 3π / 4; length=1081)  # a Hokuyo UST-10LX
const LIDAR_DIST = 0.143                            # LiDAR ahead of the center of gravity
const MAX_RANGE = 65.533                            # what the UST-10LX reads past its range

# A scan from the origin, looking along +x, of walls given as polylines;
# returns past the LiDAR's range read MAX_RANGE.
function scan(walls)
    map(BEAMS) do a
        dx, dy = cos(a), sin(a)
        best = Inf
        for w in walls, k in 1:length(w)-1
            (px, py), (qx, qy) = w[k], w[k+1]
            ex, ey = qx - px, qy - py
            den = dx * ey - dy * ex
            abs(den) < 1e-12 && continue
            t = (px * ey - py * ex) / den        # distance along the ray
            u = (px * dy - py * dx) / den        # position along the wall
            (t > 0 && 0 <= u <= 1) && (best = min(best, t))
        end
        best <= CAR.scan_range ? best : MAX_RANGE
    end
end

# a straight lane 2 m wide whose center passes `offset` to the left, `length` long
straight(offset=0.0; length=40.0) = [[(-10.0, offset + 1), (length, offset + 1)], [(-10.0, offset - 1), (length, offset - 1)]]
# a lane 2 m wide turning left on a circle of radius R (the car starts on it)
function left_turn(R)
    φ = range(-π / 2 - 0.5, π / 2; length=400)
    return [[(r * cos(a), R + r * sin(a)) for a in φ] for r in (R - 1, R + 1)]
end

# drive `n` scans in place at speed v (the scene doesn't move; enough for the filter to settle)
function settle(racer, ranges; v=3.0, n=10)
    local out
    for _ in 1:n
        out = control!(racer, ranges, v, 0.0, 1 / CAR.scan_rate)
    end
    return out
end

@testset "LidarRacer" begin
    @testset "CarParams" begin
        car = CarParams(CAR)
        @test car isa CarParams
        @test car.lf == CAR.lf && car.scan_range == CAR.scan_range
        @test CarParams(car) == car
        @test_throws ErrorException CarParams((lf=0.13,))
    end

    @testset "SpeedConfig from the car" begin
        cfg = SpeedConfig(CarParams(CAR); sweep=0.01875, latency=0.0075)
        @test cfg.v_max == CAR.speed_cmd_max
        @test cfg.a_brake == CAR.decel_cmd
        @test cfg.r_min ≈ (CAR.lf + CAR.lr) / tan(CAR.steer_cmd_max)   # the smaller steering limit
        @test cfg.delay ≈ 0.01875 + 0.0075 + 1 / CAR.scan_rate
        @test SpeedConfig(CarParams(CAR); sweep=0.0, latency=0.0, v_max=4.0, a_lat=3.0).a_lat == 3.0
    end

    @testset "Racer timing" begin
        r = Racer(CAR, BEAMS, LIDAR_DIST)
        @test r.sweep ≈ 0.75 / CAR.scan_rate   # 270° of a 360° sweep
        @test r.latency == 0.0075
        @test r.speed.v_max == CAR.speed_cmd_max
        @test Racer(CAR, BEAMS, LIDAR_DIST; v_max=3.0, latency=0.02).speed.v_max == 3.0
    end

    @testset "straight lane, centered" begin
        r = Racer(CAR, BEAMS, LIDAR_DIST)
        steer, speed, lane = settle(r, scan(straight()))
        @test lane.walls === :both
        @test abs(steer) < 0.02
        @test abs(lane.e) < 0.05
        @test r.speed.v_min < speed <= r.speed.v_max
        @test maximum(abs, r.path_y) < 0.1   # the planned line stays in the middle
    end

    @testset "steers back to the line" begin
        steer_l, _, lane_l = settle(Racer(CAR, BEAMS, LIDAR_DIST), scan(straight(0.4)))
        steer_r, _, lane_r = settle(Racer(CAR, BEAMS, LIDAR_DIST), scan(straight(-0.4)))
        @test lane_l.e > 0 && steer_l > 0    # line to the left: steer left
        @test lane_r.e < 0 && steer_r < 0
    end

    @testset "slower in a turn, and for a wall ahead" begin
        _, v_straight, _ = settle(Racer(CAR, BEAMS, LIDAR_DIST), scan(straight()))
        r = Racer(CAR, BEAMS, LIDAR_DIST)
        steer, v_turn, lane = settle(r, scan(left_turn(3.0)))
        @test steer > 0
        @test lane.κ > 0
        @test v_turn < v_straight
        # turning with the lane, the outer wall straight ahead (about 2.5 m
        # away) is not in the way
        @test free_distance(r, lane, 3.0, 1.0) > 5.0
        # not turning yet, the line still curves away from it
        @test free_distance(r, lane, 3.0, 0.0) > LidarRacer.clearance(r.perceive) + 0.3

        walls = [straight(; length=2.0); [[(2.0, -1.0), (2.0, 1.0)]]]   # a dead end 2 m ahead
        r = Racer(CAR, BEAMS, LIDAR_DIST)
        _, v_wall, lane = settle(r, scan(walls))
        # no faster than it can brake from, after the delay, to v_min `margin` short of the wall
        cfg = r.speed
        d = free_distance(r, lane, 3.0, 0.0) - cfg.margin - cfg.delay * 3.0
        @test v_wall <= sqrt(cfg.v_min^2 + 2cfg.a_brake * d) + 1e-9
        @test v_wall < v_turn
    end
end
