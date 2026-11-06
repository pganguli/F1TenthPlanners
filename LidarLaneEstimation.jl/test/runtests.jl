using LidarLaneEstimation
using StanleyController: wrap_angle
using Test

const ANGLES = range(-2.35, 2.35; length=1080)
const X_FA = 0.16          # front axle ahead of the LiDAR
const MAX_RANGE = 30.0

# A scan from the origin, looking along +x, of walls given as polylines.
function scan(walls; angles=ANGLES)
    map(angles) do a
        dx, dy = cos(a), sin(a)
        best = MAX_RANGE
        for w in walls, k in 1:length(w)-1
            (px, py), (qx, qy) = w[k], w[k+1]
            ex, ey = qx - px, qy - py
            den = dx * ey - dy * ex
            abs(den) < 1e-12 && continue
            t = (px * ey - py * ex) / den        # distance along the ray
            u = (px * dy - py * dx) / den        # position along the wall
            (t > 0 && 0 <= u <= 1) && (best = min(best, t))
        end
        best
    end
end

line(x0, y0, h; from=-10.0, to=40.0) = [(x0 + s * cos(h), y0 + s * sin(h)) for s in (from, to)]
arc(cx, cy, r, φ0, φ1; n=400) = [(cx + r * cos(φ), cy + r * sin(φ)) for φ in range(φ0, φ1; length=n)]

# Walls of a straight lane 2 m wide whose center passes `offset` to the left
# of the front axle, running at heading `h`.
function straight_lane(; offset=0.0, h=0.0)
    cx, cy = X_FA - offset * sin(h), offset * cos(h)
    return [line(cx - sin(h), cy + cos(h), h), line(cx + sin(h), cy - cos(h), h)]
end

@testset "LidarLaneEstimation" begin
    @testset "constructors" begin
        @test length(LanePerception(1080, 4.7, X_FA).angles) == 1080
        @test LanePerception(collect(ANGLES), X_FA).angles ≈ LanePerception(1080, 4.7, X_FA).angles
        @test_throws ArgumentError LanePerception([0.5, 0.0], X_FA)
        @test_throws DimensionMismatch estimate_lane!(LanePerception(1080, 4.7, X_FA), zeros(10))
    end

    @testset "straight lane" begin
        lane = estimate_lane!(LanePerception(ANGLES, X_FA), scan(straight_lane()))
        @test lane.walls === :both
        @test abs(lane.e) < 0.02 && abs(lane.ψe) < 0.02 && abs(lane.κ) < 0.02
        @test lane.width ≈ 2.0 atol = 0.05
        @test lane.clearance ≈ MAX_RANGE - X_FA atol = 0.01   # nothing ahead
        # lane center to the left, lane pointing left
        lane = estimate_lane!(LanePerception(ANGLES, X_FA), scan(straight_lane(offset=0.3)))
        @test lane.e ≈ 0.3 atol = 0.02
        lane = estimate_lane!(LanePerception(ANGLES, X_FA), scan(straight_lane(offset=-0.3)))
        @test lane.e ≈ -0.3 atol = 0.02
        lane = estimate_lane!(LanePerception(ANGLES, X_FA), scan(straight_lane(h=0.15)))
        @test lane.ψe ≈ 0.15 atol = 0.02
        @test abs(lane.e) < 0.03
    end

    @testset "curves" begin
        R = 5.0   # center line through the front axle, turning left around (X_FA, R)
        left = [arc(X_FA, R, R - 1, -π / 2 - 0.5, π / 2), arc(X_FA, R, R + 1, -π / 2 - 0.5, π / 2)]
        lane = estimate_lane!(LanePerception(ANGLES, X_FA), scan(left))
        @test lane.walls === :both
        @test lane.κ ≈ 1 / R rtol = 0.1
        @test abs(lane.e) < 0.03 && abs(lane.ψe) < 0.03
        # exact for the first 2 m; farther out, where one wall leaves the
        # LiDAR's range before the other, the curve reads up to ~1.7× too
        # sharp, which only makes the speed choice cautious
        @test all(isapprox.(lane.κ_profile[1:9], 1 / R; rtol=0.05))
        @test 1 / R <= lane.κ_ahead <= 1.8 / R
        # the same turning right
        right = [[(x, -y) for (x, y) in w] for w in left]
        lane = estimate_lane!(LanePerception(ANGLES, X_FA), scan(right))
        @test lane.κ ≈ -1 / R rtol = 0.1
        # the profile ahead sees a turn the axle hasn't reached
        bend = [vcat(line(0.0, 1.0, 0.0; from=-10.0, to=3.0)[1:1], arc(3.0, 3.0, 2.0, -π / 2, 0.0)),
                vcat(line(0.0, -1.0, 0.0; from=-10.0, to=3.0)[1:1], arc(3.0, 3.0, 4.0, -π / 2, 0.0))]
        lane = estimate_lane!(LanePerception(ANGLES, X_FA), scan(bend))
        @test abs(lane.κ) < 0.1 && lane.κ_ahead > 0.25
    end

    @testset "clearance" begin
        walls = vcat(straight_lane(), [[(5.0, -2.0), (5.0, 2.0)]])
        @test estimate_lane!(LanePerception(ANGLES, X_FA), scan(walls)).clearance ≈ 5.0 - X_FA atol = 0.01
    end

    @testset "one wall" begin
        p = LanePerception(ANGLES, X_FA)
        estimate_lane!(p, scan(straight_lane(offset=0.2)))      # learns the width
        lane = estimate_lane!(p, scan(straight_lane(offset=0.2)[2:2]))
        @test lane.walls === :right
        @test lane.e ≈ 0.2 atol = 0.03
        lane = estimate_lane!(p, scan(straight_lane(offset=0.2)[1:1]))
        @test lane.walls === :left
        @test lane.e ≈ 0.2 atol = 0.03
        @test estimate_lane!(p, fill(MAX_RANGE, length(ANGLES))).walls === :none
    end

    estimate(e, ψe; κ=0.0) = LaneEstimate(e, ψe, κ, abs(κ), 2.0, :both, 10.0, fill(κ, 17))

    @testset "filter: noise" begin
        f = LaneFilter(0.33)
        raw, out = Float64[], Float64[]
        for k in 1:2000
            z = 0.05 * sin(1.7k)   # zero-mean jitter, about as large as a real scan's
            push!(raw, z)
            push!(out, track!(f, estimate(0.0, z), 5.0, 0.0, 0.01).ψe)
        end
        rms(x) = sqrt(sum(abs2, x) / length(x))
        @test rms(out[100:end]) < rms(raw[100:end]) / 3
    end

    @testset "filter: turning is predicted" begin
        # On a straight lane the car turns left at r: the lane's heading error
        # falls at r and the lane drifts right. Measurements always lag one
        # scan behind; the filter's estimate should not lag further.
        f = LaneFilter(0.33)
        v, r, dt = 5.0, 0.5, 0.01
        e, ψe = 0.0, 0.0
        for _ in 1:100
            out = track!(f, estimate(e, ψe), v, r, dt)
            ψe -= r * dt
            e += (v * sin(ψe) - 0.33 * r * cos(ψe)) * dt
            @test abs(out.ψe - (ψe + r * dt)) < 0.01
        end
    end

    @testset "filter: outliers and restarts" begin
        f = LaneFilter(0.33)
        for _ in 1:50
            track!(f, estimate(0.0, 0.0), 3.0, 0.0, 0.01)
        end
        @test abs(track!(f, estimate(0.0, 1.0), 3.0, 0.0, 0.01).ψe) < 0.01   # one outlier: ignored
        track!(f, estimate(0.0, 0.0), 3.0, 0.0, 0.01)
        outs = [track!(f, estimate(0.5, 1.0), 3.0, 0.0, 0.01) for _ in 1:3]   # the lane really moved
        @test abs(outs[2].ψe) < 0.01 && outs[3].ψe ≈ 1.0 && outs[3].e ≈ 0.5
        # with no lane in view the prediction carries on
        none = LaneEstimate(0.0, 0.0, 0.0, 0.0, 2.0, :none, Inf, fill(NaN, 17))
        @test track!(f, none, 3.0, 0.0, 0.01).ψe ≈ 1.0 atol = 0.01
        # the curvature ahead is averaged, but a sharp turn is taken at once
        f = LaneFilter(0.33)
        for _ in 1:20
            track!(f, estimate(0.0, 0.0; κ=0.0), 3.0, 0.0, 0.01)
        end
        @test track!(f, estimate(0.0, 0.0; κ=0.1), 3.0, 0.0, 0.01).κ_profile[5] ≈ 0.025 atol = 0.002
        @test track!(f, estimate(0.0, 0.0; κ=1.0), 3.0, 0.0, 0.01).κ_ahead ≈ 1.0
    end

    @testset "target_speed" begin
        cfg = SpeedConfig(cruise=5.0, v_min=1.0, a_lat=4.0, a_brake=4.0, margin=0.5)
        lane(κ, clearance) = LaneEstimate(0.0, 0.0, κ, κ, 2.0, :both, clearance, Float64[])
        @test target_speed(cfg, lane(0.0, Inf)) == 5.0
        @test target_speed(cfg, lane(1.0, Inf)) ≈ 2.0                  # √(a_lat / κ)
        @test target_speed(cfg, lane(0.0, 0.5 + 1.5)) ≈ sqrt(1 + 2 * 4 * 1.5)
        @test target_speed(cfg, lane(100.0, 0.0)) == 1.0
    end
end
