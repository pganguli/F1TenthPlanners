using LidarLaneEstimation
using Test

const ANGLES = range(-3π / 4, 3π / 4; length=1081)   # a Hokuyo UST-10LX
const X_REF = 0.13         # reference point (here the front axle) ahead of the LiDAR
const WHEELBASE = 0.257    # the reference point ahead of the rear axle
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
    cx, cy = X_REF - offset * sin(h), offset * cos(h)
    return [line(cx - sin(h), cy + cos(h), h), line(cx + sin(h), cy - cos(h), h)]
end

@testset "LidarLaneEstimation" begin
    @testset "constructors" begin
        @test length(LanePerception(1081, 3π / 2, X_REF).angles) == 1081
        @test LanePerception(collect(ANGLES), X_REF).angles ≈ LanePerception(1081, 3π / 2, X_REF).angles
        @test_throws ArgumentError LanePerception([0.5, 0.0], X_REF)
        @test_throws DimensionMismatch estimate_lane!(LanePerception(1081, 3π / 2, X_REF), zeros(10))
    end

    @testset "straight lane" begin
        lane = estimate_lane!(LanePerception(ANGLES, X_REF), scan(straight_lane()))
        @test lane.walls === :both
        @test abs(lane.e) < 0.02 && abs(lane.ψe) < 0.02 && abs(lane.κ) < 0.02
        @test lane.width ≈ 2.0 atol = 0.05
        p = LanePerception(ANGLES, X_REF)
        detect_lane!(p, scan(straight_lane()))
        @test clearance(p) ≈ MAX_RANGE - X_REF atol = 0.01   # nothing ahead
        # the walls are seen out to r_max, so the center line reaches at most
        # √(r_max² - 1) ahead of the LiDAR (less the ends lost to smoothing)
        @test 4.5 < lane.sight < sqrt(PerceptionConfig().r_max^2 - 1) - X_REF
        # lane center to the left, lane pointing left
        lane = estimate_lane!(LanePerception(ANGLES, X_REF), scan(straight_lane(offset=0.3)))
        @test lane.e ≈ 0.3 atol = 0.02
        lane = estimate_lane!(LanePerception(ANGLES, X_REF), scan(straight_lane(offset=-0.3)))
        @test lane.e ≈ -0.3 atol = 0.02
        lane = estimate_lane!(LanePerception(ANGLES, X_REF), scan(straight_lane(h=0.15)))
        @test lane.ψe ≈ 0.15 atol = 0.02
        @test abs(lane.e) < 0.03
        # the same lane seen from a reference point a wheelbase farther back (the
        # rear axle): the center line, which passes through the front axle
        # heading left, is to the right of it
        lane = estimate_lane!(LanePerception(ANGLES, X_REF - WHEELBASE), scan(straight_lane(h=0.15)))
        @test lane.e ≈ -WHEELBASE * sin(0.15) atol = 0.02
        @test lane.ψe ≈ 0.15 atol = 0.02
    end

    @testset "curves" begin
        R = 5.0   # center line through the front axle, turning left around (X_REF, R)
        left = [arc(X_REF, R, R - 1, -π / 2 - 0.5, π / 2), arc(X_REF, R, R + 1, -π / 2 - 0.5, π / 2)]
        lane = estimate_lane!(LanePerception(ANGLES, X_REF), scan(left))
        @test lane.walls === :both
        @test lane.κ ≈ 1 / R rtol = 0.1
        @test abs(lane.e) < 0.03 && abs(lane.ψe) < 0.03
        # exact for the first 2 m; farther out, where one wall leaves the
        # LiDAR's range before the other, the curve reads up to ~1.7× too
        # sharp, which only makes the speed choice cautious
        @test all(isapprox.(lane.κ_profile[1:9], 1 / R; rtol=0.05))
        @test 1 / R <= lane.κ_ahead <= 1.8 / R
        right = [[(x, -y) for (x, y) in w] for w in left]
        lane = estimate_lane!(LanePerception(ANGLES, X_REF), scan(right))
        @test lane.κ ≈ -1 / R rtol = 0.1
        # the profile ahead sees a turn the axle hasn't reached
        bend = [vcat(line(0.0, 1.0, 0.0; from=-10.0, to=3.0)[1:1], arc(3.0, 3.0, 2.0, -π / 2, 0.0)),
                vcat(line(0.0, -1.0, 0.0; from=-10.0, to=3.0)[1:1], arc(3.0, 3.0, 4.0, -π / 2, 0.0))]
        lane = estimate_lane!(LanePerception(ANGLES, X_REF), scan(bend))
        @test abs(lane.κ) < 0.1 && lane.κ_ahead > 0.25
    end

    @testset "clearance" begin
        p = LanePerception(ANGLES, X_REF)
        walls = vcat(straight_lane(), [[(5.0, -2.0), (5.0, 2.0)]])
        detect_lane!(p, scan(walls))
        @test clearance(p) ≈ 5.0 - X_REF atol = 0.01
        @test clearance(p, 0.0) == clearance(p)
        # along a curve: a lane turning left around (X_REF, R), closed by a wall
        # across it 1 rad of turn ahead. Straight ahead, the outer wall is
        # nearer; along the arc, the wall across the lane is R × 1 rad away.
        R, φ = 5.0, 1.0
        across = [(X_REF + r * sin(φ), R - r * cos(φ)) for r in (R - 1.1, R + 1.1)]
        bend = [arc(X_REF, R, R - 1, -π / 2 - 0.5, π / 2), arc(X_REF, R, R + 1, -π / 2 - 0.5, π / 2), across]
        detect_lane!(p, scan(bend))
        straight = clearance(p)
        hw = PerceptionConfig().car_halfwidth   # where the corridor's right edge meets the outer wall
        @test straight ≈ sqrt((R + 1)^2 - (R + hw)^2) atol = 0.02
        @test clearance(p, 1 / R) ≈ R * φ atol = 0.02
        # turning the wrong way: the outer wall is in the way sooner
        @test clearance(p, -1 / R) < straight
        detect_lane!(p, scan([[(x, -y) for (x, y) in w] for w in bend]))
        @test clearance(p, -1 / R) ≈ R * φ atol = 0.02
    end

    @testset "one wall" begin
        p = LanePerception(ANGLES, X_REF)
        estimate_lane!(p, scan(straight_lane(offset=0.2)))      # learns the width
        lane = estimate_lane!(p, scan(straight_lane(offset=0.2)[2:2]))
        @test lane.walls === :right
        @test lane.e ≈ 0.2 atol = 0.03
        lane = estimate_lane!(p, scan(straight_lane(offset=0.2)[1:1]))
        @test lane.walls === :left
        @test lane.e ≈ 0.2 atol = 0.03
        @test estimate_lane!(p, fill(MAX_RANGE, length(ANGLES))).walls === :none
    end

    @testset "corridor and measuring against other lines" begin
        # a straight lane 2 m wide, its center 0.2 m to the left
        p = LanePerception(ANGLES, X_REF)
        @test detect_lane!(p, scan(straight_lane(offset=0.2))) === :both
        c = lane_corridor(p, 0.3)
        @test all(abs.(c.ny .- 1) .< 0.01)                       # normals point left
        ahead = c.x .> 0   # (behind the vehicle, the walls run out at the scan's edge)
        @test all(isapprox.(c.upper[ahead], 0.7; atol=0.03)) && all(isapprox.(c.lower[ahead], -0.7; atol=0.03))
        @test all(c.lower .<= 0 .<= c.upper)
        @test all(isapprox.(diff(c.x), 0.2; atol=0.01))         # resampled every step
        # measuring against the center line shifted by α gives e - α
        @test measure!(p, c.x, c.y .+ 0.3).e ≈ estimate_lane!(p, scan(straight_lane(offset=0.2))).e + 0.3 atol = 1e-9
        # a hairpin to the left, lane 2.2 m wide: every allowed point keeps the margin
        hairpin = [[(-5.0, 0.5), (1.5, 0.5), (1.5, 0.7), (-5.0, 0.7)],
                   vcat([(-5.0, -1.1)], arc(1.5, 0.6, 1.7, -π / 2, π / 2), [(-5.0, 2.3)])]
        p = LanePerception(ANGLES, X_REF)
        p.width = 2.2
        detect_lane!(p, scan(hairpin))
        c = lane_corridor(p, 0.3)
        returns = [(p.xs[j], p.ys[j]) for j in eachindex(p.xs) if hypot(p.xs[j], p.ys[j]) <= p.cfg.r_max]
        for (i, a) in ((i, a) for i in eachindex(c.x) for a in (c.lower[i], c.upper[i]))
            px, py = c.x[i] + a * c.nx[i], c.y[i] + a * c.ny[i]
            @test minimum(hypot(px - qx, py - qy) for (qx, qy) in returns) >= 0.3 - 1e-3   # (every stride-th return is checked)
        end
        # no lane: no corridor, and measuring reports none
        @test detect_lane!(p, fill(MAX_RANGE, length(ANGLES))) === :none
        @test lane_corridor(p, 0.3) === nothing
        @test measure!(p, [0.0, 1.0, 2.0], [0.0, 0.0, 0.0]).walls === :none
    end

    estimate(e, ψe; κ=0.0) = LaneEstimate(e, ψe, κ, abs(κ), 2.0, :both, fill(κ, 17), 4.0)

    @testset "filter: noise" begin
        f = LaneFilter(WHEELBASE)
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
        f = LaneFilter(WHEELBASE)
        v, r, dt = 5.0, 0.5, 0.01
        e, ψe = 0.0, 0.0
        for _ in 1:100
            out = track!(f, estimate(e, ψe), v, r, dt)
            ψe -= r * dt
            e += (v * sin(ψe) - WHEELBASE * r * cos(ψe)) * dt
            @test abs(out.ψe - (ψe + r * dt)) < 0.01
        end
    end

    @testset "filter: outliers and restarts" begin
        f = LaneFilter(WHEELBASE)
        for _ in 1:50
            track!(f, estimate(0.0, 0.0), 3.0, 0.0, 0.01)
        end
        @test abs(track!(f, estimate(0.0, 1.0), 3.0, 0.0, 0.01).ψe) < 0.01   # one outlier: ignored
        track!(f, estimate(0.0, 0.0), 3.0, 0.0, 0.01)
        outs = [track!(f, estimate(0.5, 1.0), 3.0, 0.0, 0.01) for _ in 1:3]   # the lane really moved
        @test abs(outs[2].ψe) < 0.01 && outs[3].ψe ≈ 1.0 && outs[3].e ≈ 0.5
        # with no lane in view the prediction carries on
        none = LaneEstimate(0.0, 0.0, 0.0, 0.0, 2.0, :none, fill(NaN, 17), 0.0)
        @test track!(f, none, 3.0, 0.0, 0.01).ψe ≈ 1.0 atol = 0.01
        # the curvature ahead is averaged, but a sharp turn is taken at once
        f = LaneFilter(WHEELBASE)
        for _ in 1:20
            track!(f, estimate(0.0, 0.0; κ=0.0), 3.0, 0.0, 0.01)
        end
        @test track!(f, estimate(0.0, 0.0; κ=0.1), 3.0, 0.0, 0.01).κ_profile[5] ≈ 0.025 atol = 0.002
        @test track!(f, estimate(0.0, 0.0; κ=1.0), 3.0, 0.0, 0.01).κ_ahead ≈ 1.0
    end

end
