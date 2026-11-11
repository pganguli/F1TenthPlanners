using StanleyController
using StanleyController: nsegments, total_length
using Test

@testset "StanleyController" begin
    @testset "wrap_angle" begin
        @test wrap_angle(0.0) == 0.0
        @test wrap_angle(3π / 2) ≈ -π / 2
        @test wrap_angle(-3π / 2) ≈ π / 2
        @test abs(wrap_angle(7π)) ≈ π
    end

    @testset "steering law" begin
        c = Stanley(k=2.0, k_soft=0.0)
        @test steering_angle(c, 0.0, 0.0, 5.0) == 0.0
        @test steering_angle(c, 0.0, 0.1, 5.0) ≈ 0.1                    # heading term alone
        @test steering_angle(c, 1.0, 0.0, 5.0) ≈ atan(2.0 / 5.0)        # Snider (2009), Eq. 5
        @test steering_angle(c, -1.0, 0.0, 5.0) ≈ -atan(2.0 / 5.0)
        @test steering_angle(c, 1.0, 0.0, 0.0) ≈ π / 2                  # standstill, no softening
        @test steering_angle(Stanley(k=2.0, k_soft=1.0), 1.0, 0.0, 0.0) ≈ atan(2.0)
        @test steering_angle(c, 0.0, 2π + 0.1, 5.0) ≈ 0.1               # heading error is wrapped
        # path to the left (e > 0) or pointing left (ψe > 0) steers left
        @test steering_angle(c, 0.3, 0.0, 3.0) > 0
        @test steering_angle(c, 0.0, 0.2, 3.0) > 0
        # clamping
        lim = Stanley(k=2.0, max_steer=0.4)
        @test steering_angle(lim, 10.0, 1.0, 1.0) == 0.4
        @test steering_angle(lim, -10.0, -1.0, 1.0) == -0.4
        # yaw damping: turning left faster than the path asks steers right
        damped = Stanley(k=2.0, k_yaw=0.5)
        @test steering_angle(damped, 0.0, 0.0, 2.0; yaw_rate=0.4, path_yaw_rate=0.0) ≈ -0.2
        @test steering_angle(damped, 0.0, 0.0, 2.0; yaw_rate=0.4, path_yaw_rate=0.4) ≈ 0.0
        # reverse speed uses |v|
        @test steering_angle(c, 1.0, 0.0, -5.0) ≈ steering_angle(c, 1.0, 0.0, 5.0)
        @test_throws ArgumentError Stanley(k=0.0)
        @test_throws ArgumentError Stanley(k_soft=-1.0)
        @test_throws ArgumentError Stanley(k_yaw=-1.0)
        @test_throws ArgumentError Stanley(max_steer=0.0)
    end

    @testset "front_axle" begin
        @test all(front_axle(1.0, 2.0, 0.0, 0.5) .≈ (1.5, 2.0))
        @test all(isapprox.(front_axle(0.0, 0.0, π / 2, 2.0), (0.0, 2.0); atol=1e-12))
    end

    @testset "straight path" begin
        p = Path([0.0, 10.0], [0.0, 0.0])
        @test length(p) == 2 && nsegments(p) == 1 && total_length(p) == 10.0
        err = tracking_error(p, 3.0, -0.5, 0.0)        # right of a path heading +x
        @test err.e ≈ 0.5 && err.ψe ≈ 0.0 && err.κ == 0.0 && err.s ≈ 3.0
        err = tracking_error(p, 3.0, 0.5, 0.2)         # left of it, nose pointing left
        @test err.e ≈ -0.5 && err.ψe ≈ -0.2
        @test steering_angle(Stanley(), tracking_error(p, 3.0, -0.5, 0.0), 2.0) > 0
        @test tracking_error(p, 3.0, 0.0, 0.0).e == 0.0
        # past the ends of an open path, the end segments are extended
        err = tracking_error(p, -2.0, -0.5, 0.0)
        @test err.e ≈ 0.5 && err.s ≈ -2.0 && err.segment == 1
        err = tracking_error(p, 12.0, 0.3, 0.0)
        @test err.e ≈ -0.3 && err.s ≈ 12.0
        bent = Path([0.0, 1.0, 1.0], [0.0, 0.0, 1.0])
        @test tracking_error(bent, 1.2, 2.0, π / 2).e ≈ 0.2   # right of a path heading +y
        @test_throws ArgumentError Path([0.0], [0.0])
        @test_throws ArgumentError Path([0.0, 0.0], [1.0, 1.0])
        @test_throws DimensionMismatch Path([0.0, 1.0], [0.0])
    end

    @testset "closed circle" begin
        R, n = 5.0, 400
        φ = range(0, 2π; length=n + 1)[1:end-1]
        p = Path(R .* cos.(φ), R .* sin.(φ); closed=true)
        @test length(p) == n && nsegments(p) == n
        @test total_length(p) ≈ 2π * R rtol = 1e-3
        @test all(isapprox.(p.curvature, 1 / R; rtol=1e-3))   # counterclockwise: left turn
        a = 0.3
        err = tracking_error(p, (R + 0.4) * cos(a), (R + 0.4) * sin(a), a + π / 2)
        @test err.e ≈ 0.4 atol = 1e-3                   # outside: path is to the left
        @test abs(err.ψe) < 1e-2
        @test err.κ ≈ 1 / R rtol = 1e-3
        @test err.s ≈ a * R rtol = 1e-2
        err = tracking_error(p, (R - 0.4) * cos(a), (R - 0.4) * sin(a), a + π / 2 - 0.1)
        @test err.e ≈ -0.4 atol = 1e-3
        @test err.ψe ≈ 0.1 atol = 1e-2
        # windowed search agrees with the full search, also across the seam
        for a in (0.05, 2π - 0.05, 3.0)
            full = tracking_error(p, R * cos(a), R * sin(a), a + π / 2)
            near = tracking_error(p, R * cos(a), R * sin(a), a + π / 2; hint=mod1(full.segment + 3, n), window=10)
            @test near.segment == full.segment && near.e ≈ full.e
        end
        # a repeated final point is dropped
        sq = Path([0.0, 1.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 1.0, 0.0]; closed=true)
        @test length(sq) == 4 && total_length(sq) == 4.0
        @test all(isapprox.(sq.curvature, π / 2))
        # clockwise circle: right turns have negative curvature
        q = Path(R .* cos.(-φ), R .* sin.(-φ); closed=true)
        @test all(isapprox.(q.curvature, -1 / R; rtol=1e-3))
    end

    @testset "points along a path, curvature of three points" begin
        sq = Path([0.0, 1.0, 1.0, 0.0], [0.0, 0.0, 1.0, 1.0]; closed=true)   # unit square, 4 m around
        @test point_at(sq, 0.5) == (0.5, 0.0) && point_at(sq, 3.5) == (0.0, 0.5)
        @test all(point_at(sq, 4.25) .≈ (0.25, 0.0)) && all(point_at(sq, -0.25) .≈ (0.0, 0.25))
        line = Path([0.0, 1.0], [0.0, 0.0])
        @test point_at(line, 2.0) == (2.0, 0.0) && point_at(line, -1.0) == (-1.0, 0.0)   # extended
        @test menger_curvature((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0)) ≈ 1.0      # unit circle, left
        @test menger_curvature((-1.0, 0.0), (0.0, 1.0), (1.0, 0.0)) ≈ -1.0     # right
        @test menger_curvature((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)) == 0.0
    end
end
