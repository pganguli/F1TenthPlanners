using VelocityProfilePlanner
using Test

@testset "VelocityProfilePlanner" begin
    @testset "cornering_speed" begin
        @test cornering_speed(0.0; a_lat=4.0, v_max=5.0) == 5.0
        @test cornering_speed(0.0; a_lat=4.0) == Inf
        @test cornering_speed(1.0; a_lat=4.0, v_max=5.0) ≈ 2.0
        @test cornering_speed(-1.0; a_lat=4.0, v_max=5.0) ≈ 2.0
        @test cornering_speed(100.0; a_lat=4.0, v_max=5.0, v_min=1.0) == 1.0
        @test cornering_speed(0.01; a_lat=4.0, v_max=5.0) == 5.0
        @test_throws ArgumentError cornering_speed(1.0; a_lat=0.0)
        @test_throws ArgumentError cornering_speed(1.0; a_lat=1.0, v_max=1.0, v_min=2.0)
    end

    @testset "a straight ending in a stop" begin
        s = collect(0.0:1.0:10.0)
        v = velocity_profile(s, zeros(11); a_lat=5.0, a_brake=2.0, v_max=8.0, v_end=0.0)
        @test v ≈ min.(8.0, sqrt.(2 * 2.0 .* (10.0 .- s)))
    end

    @testset "braking for a curve ahead" begin
        s = collect(0.0:0.5:20.0)
        κ = [x >= 15 ? 0.5 : 0.0 for x in s]          # a 2 m radius curve from 15 m on
        v = velocity_profile(s, κ; a_lat=4.0, a_brake=2.5, v_max=10.0)
        v_curve = sqrt(4.0 / 0.5)
        @test all(v[s .>= 15] .≈ v_curve)
        @test v[1] ≈ min(10.0, sqrt(v_curve^2 + 2 * 2.5 * 15))
        @test all(diff(v[s .<= 15]) .<= 1e-12)        # slowing down toward it
        # the backward pass is the slowest of all the braking limits
        direct = [minimum(sqrt(min(10.0, sqrt(4.0 / max(abs(κ[j]), eps())))^2 + 2 * 2.5 * (s[j] - s[i])) for j in i:length(s))
                  for i in eachindex(s)]
        @test v ≈ direct
    end

    @testset "accelerating from the current speed" begin
        s = collect(0.0:1.0:10.0)
        v = velocity_profile(s, zeros(11); a_lat=5.0, a_brake=2.0, a_accel=1.0, v_max=8.0, v_start=1.0)
        @test v ≈ min.(8.0, sqrt.(1.0 .+ 2 * 1.0 .* s))
    end

    @testset "speed caps" begin
        s = collect(0.0:1.0:10.0)
        cap = fill(Inf, 11)
        cap[6] = 1.0                                   # e.g. an obstacle 5 m ahead
        v = velocity_profile(s, zeros(11); a_lat=5.0, a_brake=2.0, v_cap=cap)
        @test v[1] ≈ sqrt(1 + 2 * 2.0 * 5) && v[6] == 1.0 && v[end] == Inf
    end

    @testset "friction circle" begin
        s = collect(0.0:0.25:3.0)
        κ = fill(0.2, length(s))                       # braking to a stop in a curve
        free = velocity_profile(s, κ; a_lat=4.0, a_brake=3.0, v_end=0.0)
        shared = velocity_profile(s, κ; a_lat=4.0, a_brake=3.0, a_grip=4.2, v_end=0.0)
        @test all(shared .<= free .+ 1e-12) && shared[1] < free[1]
        # with no curvature the circle costs nothing
        @test velocity_profile(s, zeros(length(s)); a_lat=4.0, a_brake=3.0, a_grip=4.5, v_end=1.0) ≈
              velocity_profile(s, zeros(length(s)); a_lat=4.0, a_brake=3.0, v_end=1.0)
    end

    @testset "argument checks" begin
        @test_throws ArgumentError velocity_profile([1.0, 0.0], [0.0, 0.0]; a_lat=1.0, a_brake=1.0)
        @test_throws DimensionMismatch velocity_profile([0.0, 1.0], [0.0]; a_lat=1.0, a_brake=1.0)
        @test_throws ArgumentError velocity_profile([0.0, 1.0], [0.0, 0.0]; a_lat=1.0, a_brake=0.0)
        @test_throws DimensionMismatch velocity_profile([0.0, 1.0], [0.0, 0.0]; a_lat=1.0, a_brake=1.0, v_cap=[1.0])
    end
end
