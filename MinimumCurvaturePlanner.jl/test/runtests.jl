using MinimumCurvaturePlanner
using MinimumCurvaturePlanner: _box_qp, _normal_equations, _entry, _Penta
using LinearAlgebra, Random, Test

include("dense_reference.jl")
using .DenseReference: dense_offsets, dense_box_qp, dense_qp
include("random_problems.jl")

# Curvature of the circle through three points (positive turning left).
function menger(a, b, c)
    cross = (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1])
    return 2cross / (hypot(b[1] - a[1], b[2] - a[2]) * hypot(c[1] - b[1], c[2] - b[2]) * hypot(c[1] - a[1], c[2] - a[2]))
end
sharpest(x, y; closed=false) = maximum(abs(menger((x[mod1(k - 1, end)], y[mod1(k - 1, end)]), (x[k], y[k]),
    (x[mod1(k + 1, end)], y[mod1(k + 1, end)]))) for k in (closed ? eachindex(x) : 2:length(x)-1))

# a stadium (two straights joined by half circles of radius r), points ~ds apart
function stadium(len, r, ds)
    pts = Tuple{Float64,Float64}[]
    for x in 0:ds:len-ds; push!(pts, (x, -r)); end
    for φ in range(-π / 2, π / 2; length=round(Int, π * r / ds) + 1)[1:end-1]; push!(pts, (len + r * cos(φ), r * sin(φ))); end
    for x in len:-ds:ds; push!(pts, (x, r)); end
    for φ in range(π / 2, 3π / 2; length=round(Int, π * r / ds) + 1)[1:end-1]; push!(pts, (r * cos(φ), r * sin(φ))); end
    return first.(pts), last.(pts)
end

@testset "MinimumCurvaturePlanner" begin
    @testset "box-constrained QP" begin
        # against projected gradient descent, run to convergence
        H = [4.0 1.0 0.5; 1.0 3.0 0.2; 0.5 0.2 2.0]
        g = [-8.0, 3.0, -1.0]
        lb, ub = [-1.0, -0.5, -2.0], [1.0, 0.5, 0.1]
        x = zeros(3)
        for _ in 1:20000
            x = clamp.(x .- 0.05 .* (H * x .+ g), lb, ub)
        end
        @test _box_qp(H, g, lb, ub) ≈ x atol = 1e-8
        @test _box_qp(H, g, fill(-Inf, 3), fill(Inf, 3)) ≈ -(H \ g)   # no bounds: the unconstrained minimum
    end

    @testset "normals" begin
        nx, ny = left_normals([0.0, 1.0, 2.0], [0.0, 0.0, 0.0])
        @test nx ≈ [0, 0, 0] atol = 1e-12
        @test ny ≈ [1, 1, 1]
        # a closed square: the corner normals bisect the turn
        nx, ny = left_normals([0.0, 1.0, 1.0, 0.0], [0.0, 0.0, 1.0, 1.0]; closed=true)
        @test (nx[1], ny[1]) .|> (v -> round(v; digits=6)) == (round(sqrt(0.5); digits=6), round(sqrt(0.5); digits=6))
    end

    @testset "a straight stays on the reference" begin
        x = collect(0.0:0.2:5.0)
        y = zeros(length(x))
        px, py = min_curvature_path(x, y, fill(-1.0, length(x)), fill(1.0, length(x)))
        @test px ≈ x && maximum(abs, py) < 1e-6
    end

    sumsq(x, y; closed=false) = sum(menger((x[mod1(k - 1, end)], y[mod1(k - 1, end)]), (x[k], y[k]),
        (x[mod1(k + 1, end)], y[mod1(k + 1, end)]))^2 for k in (closed ? eachindex(x) : 2:length(x)-1))

    @testset "an open bend" begin
        # a 90° left bend of radius 1 between straights
        pts = vcat([(x, 0.0) for x in -3.0:0.2:-0.2], [(sin(φ), 1 - cos(φ)) for φ in range(0, π / 2; length=9)],
            [(1.0, y) for y in 1.2:0.2:4.0])
        x, y = first.(pts), last.(pts)
        n = length(x)
        # a narrow corridor: the quadratic program alone already cuts the corner
        α = min_curvature_offsets(x, y, fill(-0.1, n), fill(0.1, n))
        @test all(abs.(α) .<= 0.1 + 1e-9)
        px, py = min_curvature_path(x, y, fill(-0.1, n), fill(0.1, n))
        @test sumsq(px, py) < sumsq(x, y)
        # a wide one: refining on the exact curvature bends far less
        px, py = min_curvature_path(x, y, fill(-1.0, n), fill(1.0, n); iterations=20)
        @test all(abs.(min_curvature_offsets(x, y, fill(-1.0, n), fill(1.0, n); iterations=20)) .<= 1 + 1e-9)
        @test sumsq(px, py) < 0.1 * sumsq(x, y)
        @test sharpest(px, py) < 0.5 * sharpest(x, y)
    end

    @testset "a closed lap" begin
        x, y = stadium(10.0, 2.0, 0.2)
        n = length(x)
        lower, upper = fill(-0.8, n), fill(0.8, n)
        α = min_curvature_offsets(x, y, lower, upper; closed=true, iterations=20)
        @test all(abs.(α) .<= 0.8 + 1e-9)
        px, py = min_curvature_path(x, y, lower, upper; closed=true, iterations=20)
        @test sumsq(px, py; closed=true) < 0.6 * sumsq(x, y; closed=true)
        @test sharpest(px, py; closed=true) < 0.85 * sharpest(x, y; closed=true)
        # the quadratic program alone pulls the long corners inward (the
        # linearization's bias), and the refinement undoes it
        p0 = min_curvature_path(x, y, lower, upper; closed=true)
        @test sumsq(px, py; closed=true) < sumsq(p0...; closed=true)
    end

    @testset "banded solver against the dense one" begin
        rng = Xoshiro(20261107)
        qpcost(H, g, α) = dot(α, H * α) / 2 + dot(g, α)
        # The quadratic program alone. Short open lines are ill-conditioned
        # (κ(H) ≈ 1.6e7: only the tiebreak pins their shift and tilt), which
        # limits either solver to about 1e-9 of the exact answer; closed ones
        # agree to about 1e-14.
        for closed in (false, true), n in [3:12; 15; 20; 31; 50; 100; 200; 300], _ in 1:3
            x, y, lower, upper, normals = random_problem(rng, n, closed)
            α = min_curvature_offsets(x, y, lower, upper; closed, normals)
            ref = dense_offsets(x, y, lower, upper; closed, normals)
            H, g = dense_qp(x, y; closed, normals)
            @test isapprox(α, ref; rtol=closed ? 1e-10 : 1e-9, atol=1e-14)
            @test isapprox(qpcost(H, g, α), qpcost(H, g, ref); rtol=1e-12, atol=1e-14)
            @test all(lower .<= α .<= upper)
        end
        # The solver on matrices like the refinement's (CᵀC with C = J, three
        # nonzeros per row), with many variables held, so that few are free.
        # A larger tiebreak keeps them well conditioned, to compare tightly.
        for closed in (false, true), n in [3:12; 20; 57], held in (0.1, 0.5, 0.9), _ in 1:3
            ks = closed ? (1:n) : (2:n-1)
            C, v = randn(rng, length(ks), 3), randn(rng, length(ks))
            H, g = _normal_equations(ks, C, v, n, closed, 1e-2)
            Hd = H isa _Penta ? [_entry(H, i, j) for i in 1:n, j in 1:n] : H
            A = zeros(length(ks), n)
            for (row, k) in enumerate(ks), p in 1:3
                A[row, mod1(k + p - 2, n)] += C[row, p]
            end
            @test Hd ≈ A' * A + 1e-2 * I rtol = 1e-14
            @test g ≈ A' * v rtol = 1e-14
            lb, ub = -rand(rng, n), rand(rng, n)
            for i in 1:n
                rand(rng) < held && (lb[i] = ub[i] = 0.0)
            end
            @test _box_qp(H, g, lb, ub) ≈ dense_box_qp(Hd, g, lb, ub) rtol = 1e-10
        end
        # Refined: the exact curvature's Jacobian comes from finite
        # differences (step 1e-7 m), which turn rounding-level differences into
        # ~1e-9 of the Jacobian, and Gauss-Newton grows them from step to step:
        # moving the dense version's own input by one ulp moves its result by up
        # to ~1e-5 on these wobbly loops. So only agreement well within the
        # refinement's own stopping tolerance (steps under 1e-4 m) is asked.
        for closed in (false, true), n in [3:8; 20; 100; 300], _ in 1:2
            x, y, lower, upper, normals = random_problem(rng, n, closed)
            α = min_curvature_offsets(x, y, lower, upper; closed, normals, iterations=20)
            ref = dense_offsets(x, y, lower, upper; closed, normals, iterations=20)
            @test isapprox(α, ref; rtol=1e-3)
            @test all(lower .- 1e-12 .<= α .<= upper .+ 1e-12)   # α + t·step may round past a bound
        end
        # the stadium lap, refined, as the tests above use it
        x, y = stadium(10.0, 2.0, 0.2)
        n = length(x)
        α = min_curvature_offsets(x, y, fill(-0.8, n), fill(0.8, n); closed=true, iterations=20)
        @test α ≈ dense_offsets(x, y, fill(-0.8, n), fill(0.8, n); closed=true, iterations=20) rtol = 1e-6
    end

    @testset "edge cases" begin
        for (closed, n) in ((false, 3), (true, 3), (true, 4), (true, 5), (false, 9), (true, 9)), it in (0, 20)
            x, y = closed ? (cos.(2π .* (0:n-1) ./ n), sin.(2π .* (0:n-1) ./ n)) : (collect(0.0:n-1), 0.1 .* (0:n-1) .^ 2)
            # every point held: nothing moves
            @test min_curvature_offsets(x, y, zeros(n), zeros(n); closed, iterations=it) == zeros(n)
            # a corridor wide enough that no bound is reached (refined, a
            # 3-point loop then shrinks to a point, where curvature is undefined)
            wide = fill(1e3, n)
            closed && n == 3 && it > 0 || @test min_curvature_offsets(x, y, -wide, wide; closed, iterations=it) ≈
                  dense_offsets(x, y, -wide, wide; closed, iterations=it) rtol = it == 0 ? 1e-9 : 1e-6
            # all but one held, then all but two
            for free in (1:1, 1:2)
                lo, up = zeros(n), zeros(n)
                lo[free] .= -1; up[free] .= 1
                @test min_curvature_offsets(x, y, lo, up; closed, iterations=it) ≈
                      dense_offsets(x, y, lo, up; closed, iterations=it) rtol = 1e-9 atol = 1e-14
            end
        end
    end

    @testset "argument checks" begin
        x, y = [0.0, 1.0, 2.0], [0.0, 0.0, 0.0]
        @test_throws ArgumentError min_curvature_path(x, y, [0.1, -1.0, -1.0], [1.0, 1.0, 1.0])   # reference outside
        @test_throws DimensionMismatch min_curvature_path(x, y, [-1.0, -1.0], [1.0, 1.0])
        @test_throws ArgumentError min_curvature_path(x[1:2], y[1:2], [-1.0, -1.0], [1.0, 1.0])
    end
end
