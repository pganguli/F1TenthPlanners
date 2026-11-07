# Random problems: a wobbly loop (closed) or a random-heading line (open),
# normals tilted off the line's own, and bounds of every kind: some points
# held (lower == upper), some unbounded on one or both sides.
function random_problem(rng, n, closed)
    if closed
        θ = range(0, 2π; length=n + 1)[1:n]
        R = 5 .+ 0.8 .* sin.(3 .* θ .+ 2π * rand(rng)) .+ 0.05 .* randn(rng, n)
        x, y = R .* cos.(θ), R .* sin.(θ)
    else
        h = cumsum(0.15 .* randn(rng, n))
        x, y = cumsum(0.2 .* cos.(h)), cumsum(0.2 .* sin.(h))
    end
    nx, ny = left_normals(x, y; closed)
    φ = 0.1 .* randn(rng, n)
    normals = (nx .* cos.(φ) .- ny .* sin.(φ), nx .* sin.(φ) .+ ny .* cos.(φ))
    lower, upper = -0.5 .* rand(rng, n), 0.5 .* rand(rng, n)
    for i in 1:n
        u = rand(rng)
        if u < 0.1
            lower[i] = upper[i] = 0.0
        elseif u < 0.15
            lower[i] = -Inf
        elseif u < 0.2
            upper[i] = Inf
        elseif u < 0.22
            lower[i], upper[i] = -Inf, Inf
        end
    end
    return x, y, lower, upper, normals
end
