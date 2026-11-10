# How the networks' errors are distributed

A Student t, fitted by maximum likelihood to every network's errors (estimate − truth) on the
held-out tracks (Spa, Monza, Silverstone, Oschersleben, YasMarina) while driving, over the same entries as the RMS errors
(profiles: the entries the car can see, pooled over the distance ahead; at most 200000 errors
per fit, a random subset); and to the same errors divided by the network's own standard deviation
(columns "/ σ": what the selector and the Kalman filter treat as a standard Gaussian). Location μ,
scale s, degrees of freedom ν: the tails are heavier the smaller ν (a Gaussian as ν → ∞, a Cauchy
at ν = 1). Its density: Γ((ν+1)/2) / (Γ(ν/2) √(νπ) s) · (1 + ((x − μ)/s)² / ν)^(−(ν+1)/2).

The Student t had the least AIC of the three for 18 of 18 networks' errors and 17 of 18 of the errors over σ (otherwise: `widths`, small_weak (errors over σ): Laplace). A Gaussian and a Laplace were the other candidates. From [`errors.jl`](../errors.jl); the
networks are those of [networks.md](networks.md).

| Estimate | Network | μ | s | ν | Unit | μ / σ | s / σ | ν / σ |
|---|---|---|---|---|---|---|---|---|
| `e` | small_weak | -0.000195 | 0.00616 | 3.9 | m | -0.0304 | 0.998 | 9.4 |
| `e` | small_strong | -0.000185 | 0.00531 | 3.9 | m | -0.0302 | 0.937 | 7.1 |
| `e` | large | -0.000154 | 0.0052 | 3.9 | m | -0.0277 | 1.0 | 7.1 |
| `ψe` | small_weak | 0.000616 | 0.00781 | 2.1 | rad | 0.0621 | 0.933 | 6.2 |
| `ψe` | small_strong | 0.00278 | 0.00628 | 2.1 | rad | 0.322 | 0.864 | 6.0 |
| `ψe` | large | -0.000373 | 0.00596 | 2.0 | rad | -0.0516 | 0.886 | 5.4 |
| `κ_profile` | small_weak | -0.000158 | 0.00697 | 0.82 | 1/m | -0.0186 | 0.602 | 3.0 |
| `κ_profile` | small_strong | -1.44e-5 | 0.00488 | 0.77 | 1/m | -0.00157 | 0.574 | 3.4 |
| `κ_profile` | large | 0.00011 | 0.00471 | 0.79 | 1/m | 0.0131 | 0.654 | 3.4 |
| `widths` | small_weak | 0.000618 | 0.025 | 5.0 | m | -0.00784 | 0.96 | 3.6 |
| `widths` | small_strong | -0.000603 | 0.0227 | 4.8 | m | -0.0402 | 1.0 | 6.2 |
| `widths` | large | 0.00147 | 0.0226 | 4.6 | m | 0.0436 | 0.994 | 6.0 |
| `sight` | small_weak | -0.00818 | 0.0278 | 0.88 | m | -0.163 | 0.548 | 2.8 |
| `sight` | small_strong | -0.00803 | 0.0327 | 1.3 | m | -0.17 | 0.781 | 5.0 |
| `sight` | large | -0.00423 | 0.024 | 1.1 | m | -0.155 | 0.986 | 5.8 |
| `clearance` | small_weak | 0.00538 | 0.0298 | 1.2 | m | 0.143 | 1.0 | 7.1 |
| `clearance` | small_strong | 0.00379 | 0.026 | 1.6 | m | 0.129 | 0.991 | 7.4 |
| `clearance` | large | 0.0101 | 0.026 | 1.8 | m | 0.369 | 1.1 | 11.0 |
