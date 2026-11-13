# Sensitivity from the linearized closed loop

No simulation and no map: the closed loop linearized about a situation, and the sets it can reach
(zonotopes). From [`analytic.jl`](../analytic.jl), which says how each part is modeled. Unlike the
simulation ([sensitivity.md](sensitivity.md)): no Kalman filter (the estimates go to the planners as
they are), and no age limit on late results. First one situation in detail, then a grid of them.

## The situation

The car in steady state on a lane of constant curvature 0.1 1/m (radius 10.0 m; the
racetracks' center lines bend more than this between 10% and 25% of the way) and width 2.26 m (their
median), its center line, walls and what the LiDAR sees computed from the geometry. There, the controller
holds it at 4.482 m/s, 0.256 m to the left of the lane's center (toward the inside),
steering 0.0519 rad; the estimates' true values: sight 9.08 m,
clearance 1.28 to 5.07 m
along the fan. Linearized there, sampled every scan (25 ms, commands taking effect 26.25 ms
after their scan), the closed loop is stable (spectral radius 0.9725).

## Accuracy

Each estimate with a constant bias anywhere in the box ± the car network's (small_weak) median error
in this situation, each entry on its own (from [error_model.toml](error_model.toml)); the others true.
The sensitivity: the final reachable set, every steady state such biases lead to, by its width in the
car's lateral offset and in its speed. Linear: from the loop's derivative (a bias k times as large gives a
set k times as wide). Finite: the nonlinear steady state solved at the box's extremes (each entry the way
the linear model moves the car most, both ways; every entry the same way, both ways; for the widths also
the two sides opposite), which sees what a bias of this size does where the controller steps or saturates.

| Estimate | Lateral offset: linear [m] | finite [m] | Speed: linear [m/s] | Bias box |
|---|---|---|---|---|
| `e` | 0.00984 | 0.00984 | 0.00197 | ± 0.00512 |
| `ψe` | 0.0356 | 0.0356 | 0.00711 | ± 0.00825 |
| `κ_profile` | 0.261 | 0.2 | 0.0521 | ± 0.0202 to 0.0393 per entry |
| `widths` | 0.0 | 0.0 | 0.0 | ± 0.0501 to 0.0525 per entry |
| `sight` | 0.0 | 0.0 | 0.0 | ± 0.205 |
| `clearance` | 0.0175 | 0.0175 | 0.178 | ± 0.02 to 0.281 per entry |

## Latency

Each estimate late by whole scans, as a late result is used at the first scan after it arrives (no age
limit), the others on time, while the road's curvature varies: a first-order process along the lane,
within ± 0.305 1/m (the racetracks' 95th percentile |κ|), decorrelating over 2.02 m (fitted to their
center lines' autocorrelation). The reachable set of the difference the delay makes to the lateral offset
(against the same loop with the estimate on time, on the same road), over all such roads, by its width, at
delays of 0, 1, 2, 3, 4 scans; the sensitivity: its growth per scan (least squares). The delay
margin: the shortest delay at which the loop goes unstable. (Over the same roads, the lateral offset
itself, with every estimate on time, spans 1.45 m: mostly the car settling to a different offset
in a tighter or wider bend.)

| Estimate | Sensitivity [m per scan of delay] | Width at 0 / 1 / 2 / 3 / 4 scans [m] | Delay margin |
|---|---|---|---|
| `e` | 0.069 | 0.0 / 0.0621 / 0.129 / 0.203 / 0.285 | 25 scans (625.0 ms) |
| `ψe` | 0.151 | 0.0 / 0.0798 / 0.173 / 0.33 / 0.776 | 6 scans (150.0 ms) |
| `κ_profile` | 0.12 | 0.0 / 0.121 / 0.241 / 0.362 / 0.481 | none up to 10 s |
| `widths` | 0.0 | 0.0 / 0.0 / 0.0 / 0.0 / 0.0 | none up to 10 s |
| `sight` | 0.0 | 0.0 / 0.0 / 0.0 / 0.0 / 0.0 | none up to 10 s |
| `clearance` | 0.00255 | 0.0 / 0.00254 / 0.00509 / 0.00763 / 0.0102 | none up to 10 s |

## Across situations

The same, on a lane of each curvature (from straight to the racetracks' tightest bends) and width (about
their 10th, 50th and 90th percentile). The speed is the controller's own there: the speed planner's
limit active there, mostly braking for the free distance ahead along the arc the car turns on (the
planned line cuts to the inside of the bend within its horizon), the yaw-rate cap in the tightest
wide bends. Loop: stable; or, in tight bends, "speed held": the speed loop is unstable in small signal
there (the speed changes the yaw rate, the arc and so the free distance; on the car the command limiter's
ramps hold it to a chatter of a few cm/s), so the lateral loop is analyzed with the speed held at the
steady state's.

### Accuracy: lateral offset, width of the final set [m]

Linear; in brackets the finite (nonlinear) one where it differs by more than 20%; ⚠ where some of the
box's extremes leave no steady state (the loop chatters there).

| κ0 [1/m] | width [m] | speed [m/s] | loop | `e` | `ψe` | `κ_profile` | `widths` | `sight` | `clearance` |
|---|---|---|---|---|---|---|---|---|---|
| 0.0 | 1.6 | 7.24 | stable | 0.00877 | 0.0279 | 0.0712 | 0.0 | 0.0 | 0.0 |
| 0.0 | 2.26 | 7.24 | stable | 0.00877 | 0.0279 | 0.0712 | 0.0 | 0.0 | 0.0 |
| 0.0 | 3.2 | 7.24 | stable | 0.00877 | 0.0279 | 0.0712 | 0.0 | 0.0 | 0.0 |
| 0.02 | 1.6 | 6.21 | stable | 0.00954 | 0.0316 | 0.193 | 0.0 | 0.0 | 0.00364 |
| 0.02 | 2.26 | 6.59 | stable | 0.00904 | 0.0315 | 0.204 | 0.0 | 0.0 | 0.00253 |
| 0.02 | 3.2 | 6.62 | stable | 0.00904 | 0.0316 | 0.206 | 0.0 | 0.0 | 0.00254 (finite: 0.016) |
| 0.05 | 1.6 | 4.71 | stable | 0.0101 | 0.0303 | 0.182 | 0.0 | 0.0 | 0.0155 |
| 0.05 | 2.26 | 5.24 | stable | 0.00987 | 0.0321 | 0.215 | 0.0 | 0.0 | 0.0114 |
| 0.05 | 3.2 | 5.63 | stable | 0.0094 | 0.0348 | 0.17 | 0.0 | 0.0 | 0.0 |
| 0.1 | 1.6 | 4.0 | stable | 0.00977 | 0.0321 | 0.222 | 0.0 | 0.0 (finite: 0.000863) | 0.0164 |
| 0.1 | 2.26 | 4.48 | stable | 0.00984 | 0.0356 | 0.261 (finite: 0.2) | 0.0 | 0.0 | 0.0175 |
| 0.1 | 3.2 | 5.0 | stable | 0.00987 | 0.0379 | 0.291 (finite: 0.211) | 0.0 | 0.0 | 0.0126 |
| 0.15 | 1.6 | 4.38 | speed held | 0.0104 | 0.0401 | 0.319 (finite: 0.24) | 0.0 | 0.0 ⚠ | 0.0 (finite: 0.00342) |
| 0.15 | 2.26 | 5.02 | speed held | 0.0104 | 0.0328 | 0.154 (finite: 0.186) | 0.0 | 0.0 (finite: 0.012) | 0.0 (finite: 0.0015) |
| 0.15 | 3.2 | 5.65 | speed held | 0.0102 | 0.0324 | 0.146 (finite: 0.26) | 0.0 | 0.0 | 0.0 |
| 0.2 | 1.6 | 4.81 | speed held | 0.00972 | 0.0454 (finite: 0.0226) ⚠ | 0.368 (finite: 0.253) | 0.0 | 0.0 (finite: 0.0196) | 0.0 |
| 0.2 | 2.26 | 4.82 | speed held | 0.01 | 0.0346 | 0.173 (finite: 0.243) | 0.0 | 0.0 (finite: 0.022) | 0.0 |
| 0.2 | 3.2 | 4.83 | speed held | 0.00921 | 0.0632 | 0.611 (finite: 0.167) ⚠ | 0.0 (finite: 0.103) | 0.0 (finite: 0.0242) | 0.0 |
| 0.3 | 1.6 | 3.83 | speed held | 0.0108 | 0.045 | 0.334 | 0.0 | 0.0 ⚠ | 0.0 |
| 0.3 | 2.26 | 3.82 | speed held | 0.0106 | 0.0486 | 0.357 (finite: 0.512) | 0.0 | 0.0 (finite: 0.00268) | 0.0 |
| 0.3 | 3.2 | 3.74 | speed held | 0.00911 | 0.0829 | 0.988 | 0.0 (finite: 0.0399) | 0.0 (finite: 0.023) | 0.0 |
| 0.5 | 1.6 | – | no steady state | | | | | | |
| 0.5 | 2.26 | – | no steady state | | | | | | |
| 0.5 | 3.2 | – | no steady state | | | | | | |

### Latency: sensitivity [m per scan of delay]

Over the delays (0 to 4 scans) the loop stays stable at; ⚠ where it goes unstable within
them, "unstable at n" where it does from n scans.

| κ0 [1/m] | width [m] | speed [m/s] | loop | `e` | `ψe` | `κ_profile` | `widths` | `sight` | `clearance` |
|---|---|---|---|---|---|---|---|---|---|
| 0.0 | 1.6 | 7.24 | stable | 0.471 | 1.41 | 0.491 | 0.0 | 0.0 | 0.0 |
| 0.0 | 2.26 | 7.24 | stable | 0.471 | 1.41 | 0.491 | 0.0 | 0.0 | 0.0 |
| 0.0 | 3.2 | 7.24 | stable | 0.471 | 1.41 | 0.491 | 0.0 | 0.0 | 0.0 |
| 0.02 | 1.6 | 6.21 | stable | 0.334 | 0.304 | 0.361 | 0.0 | 0.0 | 0.0713 |
| 0.02 | 2.26 | 6.59 | stable | 0.297 | 0.386 | 0.347 | 0.0 | 0.0 | 0.000541 |
| 0.02 | 3.2 | 6.62 | stable | 0.304 | 0.424 | 0.352 | 0.0 | 0.0 | 0.000448 |
| 0.05 | 1.6 | 4.71 | stable | 0.143 | 0.136 | 0.162 | 0.0 | 0.0 | 0.0468 |
| 0.05 | 2.26 | 5.24 | stable | 0.174 | 0.167 | 0.206 | 0.0 | 0.0 | 0.0695 |
| 0.05 | 3.2 | 5.63 | stable | 0.129 | 1.29 | 0.152 | 0.0 | 0.0 | 0.0 |
| 0.1 | 1.6 | 4.0 | stable | 0.0535 | 0.107 | 0.0941 | 0.0 | 0.0 | 0.00294 |
| 0.1 | 2.26 | 4.48 | stable | 0.069 | 0.151 | 0.12 | 0.0 | 0.0 | 0.00255 |
| 0.1 | 3.2 | 5.0 | stable | 0.088 | 0.204 | 0.149 | 0.0 | 0.0 | 0.00206 |
| 0.15 | 1.6 | 4.38 | speed held | 0.0596 | 0.467 | 0.117 | 0.0 | 0.0 | 0.0 |
| 0.15 | 2.26 | 5.02 | speed held | 0.0754 | 0.342 ⚠ | 0.0836 | 0.0 | 0.0 | 0.0 |
| 0.15 | 3.2 | 5.65 | speed held | 0.22 | 0.489 ⚠ | 0.112 | 0.0 | 0.0 | 0.0 |
| 0.2 | 1.6 | 4.81 | speed held | 0.0541 | 0.274 | 0.126 | 0.0 | 0.0 | 0.0 |
| 0.2 | 2.26 | 4.82 | speed held | 0.0811 | 0.304 ⚠ | 0.0766 | 0.0 | 0.0 | 0.0 |
| 0.2 | 3.2 | 4.83 | speed held | 0.0547 | 0.145 | 0.162 | 0.0 | 0.0 | 0.0 |
| 0.3 | 1.6 | 3.83 | speed held | 0.0308 | 0.192 ⚠ | 0.0554 | 0.0 | 0.0 | 0.0 |
| 0.3 | 2.26 | 3.82 | speed held | 0.0265 | 3.2 | 0.0558 | 0.0 | 0.0 | 0.0 |
| 0.3 | 3.2 | 3.74 | speed held | 0.0311 | 0.0899 | 0.107 | 0.0 | 0.0 | 0.0 |
| 0.5 | 1.6 | – | no steady state | | | | | | |
| 0.5 | 2.26 | – | no steady state | | | | | | |
| 0.5 | 3.2 | – | no steady state | | | | | | |

### Delay margin [ms] (– : none up to 10 s)

| κ0 [1/m] | width [m] | speed [m/s] | loop | `e` | `ψe` | `κ_profile` | `widths` | `sight` | `clearance` |
|---|---|---|---|---|---|---|---|---|---|
| 0.0 | 1.6 | 7.24 | stable | 375 | 125 | – | – | – | – |
| 0.0 | 2.26 | 7.24 | stable | 375 | 125 | – | – | – | – |
| 0.0 | 3.2 | 7.24 | stable | 375 | 125 | – | – | – | – |
| 0.02 | 1.6 | 6.21 | stable | 325 | 200 | – | – | – | – |
| 0.02 | 2.26 | 6.59 | stable | 400 | 150 | – | – | – | – |
| 0.02 | 3.2 | 6.62 | stable | 400 | 150 | – | – | – | – |
| 0.05 | 1.6 | 4.71 | stable | 375 | 225 | – | – | – | – |
| 0.05 | 2.26 | 5.24 | stable | 375 | 225 | – | – | – | 2575 |
| 0.05 | 3.2 | 5.63 | stable | 500 | 125 | – | – | – | – |
| 0.1 | 1.6 | 4.0 | stable | 650 | 150 | – | – | – | – |
| 0.1 | 2.26 | 4.48 | stable | 625 | 150 | – | – | – | – |
| 0.1 | 3.2 | 5.0 | stable | 575 | 150 | – | – | – | – |
| 0.15 | 1.6 | 4.38 | speed held | 625 | 125 | – | – | – | – |
| 0.15 | 2.26 | 5.02 | speed held | 350 | 100 | – | – | – | – |
| 0.15 | 3.2 | 5.65 | speed held | 275 | 100 | – | – | – | – |
| 0.2 | 1.6 | 4.81 | speed held | 600 | 125 | – | – | – | – |
| 0.2 | 2.26 | 4.82 | speed held | 375 | 100 | – | – | – | – |
| 0.2 | 3.2 | 4.83 | speed held | 1050 | 150 | – | – | – | – |
| 0.3 | 1.6 | 3.83 | speed held | 500 | 100 | – | – | – | – |
| 0.3 | 2.26 | 3.82 | speed held | 575 | 125 | – | – | – | – |
| 0.3 | 3.2 | 3.74 | speed held | 1725 | 150 | – | – | – | – |
| 0.5 | 1.6 | – | no steady state | | | | | | |
| 0.5 | 2.26 | – | no steady state | | | | | | |
| 0.5 | 3.2 | – | no steady state | | | | | | |

## What a linear model leaves out

The command limiter's and the VESC's ramps (except as the held speed in tight bends), the steering
limits and the tires' grip limit (it holds for small deviations); the speed planner's switches between
its limits (only the one active at the situation counts); sight and clearance at their caps (there, their
derivatives are zero). The finite column of the accuracy table sees the controller's steps for a bias of
the median error's size, but not the dynamics on the way there. Sight and clearance are where the view and
the arc truly end, not rounded to the samples truth.jl uses, so that they change smoothly.
