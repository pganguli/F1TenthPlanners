# Sensitivity from the linearized closed loop

No simulation and no map: the closed loop linearized about one situation, and the sets it can reach
(zonotopes). From [`analytic.jl`](../analytic.jl), which says how each part is modeled. Unlike the
simulation ([sensitivity.md](sensitivity.md)): no Kalman filter (the estimates go to the planners as
they are), and no age limit on late results.

## The situation

The car in steady state on a lane of constant curvature 0.1 1/m (radius 10.0 m; the
racetracks' center lines bend more than this between 10% and 25% of the way) and width 2.26 m (their
median), its center line, walls and what the LiDAR sees computed from the geometry. (On a tighter lane,
from 0.15 1/m, the speed planner's yaw-rate cap sets the speed, and the speed loop through it is unstable
in small signal: in the simulator the command limiter's ramps hold it to a chatter of about ±2 cm/s, so a
linear model doesn't hold there.) There, the controller holds it at 4.482 m/s, 0.256 m
to the left of the lane's center (toward the inside), steering 0.0519 rad; the estimates' true
values: sight 9.08 m, clearance 1.28 to
5.07 m along the fan. Linearized there, sampled every scan (25 ms,
commands taking effect 26.25 ms after their scan), the closed loop is stable (spectral
radius 0.9725).

## Accuracy

Each estimate with a constant bias anywhere in the box ± the car network's (small_weak) median error
in this situation, each entry on its own (from [error_model.toml](error_model.toml)); the others true.
The sensitivity: the final reachable set, every steady state such biases lead to, by its width in the
car's lateral offset and in its speed. The loop is linear, so a bias k times as large gives a set k times
as wide.

| Estimate | Lateral offset: width of the final set [m] | Speed: width [m/s] | Bias box |
|---|---|---|---|
| `e` | 0.00984 | 0.00197 | ± 0.00512 |
| `ψe` | 0.0356 | 0.00711 | ± 0.00825 |
| `κ_profile` | 0.261 | 0.0521 | ± 0.0202 to 0.0393 per entry |
| `widths` | 0.0 | 0.0 | ± 0.0501 to 0.0525 per entry |
| `sight` | 0.0 | 0.0 | ± 0.205 |
| `clearance` | 0.0175 | 0.178 | ± 0.02 to 0.281 per entry |

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

## Why some are zero

At this situation, a small change in `sight` or the `widths` changes no command: the path planner
rebuilds the lane in 0.25 m steps up to the sight (a small change crosses no step), the
sight's braking limit on the speed is not the active one, and the planned line doesn't touch the lane's
bounds. Their sensitivity is exactly zero here: they matter only through larger errors (crossing a step,
reaching a bound), which a linear model can't see. `clearance` acts on the speed only (it sets the speed
here, through the free distance ahead), and on the lateral offset only through the speed.

## What a linear model leaves out

The command limiter's and the VESC's ramps, the steering limits and the tires' grip limit (it holds for
small deviations); the speed planner's switches between its limits (only the one active here counts);
sight and clearance at their caps (there, their derivatives are zero). Sight and clearance are where the
view and the arc truly end, not rounded to the samples truth.jl uses, so that they change smoothly.
