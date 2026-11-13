# The simulation's sensitivities, from the linearized closed loop

No simulation and no map, but the same questions as the simulation campaigns
([empirical.toml](empirical.toml), from the simulation branch's sensitivity.md): the RMS lateral
deviation from the reference run with the car network's errors on one estimate (accuracy), and its
growth per 25 ms the estimate comes late from the cloud, true (latency). From
[`matched.jl`](../matched.jl), which says how each part is modeled: analytic.jl's linearized closed
loop with LidarLaneNets.jl's Kalman filter in it (as the car runs it), driven by random roads (each
situation's lane bending about its curvature as much as the racetracks' do ahead of the car in that
situation, decorrelating over 2.02 m) and by the network's recorded errors' statistics at each
situation, and its stationary covariance (the Lyapunov equation).
Each situation weighted by how much of the 23 racetracks is like it: a lane of constant curvature, or,
within 10 m (the range of clearance and sight), a straight with a bend ahead or a bend with a straight
ahead (the scene held as it is about the car); 0.1% of the tracks' length, in situations without a steady state, counted as the nearest that has one. Where the speed loop
is unstable in small signal (tight bends), the speed command goes through the command limiter's ramp's
describing function (a low-pass, the shortest that makes the loop just stable; "ramp" below), as on the
car, where the ramp holds the speed to a chatter of a few cm/s about its steady value.

## Against the simulation

| Estimate | Accuracy: analytic [m] | simulation [m] | ratio | Latency: analytic [m per 25 ms] | simulation | ratio |
|---|---|---|---|---|---|---|
| `e` | 0.00282 | 0.0042 (0.0041–0.0043) | 0.67 | 0.00268 | 0.0024 (0.0022–0.0027) | 1.1 |
| `ψe` | 0.01 | 0.012 (0.011–0.012) | 0.83 | 0.00426 | 0.0032 (0.0028–0.0036) | 1.3 |
| `κ_profile` | 0.0511 | 0.05 (0.048–0.052) | 1 | 0.00286 | 0.0032 (0.0029–0.0034) | 0.89 |
| `widths` | 0.000994 | 0.0052 (0.0042–0.0062) | 0.19 | 0.00251 | 0.0025 (0.0023–0.0027) | 1 |
| `sight` | 0.00362 | 0.0079 (0.0073–0.0086) | 0.46 | 0.00291 | 0.0027 (0.0024–0.003) | 1.1 |
| `clearance` | 0.0122 | 0.016 (0.015–0.016) | 0.76 | 0.00414 | 0.0036 (0.0033–0.0039) | 1.1 |

## By situation

Accuracy [m] / latency [m per 25 ms], per estimate.

| Situation | width [m] | share of the tracks [%] | road: σκ [1/m] | speed [m/s] | loop | `e` | `ψe` | `κ_profile` | `widths` | `sight` | `clearance` |
|---|---|---|---|---|---|---|---|---|---|---|---|
| κ 0.0 | 2.26 | 34.0 | 0.0013 | 7.24 | stable | 0.00252 / 6.42e-5 | 0.00508 / 1.13e-5 | 0.0107 / 3.8e-5 | 0.0 / 0.0 | 0.0 / 0.0 | 0.0 / 0.0 |
| κ 0.02 | 2.26 | 14.0 | 0.059 | 6.59 | stable | 0.00239 / 0.0024 | 0.00991 / 0.000357 | 0.0797 / 0.00262 | 0.0 / 0.00222 | 0.0 / 0.0 | 0.00495 / 0.00184 |
| κ 0.05 | 2.26 | 11.0 | 0.071 | 5.24 | stable | 0.00196 / 0.00177 | 0.00875 / 0.000428 | 0.0452 / 0.00183 | 0.0 / 0.00148 | 0.0 / 0.0 | 0.0158 / 0.0056 |
| κ 0.1 | 2.26 | 1.9 | 0.035 | 4.48 | stable | 0.0018 / 0.00209 | 0.00686 / 0.000792 | 0.0248 / 0.000444 | 0.0 / 0.000126 | 8.45e-6 / 0.0 | 0.0228 / 0.00721 |
| κ 0.15 | 2.26 | 0.99 | 0.04 | 5.02 | ramp (0.26 s) | 0.00564 / 0.00725 | 0.0353 / 0.00213 | 0.033 / 0.00119 | 6.59e-10 / 0.0 | 0.0145 / 0.00643 | 0.0396 / 0.0121 |
| κ 0.2 | 2.26 | 0.51 | 0.053 | 4.82 | ramp (0.83 s) | 0.00489 / 0.00602 | 0.0165 / 0.00405 | 0.0362 / 0.0014 | 0.0 / 0.0 | 0.00979 / 0.00386 | 0.00323 / 0.0 |
| κ 0.3 | 2.26 | 0.49 | 0.079 | 3.82 | ramp (0.58 s) | 0.00444 / 0.00436 | 0.0147 / 0.00726 | 0.0597 / 0.00159 | 0.0 / 0.0 | 0.000176 / 2.88e-5 | 0.0 / 0.0 |
| straight, κ 0.1 bend 2 m ahead | 2.26 | 0.97 | 0.021 | 5.86 | stable | 0.00253 / 0.00076 | 0.008 / 0.000117 | 0.0282 / 0.00707 | 0.0 / 0.00706 | 0.0 / 0.0 | 0.0252 / 0.00913 |
| straight, κ 0.1 bend 5 m ahead | 2.26 | 1.2 | 0.0041 | 7.24 | stable | 0.00248 / 0.000207 | 0.0052 / 3.63e-5 | 0.0154 / 0.00624 | 0.0 / 0.00624 | 0.000794 / 0.00107 | 0.00361 / 0.00306 |
| straight, κ 0.1 bend 8 m ahead | 2.26 | 1.2 | 0.0026 | 7.24 | stable | 0.00254 / 0.00013 | 0.00511 / 2.3e-5 | 0.0122 / 7.7e-5 | 0.0 / 2.21e-6 | 1.29e-7 / 1.75e-7 | 3.9e-7 / 5.01e-7 |
| straight, κ 0.2 bend 2 m ahead | 2.26 | 1.1 | 0.032 | 5.28 | stable | 0.00227 / 0.000813 | 0.00747 / 0.000132 | 0.0537 / 0.000554 | 0.0 / 0.0 | 0.0124 / 0.00808 | 0.031 / 0.0121 |
| straight, κ 0.2 bend 5 m ahead | 2.26 | 1.1 | 0.0028 | 6.39 | stable | 0.00244 / 0.000103 | 0.0049 / 1.62e-5 | 0.0108 / 0.00733 | 0.0 / 0.00733 | 0.0 / 0.0 | 0.00655 / 0.00271 |
| straight, κ 0.2 bend 8 m ahead | 2.26 | 0.98 | 0.0016 | 7.24 | stable | 0.00252 / 8.03e-5 | 0.00509 / 1.42e-5 | 0.011 / 4.77e-5 | 0.0 / 4.41e-6 | 2.58e-7 / 3.49e-7 | 7.8e-7 / 1.0e-6 |
| straight, κ 0.3 bend 2 m ahead | 2.26 | 3.8 | 0.11 | 4.97 | stable | 0.00215 / 0.00233 | 0.00777 / 0.000363 | 0.0864 / 0.00176 | 0.0 / 0.0 | 0.0118 / 0.013 | 0.0365 / 0.0122 |
| straight, κ 0.3 bend 5 m ahead | 2.26 | 3.7 | 0.0025 | 6.18 | stable | 0.00234 / 8.44e-5 | 0.00492 / 1.28e-5 | 0.0549 / 0.00987 | 0.00518 / 0.00987 | 0.0 / 0.0 | 0.00985 / 0.00399 |
| straight, κ 0.3 bend 8 m ahead | 2.26 | 3.7 | 0.0017 | 7.24 | stable | 0.00252 / 8.42e-5 | 0.00509 / 1.48e-5 | 0.0111 / 5.03e-5 | 0.0 / 6.62e-6 | 3.86e-7 / 5.24e-7 | 1.17e-6 / 1.5e-6 |
| κ 0.1 bend, straight 2 m ahead | 2.26 | 4.0 | 0.078 | 4.71 | stable | 0.00178 / 0.00247 | 0.00888 / 0.000798 | 0.0365 / 0.00101 | 0.0 / 4.46e-5 | 1.86e-9 / -5.59e-10 | 0.0062 / 0.00213 |
| κ 0.1 bend, straight 5 m ahead | 2.26 | 2.2 | 0.088 | 4.48 | stable | 0.00179 / 0.0023 | 0.00805 / 0.00132 | 0.0618 / 0.00106 | 0.0 / 0.000171 | 0.0 / 0.0 | 0.0199 / 0.00664 |
| κ 0.1 bend, straight 8 m ahead | 2.26 | 1.6 | 0.048 | 4.48 | stable | 0.00181 / 0.00212 | 0.00722 / 0.000908 | 0.0317 / 0.000587 | 1.86e-9 / 0.000126 | 8.45e-6 / 0.0 | 0.0227 / 0.00721 |
| κ 0.2 bend, straight 2 m ahead | 2.26 | 2.4 | 0.098 | 4.65 | stable | 0.00233 / 0.0056 | 0.011 / 0.00425 | 0.0975 / 0.00133 | 1.86e-9 / 0.000343 | 0.0 / 0.0 | 0.00622 / 0.000674 |
| κ 0.2 bend, straight 5 m ahead | 2.26 | 1.8 | 0.086 | 4.82 | ramp (0.83 s) | 0.0054 / 0.00691 | 0.0195 / 0.00531 | 0.0412 / 0.00228 | 0.0 / 0.0 | 0.0117 / 0.00454 | 0.0108 / 0.0 |
| κ 0.2 bend, straight 8 m ahead | 2.26 | 1.1 | 0.061 | 4.82 | ramp (0.83 s) | 0.00501 / 0.00619 | 0.0172 / 0.00435 | 0.0373 / 0.00161 | 0.0 / 0.0 | 0.00979 / 0.00386 | 0.00323 / 0.0 |
| κ 0.3 bend, straight 2 m ahead | 2.26 | 3.9 | 0.22 | 3.88 | ramp (0.58 s) | 0.00533 / 0.00681 | 0.0226 / 0.0191 | 0.0949 / 0.00538 | 0.0 / 0.00234 | 0.0 / -6.21e-10 | 0.0 / -6.21e-10 |
| κ 0.3 bend, straight 5 m ahead | 2.26 | 1.9 | 0.14 | 3.82 | ramp (0.58 s) | 0.00511 / 0.00486 | 0.0184 / 0.00928 | 0.0832 / 0.00278 | 0.0 / 0.0 | 0.000353 / 2.88e-5 | 0.00201 / 0.0 |
| κ 0.3 bend, straight 8 m ahead | 2.26 | 0.82 | 0.089 | 3.82 | ramp (0.58 s) | 0.00451 / 0.00442 | 0.0153 / 0.00753 | 0.0633 / 0.00178 | 0.0 / 0.0 | 0.000176 / 2.88e-5 | 0.0 / 0.0 |

## What it leaves out

Crashes (a linear model has none), the speed planner's switches between its limits, the robust
(Student t) updates and the restarts of the tracker, the controller's steps other than the two computed
exactly (a profile's far end lost to a delay; sight and clearance taken k_σ σ short), errors that depend
on more than the lane (the matching sees the true lane only), and bends that tighten or open (every
situation is a lane of constant curvature, its road varying about it).
