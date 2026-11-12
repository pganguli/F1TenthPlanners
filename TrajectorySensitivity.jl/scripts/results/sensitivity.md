# Trajectory sensitivity to each estimate

From [`sensitivity.jl`](../sensitivity.jl), on the representative track ([track.md](track.md)),
2 laps from a standing start. The reference (every estimate true and on time) takes
64.97 s (32.5 s a lap, 5.51 m/s on average),
its body at least 0.255 m from a wall. Each run changes one estimate, the others
true and on time, and is compared with the reference at the same distance along the centerline:
Δd is the change in the car's lateral position, Δv in its speed, Δlap in the time for the 2 laps
(– after a crash). Per run in [runs.csv](runs.csv).

## Accuracy

The estimate with a network's errors (drawn as [error_model.md](error_model.md) checks) times a
gain, from the car at every scan, with the σ such a network would give; gain 1 of small_weak is the
car's network, gain 1 of large the cloud's (here without its delay). 20
runs (error seeds) each. Max |Δd|: the median over the runs (and the worst). Tube: the width of the
band all runs' lateral positions lie in, at each distance along the track: its mean and greatest.

| Estimate | Errors | Gain | Max \|Δd\| median (worst) [m] | Max \|Δv\| median [m/s] | Δlap median [s] | Least gap [m] | Crashes | Tube mean / max [m] |
|---|---|---|---|---|---|---|---|---|
| `e` | small_weak | 0.5 | 0.0285 (0.0398) | 0.139 | 0.0573 | 0.25 | 0 / 20 | 0.0106 / 0.0475 |
| `e` | small_weak | 1.0 | 0.0368 (0.0485) | 0.183 | 0.0849 | 0.25 | 0 / 20 | 0.0187 / 0.0612 |
| `e` | small_weak | 2.0 | 0.0472 (0.059) | 0.228 | 0.0954 | 0.25 | 0 / 20 | 0.0339 / 0.0978 |
| `e` | small_weak | 4.0 | 0.068 (0.12) | 0.423 | 0.163 | 0.25 | 0 / 20 | 0.0604 / 0.156 |
| `e` | small_weak | 8.0 | 0.132 (0.157) | 0.631 | 0.174 | 0.212 | 0 / 20 | 0.101 / 0.224 |
| `e` | small_weak | 16.0 | 0.219 (0.486) | 0.661 | 0.225 | 0.206 | 0 / 20 | 0.161 / 0.528 |
| `e` | large | 1.0 | 0.0291 (0.0466) | 0.171 | 0.0781 | 0.25 | 0 / 20 | 0.0164 / 0.0523 |
| `ψe` | small_weak | 0.5 | 0.0542 (0.0916) | 0.314 | 0.123 | 0.25 | 0 / 20 | 0.0313 / 0.133 |
| `ψe` | small_weak | 1.0 | 0.0972 (0.18) | 0.447 | 0.201 | 0.158 | 0 / 20 | 0.0546 / 0.233 |
| `ψe` | small_weak | 2.0 | 0.15 (0.342) | 0.535 | 0.353 | 0.0707 | 0 / 20 | 0.0809 / 0.489 |
| `ψe` | small_weak | 4.0 | 0.186 (0.279) | 0.708 | 0.595 | 0.212 | 0 / 20 | 0.0805 / 0.365 |
| `ψe` | small_weak | 8.0 | 0.24 (0.591) | 1.3 | 1.17 | 0.25 | 0 / 20 | 0.0617 / 0.642 |
| `ψe` | small_weak | 16.0 | 0.632 (1.05) | 2.35 | 10.7 | 0.0 | 17 / 20 | 0.048 / 0.846 |
| `ψe` | large | 1.0 | 0.0734 (0.115) | 0.318 | 0.154 | 0.212 | 0 / 20 | 0.0375 / 0.173 |
| `κ_profile` | small_weak | 0.5 | 0.725 (1.21) | 2.33 | 2.63 | 0.0 | 10 / 20 | 0.207 / 1.54 |
| `κ_profile` | small_weak | 1.0 | 0.962 (1.33) | 2.26 | 6.45 | 0.0 | 18 / 20 | 0.262 / 1.63 |
| `κ_profile` | small_weak | 2.0 | 0.927 (1.35) | 2.46 | – | 0.0 | 20 / 20 | 0.263 / 1.63 |
| `κ_profile` | small_weak | 4.0 | 1.03 (1.4) | 2.91 | – | 0.0 | 20 / 20 | 0.357 / 1.58 |
| `κ_profile` | small_weak | 8.0 | 0.937 (1.39) | 3.06 | – | 0.0 | 20 / 20 | 0.576 / 1.62 |
| `κ_profile` | small_weak | 16.0 | 1.04 (1.31) | 4.49 | – | 0.0 | 20 / 20 | 0.602 / 1.79 |
| `κ_profile` | large | 1.0 | 0.683 (1.16) | 2.3 | 4.32 | 0.0 | 11 / 20 | 0.246 / 1.85 |
| `widths` | small_weak | 0.5 | 0.00309 (0.0379) | 0.0316 | -0.000181 | 0.255 | 0 / 20 | 0.00249 / 0.0382 |
| `widths` | small_weak | 1.0 | 0.0428 (0.237) | 0.216 | 0.0393 | 0.255 | 0 / 20 | 0.0139 / 0.292 |
| `widths` | small_weak | 2.0 | 0.341 (0.529) | 0.78 | 0.268 | 0.255 | 0 / 20 | 0.0421 / 0.668 |
| `widths` | small_weak | 4.0 | 0.45 (0.584) | 1.19 | 0.689 | 0.25 | 0 / 20 | 0.0724 / 0.512 |
| `widths` | small_weak | 8.0 | 0.523 (0.721) | 2.18 | 1.9 | 0.2 | 0 / 20 | 0.155 / 0.79 |
| `widths` | small_weak | 16.0 | 0.677 (0.867) | 2.85 | 3.3 | 0.05 | 0 / 20 | 0.19 / 0.886 |
| `widths` | large | 1.0 | 0.0688 (0.221) | 0.22 | 0.0504 | 0.255 | 0 / 20 | 0.0119 / 0.298 |
| `sight` | small_weak | 0.5 | 0.129 (0.539) | 0.236 | 0.211 | 0.224 | 0 / 20 | 0.0312 / 0.575 |
| `sight` | small_weak | 1.0 | 0.239 (0.53) | 0.577 | 0.502 | 0.1 | 0 / 20 | 0.0493 / 0.62 |
| `sight` | small_weak | 2.0 | 0.452 (0.657) | 0.917 | 1.14 | 0.1 | 0 / 20 | 0.0729 / 0.769 |
| `sight` | small_weak | 4.0 | 0.515 (0.781) | 1.19 | 2.49 | 0.1 | 0 / 20 | 0.0951 / 0.814 |
| `sight` | small_weak | 8.0 | 0.599 (0.98) | 1.99 | 4.3 | 0.0707 | 0 / 20 | 0.113 / 1.19 |
| `sight` | small_weak | 16.0 | 0.621 (0.976) | 2.26 | 8.18 | 0.0707 | 0 / 20 | 0.109 / 0.983 |
| `sight` | large | 1.0 | 0.112 (0.397) | 0.253 | 0.227 | 0.206 | 0 / 20 | 0.0271 / 0.474 |
| `clearance` | small_weak | 0.5 | 0.0592 (0.158) | 0.99 | 0.597 | 0.25 | 0 / 20 | 0.0152 / 0.16 |
| `clearance` | small_weak | 1.0 | 0.0832 (0.21) | 1.44 | 1.69 | 0.25 | 0 / 20 | 0.0221 / 0.295 |
| `clearance` | small_weak | 2.0 | 0.123 (0.266) | 1.89 | 4.86 | 0.25 | 0 / 20 | 0.0302 / 0.241 |
| `clearance` | small_weak | 4.0 | 0.186 (0.323) | 2.6 | 8.94 | 0.224 | 0 / 20 | 0.0425 / 0.541 |
| `clearance` | small_weak | 8.0 | 0.317 (0.404) | 4.97 | 23.8 | 0.206 | 0 / 20 | 0.0714 / 0.394 |
| `clearance` | small_weak | 16.0 | 0.55 (0.605) | 6.36 | – | 0.2 | 0 / 20 | 0.0799 / 0.392 |
| `clearance` | large | 1.0 | 0.0518 (0.0712) | 0.343 | 0.518 | 0.25 | 0 / 20 | 0.00775 / 0.0664 |

Where on the track: the tube's greatest width [m] per kind of segment, with the car network's errors
(small_weak, gain 1).

| Estimate | long straight | hairpin | medium corner | fast sweeper | chicane | tight hairpin | tightening corner | double apex | straight |
|---|---|---|---|---|---|---|---|---|---|
| `e` | 0.0318 | 0.0506 | 0.0612 | 0.0397 | 0.0268 | 0.0557 | 0.0407 | 0.0486 | 0.0412 |
| `ψe` | 0.165 | 0.127 | 0.123 | 0.109 | 0.178 | 0.23 | 0.183 | 0.164 | 0.233 |
| `κ_profile` | 0.592 | 1.55 | 1.43 | 0.811 | 1.09 | 0.842 | 1.5 | 1.63 | 1.27 |
| `widths` | 0.0238 | 0.0395 | 0.0795 | 0.292 | 0.0565 | 0.0405 | 0.029 | 0.029 | 0.152 |
| `sight` | 0.137 | 0.322 | 0.539 | 0.0655 | 0.0085 | 0.62 | 0.334 | 0.545 | 0.419 |
| `clearance` | 0.0607 | 0.295 | 0.0734 | 0.114 | 0.112 | 0.0971 | 0.105 | 0.0965 | 0.196 |

## Latency

The estimate true, but only from the cloud (as the selector and the Kalman filter handle a cloud
result: counted at its scan's time, the scans after it filtered again), arriving a delay after its
scan; the car's network gives none. The selector drops results older than 100 ms, so past that the
estimate is never measured: the filter only predicts it (e, ψe, κ) or keeps its last value (the rest),
and without e and ψe the lane is never found and the car stays put.

| Estimate | 0 ms | 25 ms | 50 ms | 75 ms | 100 ms | 125 ms |
|---|---|---|---|---|---|---|
| `e` | 0.00909 m, 0.000902 s | 0.0323 m, 0.0552 s | 0.0374 m, 0.0851 s | 0.0441 m, 0.164 s | 0.0537 m, 0.202 s | no start |
| `ψe` | 0.00813 m, -0.000532 s | 0.0415 m, 0.065 s | 0.0444 m, 0.0785 s | 0.0655 m, 0.204 s | 0.0701 m, 0.327 s | no start |
| `κ_profile` | 0.00983 m, -0.0298 s | 0.0346 m, 0.155 s | 0.0425 m, 0.282 s | 0.0513 m, 0.323 s | 0.072 m, 0.306 s | 0.863 m, 10.3 s |
| `widths` | 0.0 m, 0.0 s | 0.02 m, 0.128 s | 0.0277 m, 0.172 s | 0.0351 m, 0.198 s | 0.0377 m, 0.223 s | 0.864 m, 10.4 s |
| `sight` | 0.0262 m, 0.0276 s | 0.0505 m, 0.281 s | 0.0805 m, 0.559 s | 0.0879 m, 0.772 s | 0.0916 m, 1.19 s | 0.511 m, – s |
| `clearance` | 0.0223 m, -4.72e-5 s | 0.0226 m, 0.21 s | 0.0316 m, 0.496 s | 0.0814 m, 2.85 s | 0.074 m, 3.38 s | 0.608 m, – s |

Each cell: max |Δd|, and Δlap.
