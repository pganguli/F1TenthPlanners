# Racing the held-out tracks: safety first (small_weak on the car)

Networks: `small_weak` on the car (the small networks), `large` in the cloud, from
`../../LidarLaneNets.jl/weights`.

Tracks: Spa, Monza, Silverstone, Oschersleben, YasMarina. Each configuration: one lap of each track per seed, 8 seeds
(40 runs). A seed sets the LiDAR's noise and where the lap starts. Values are means over
the runs with 95% confidence intervals. Every run's numbers are in [runs-small_weak.csv](runs-small_weak.csv).

## Safety

From the true map and state. Time to collision: the true free distance along the arc the car is
driving, over its speed (above 1 m/s); least over each run. Planning on too much free distance: the
free distance ahead the speed was set for exceeds the true one, along the same arcs from where the
scan was taken, by more than 0.25 m.

| Configuration | Cloud latency [ms] | Crashes | Closest to a wall, worst run [m] | Closest to a wall [m] | Time within 0.2 m of a wall [%] | Scans with time to collision under 0.5 s [%] | Least time to collision [s] | Largest slip angle [°] | Scans planning on too much free distance [%] |
|---|---|---|---|---|---|---|---|---|---|
| small only | – | 8 | 0.07 | 0.27 ± 0.04 | 0.24 ± 0.21 | 0.95 ± 1.0 | 0.43 ± 0.06 | 4.7 ± 0.3 | 5.97 ± 0.78 |
| large only | 25 | 0 | 0.1 | 0.31 ± 0.03 | 0.09 ± 0.06 | 0.04 ± 0.03 | 0.56 ± 0.02 | 4.9 ± 0.3 | 5.79 ± 0.45 |
| both, every scan | 25 | 0 | 0.21 | 0.37 ± 0.01 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.55 ± 0.01 | 5.2 ± 0.3 | 6.37 ± 0.49 |
| round robin | 25 | 4 | 0.07 | 0.29 ± 0.04 | 0.18 ± 0.12 | 0.32 ± 0.26 | 0.46 ± 0.05 | 4.9 ± 0.3 | 6.98 ± 0.78 |
| far ahead | 25 | 0 | 0.21 | 0.39 ± 0.02 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.59 ± 0.01 | 4.5 ± 0.3 | 4.68 ± 0.51 |
| large only | 50 | 0 | 0.1 | 0.32 ± 0.04 | 0.16 ± 0.11 | 0.14 ± 0.09 | 0.55 ± 0.04 | 4.8 ± 0.3 | 5.25 ± 0.44 |
| both, every scan | 50 | 0 | 0.21 | 0.36 ± 0.02 | 0.0 ± 0.0 | 0.01 ± 0.01 | 0.54 ± 0.01 | 5.2 ± 0.3 | 6.58 ± 0.51 |
| round robin | 50 | 8 | 0.07 | 0.29 ± 0.04 | 0.29 ± 0.32 | 0.74 ± 0.76 | 0.45 ± 0.05 | 4.9 ± 0.3 | 7.6 ± 1.21 |
| far ahead | 50 | 0 | 0.16 | 0.36 ± 0.03 | 0.01 ± 0.02 | 0.02 ± 0.02 | 0.56 ± 0.01 | 4.5 ± 0.3 | 4.93 ± 0.6 |
| large only | 100 | 5 | 0.1 | 0.35 ± 0.04 | 0.3 ± 0.2 | 0.4 ± 0.29 | 0.53 ± 0.05 | 5.0 ± 0.4 | 4.76 ± 0.45 |
| both, every scan | 100 | 8 | 0.07 | 0.29 ± 0.04 | 0.32 ± 0.33 | 0.86 ± 0.92 | 0.44 ± 0.05 | 5.0 ± 0.3 | 8.1 ± 1.45 |
| round robin | 100 | 8 | 0.07 | 0.27 ± 0.05 | 0.34 ± 0.28 | 1.09 ± 1.08 | 0.42 ± 0.06 | 4.9 ± 0.3 | 7.57 ± 0.88 |
| far ahead | 100 | 8 | 0.07 | 0.3 ± 0.05 | 0.27 ± 0.27 | 0.8 ± 0.86 | 0.46 ± 0.06 | 4.5 ± 0.3 | 6.4 ± 1.58 |

### Against small only, run for run

Each run minus small only's run on the same track, seed and start (pp: percentage points). An
interval that excludes 0 is a difference the runs support.

| Configuration | Cloud latency [ms] | Closest to a wall [m] (more is safer) | Time within 0.2 m of a wall [pp] | Scans with time to collision under 0.5 s [pp] | Largest slip angle [°] | Scans planning on too much free distance [pp] |
|---|---|---|---|---|---|---|
| large only | 25 | 0.04 ± 0.03 | -0.15 ± 0.21 | -0.91 ± 1.0 | 0.2 ± 0.2 | -0.18 ± 0.59 |
| both, every scan | 25 | 0.1 ± 0.04 | -0.24 ± 0.21 | -0.95 ± 1.0 | 0.6 ± 0.2 | 0.41 ± 0.43 |
| round robin | 25 | 0.02 ± 0.01 | -0.05 ± 0.22 | -0.63 ± 0.97 | 0.3 ± 0.2 | 1.02 ± 0.35 |
| far ahead | 25 | 0.12 ± 0.03 | -0.24 ± 0.21 | -0.95 ± 1.0 | -0.2 ± 0.2 | -1.28 ± 0.38 |
| large only | 50 | 0.06 ± 0.03 | -0.07 ± 0.23 | -0.81 ± 1.01 | 0.1 ± 0.2 | -0.72 ± 0.58 |
| both, every scan | 50 | 0.09 ± 0.03 | -0.24 ± 0.21 | -0.94 ± 1.0 | 0.5 ± 0.2 | 0.61 ± 0.4 |
| round robin | 50 | 0.02 ± 0.01 | 0.05 ± 0.13 | -0.21 ± 0.25 | 0.3 ± 0.1 | 1.63 ± 0.58 |
| far ahead | 50 | 0.09 ± 0.02 | -0.22 ± 0.21 | -0.93 ± 1.0 | -0.2 ± 0.2 | -1.04 ± 0.34 |
| large only | 100 | 0.09 ± 0.04 | 0.07 ± 0.29 | -0.56 ± 1.06 | 0.3 ± 0.4 | -1.21 ± 0.49 |
| both, every scan | 100 | 0.02 ± 0.01 | 0.08 ± 0.14 | -0.09 ± 0.09 | 0.3 ± 0.2 | 2.13 ± 0.83 |
| round robin | 100 | 0.0 ± 0.01 | 0.11 ± 0.09 | 0.14 ± 0.12 | 0.2 ± 0.1 | 1.6 ± 0.26 |
| far ahead | 100 | 0.03 ± 0.01 | 0.03 ± 0.09 | -0.15 ± 0.14 | -0.2 ± 0.2 | 0.44 ± 0.96 |

## Lap times and tracking

| Configuration | Cloud latency [ms] | Laps finished | Lap time [s] | Lap time vs small only, paired [%] | e error [m] | ψe error [°] | Cloud calls per scan |
|---|---|---|---|---|---|---|---|
| small only | – | 32 / 40 | 78.8 ± 6.73 | – | 0.0132 ± 0.0034 | 1.98 ± 0.83 | 0.0 |
| large only | 25 | 40 / 40 | 75.31 ± 4.96 | -4.5 ± 0.25 | 0.01 ± 0.0008 | 1.75 ± 0.38 | 6.0 |
| both, every scan | 25 | 40 / 40 | 75.27 ± 5.01 | -4.69 ± 0.15 | 0.0089 ± 0.0006 | 1.22 ± 0.22 | 6.0 |
| round robin | 25 | 36 / 40 | 76.68 ± 5.74 | -2.87 ± 0.09 | 0.0102 ± 0.0011 | 1.32 ± 0.25 | 1.0 |
| far ahead | 25 | 40 / 40 | 78.14 ± 5.25 | -0.88 ± 0.14 | 0.0093 ± 0.0008 | 1.25 ± 0.22 | 3.0 |
| large only | 50 | 40 / 40 | 77.31 ± 5.12 | -1.97 ± 0.29 | 0.0123 ± 0.0009 | 1.96 ± 0.39 | 6.0 |
| both, every scan | 50 | 40 / 40 | 75.92 ± 5.08 | -3.94 ± 0.12 | 0.0093 ± 0.0007 | 1.26 ± 0.23 | 6.0 |
| round robin | 50 | 32 / 40 | 76.83 ± 6.53 | -2.48 ± 0.13 | 0.0115 ± 0.0022 | 1.44 ± 0.37 | 1.0 |
| far ahead | 50 | 40 / 40 | 78.3 ± 5.24 | -0.8 ± 0.14 | 0.0096 ± 0.0009 | 1.26 ± 0.23 | 3.0 |
| large only | 100 | 35 / 40 | 79.97 ± 6.04 | 1.12 ± 0.44 | 0.0186 ± 0.0011 | 2.59 ± 0.43 | 6.0 |
| both, every scan | 100 | 32 / 40 | 76.58 ± 6.51 | -2.8 ± 0.1 | 0.0109 ± 0.0018 | 1.36 ± 0.3 | 6.0 |
| round robin | 100 | 32 / 40 | 77.51 ± 6.58 | -1.59 ± 0.13 | 0.0123 ± 0.0029 | 1.66 ± 0.59 | 1.0 |
| far ahead | 100 | 32 / 40 | 78.4 ± 6.68 | -0.5 ± 0.09 | 0.0115 ± 0.0023 | 1.36 ± 0.31 | 3.0 |
