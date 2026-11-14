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
| small only | – | 8 | 0.07 | 0.27 ± 0.04 | 0.24 ± 0.21 | 0.95 ± 1.0 | 0.44 ± 0.06 | 4.6 ± 0.3 | 5.8 ± 0.81 |
| large only | 25 | 0 | 0.1 | 0.34 ± 0.03 | 0.07 ± 0.05 | 0.03 ± 0.03 | 0.57 ± 0.02 | 4.7 ± 0.3 | 5.47 ± 0.44 |
| both, every scan | 25 | 0 | 0.19 | 0.38 ± 0.02 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.56 ± 0.01 | 5.1 ± 0.3 | 6.11 ± 0.46 |
| round robin | 25 | 3 | 0.07 | 0.3 ± 0.04 | 0.14 ± 0.09 | 0.24 ± 0.19 | 0.48 ± 0.04 | 4.9 ± 0.3 | 6.78 ± 0.73 |
| far ahead | 25 | 0 | 0.27 | 0.39 ± 0.01 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.6 ± 0.01 | 4.5 ± 0.3 | 4.47 ± 0.51 |
| large only | 50 | 0 | 0.1 | 0.33 ± 0.04 | 0.16 ± 0.11 | 0.15 ± 0.1 | 0.54 ± 0.05 | 4.8 ± 0.3 | 5.07 ± 0.45 |
| both, every scan | 50 | 0 | 0.19 | 0.36 ± 0.03 | 0.02 ± 0.03 | 0.0 ± 0.0 | 0.55 ± 0.01 | 5.1 ± 0.3 | 6.36 ± 0.5 |
| round robin | 50 | 7 | 0.07 | 0.3 ± 0.04 | 0.27 ± 0.32 | 0.69 ± 0.75 | 0.44 ± 0.06 | 5.0 ± 0.3 | 7.33 ± 1.2 |
| far ahead | 50 | 0 | 0.19 | 0.36 ± 0.02 | 0.01 ± 0.01 | 0.01 ± 0.01 | 0.58 ± 0.01 | 4.5 ± 0.3 | 4.77 ± 0.57 |
| large only | 100 | 4 | 0.1 | 0.35 ± 0.04 | 0.58 ± 0.63 | 0.85 ± 1.09 | 0.53 ± 0.05 | 5.0 ± 0.5 | 4.91 ± 0.54 |
| both, every scan | 100 | 8 | 0.07 | 0.3 ± 0.05 | 0.29 ± 0.32 | 0.83 ± 0.92 | 0.45 ± 0.05 | 5.0 ± 0.4 | 7.76 ± 1.45 |
| round robin | 100 | 8 | 0.07 | 0.28 ± 0.05 | 0.28 ± 0.28 | 0.97 ± 1.06 | 0.43 ± 0.05 | 4.8 ± 0.3 | 7.38 ± 0.88 |
| far ahead | 100 | 8 | 0.07 | 0.3 ± 0.05 | 0.25 ± 0.27 | 0.78 ± 0.86 | 0.46 ± 0.06 | 4.5 ± 0.3 | 6.34 ± 1.57 |

### Against small only, run for run

Each run minus small only's run on the same track, seed and start (pp: percentage points). An
interval that excludes 0 is a difference the runs support.

| Configuration | Cloud latency [ms] | Closest to a wall [m] (more is safer) | Time within 0.2 m of a wall [pp] | Scans with time to collision under 0.5 s [pp] | Largest slip angle [°] | Scans planning on too much free distance [pp] |
|---|---|---|---|---|---|---|
| large only | 25 | 0.07 ± 0.03 | -0.17 ± 0.22 | -0.91 ± 1.0 | 0.1 ± 0.2 | -0.34 ± 0.61 |
| both, every scan | 25 | 0.11 ± 0.04 | -0.24 ± 0.21 | -0.95 ± 1.0 | 0.4 ± 0.1 | 0.31 ± 0.51 |
| round robin | 25 | 0.03 ± 0.01 | -0.09 ± 0.21 | -0.71 ± 0.98 | 0.3 ± 0.2 | 0.98 ± 0.42 |
| far ahead | 25 | 0.12 ± 0.03 | -0.24 ± 0.21 | -0.95 ± 1.0 | -0.1 ± 0.1 | -1.33 ± 0.41 |
| large only | 50 | 0.06 ± 0.04 | -0.07 ± 0.24 | -0.8 ± 1.01 | 0.1 ± 0.3 | -0.73 ± 0.6 |
| both, every scan | 50 | 0.09 ± 0.02 | -0.22 ± 0.21 | -0.95 ± 1.0 | 0.5 ± 0.2 | 0.56 ± 0.44 |
| round robin | 50 | 0.03 ± 0.01 | 0.04 ± 0.12 | -0.25 ± 0.27 | 0.4 ± 0.2 | 1.53 ± 0.58 |
| far ahead | 50 | 0.09 ± 0.03 | -0.23 ± 0.22 | -0.94 ± 1.0 | -0.2 ± 0.1 | -1.03 ± 0.39 |
| large only | 100 | 0.08 ± 0.04 | 0.34 ± 0.68 | -0.09 ± 1.52 | 0.4 ± 0.5 | -0.89 ± 0.67 |
| both, every scan | 100 | 0.03 ± 0.01 | 0.05 ± 0.13 | -0.12 ± 0.09 | 0.3 ± 0.2 | 1.96 ± 0.82 |
| round robin | 100 | 0.01 ± 0.01 | 0.04 ± 0.1 | 0.02 ± 0.12 | 0.2 ± 0.1 | 1.58 ± 0.25 |
| far ahead | 100 | 0.03 ± 0.01 | 0.01 ± 0.09 | -0.17 ± 0.15 | -0.1 ± 0.1 | 0.54 ± 0.94 |

## Lap times and tracking

| Configuration | Cloud latency [ms] | Laps finished | Lap time [s] | Lap time vs small only, paired [%] | e error [m] | ψe error [°] | Cloud calls per scan |
|---|---|---|---|---|---|---|---|
| small only | – | 32 / 40 | 80.03 ± 6.71 | – | 0.0131 ± 0.0034 | 1.93 ± 0.81 | 0.0 |
| large only | 25 | 40 / 40 | 78.74 ± 5.21 | -1.71 ± 0.21 | 0.0097 ± 0.0008 | 1.7 ± 0.38 | 6.0 |
| both, every scan | 25 | 40 / 40 | 78.46 ± 5.2 | -2.2 ± 0.17 | 0.0088 ± 0.0006 | 1.19 ± 0.22 | 6.0 |
| round robin | 25 | 37 / 40 | 79.15 ± 5.69 | -1.38 ± 0.13 | 0.0099 ± 0.001 | 1.27 ± 0.23 | 1.0 |
| far ahead | 25 | 40 / 40 | 79.57 ± 5.26 | -0.68 ± 0.08 | 0.0093 ± 0.0008 | 1.24 ± 0.21 | 3.0 |
| large only | 50 | 40 / 40 | 79.74 ± 5.28 | -0.48 ± 0.28 | 0.012 ± 0.0008 | 1.93 ± 0.39 | 6.0 |
| both, every scan | 50 | 40 / 40 | 78.76 ± 5.22 | -1.9 ± 0.17 | 0.0091 ± 0.0007 | 1.23 ± 0.22 | 6.0 |
| round robin | 50 | 33 / 40 | 79.16 ± 6.43 | -1.19 ± 0.11 | 0.011 ± 0.0021 | 1.41 ± 0.37 | 1.0 |
| far ahead | 50 | 40 / 40 | 79.72 ± 5.27 | -0.56 ± 0.09 | 0.0096 ± 0.0009 | 1.25 ± 0.22 | 3.0 |
| large only | 100 | 36 / 40 | 81.34 ± 6.0 | 1.42 ± 0.33 | 0.0187 ± 0.0014 | 2.66 ± 0.51 | 6.0 |
| both, every scan | 100 | 32 / 40 | 78.95 ± 6.63 | -1.37 ± 0.14 | 0.0108 ± 0.0018 | 1.35 ± 0.3 | 6.0 |
| round robin | 100 | 32 / 40 | 79.39 ± 6.66 | -0.82 ± 0.09 | 0.0122 ± 0.0029 | 1.6 ± 0.58 | 1.0 |
| far ahead | 100 | 32 / 40 | 79.74 ± 6.68 | -0.38 ± 0.06 | 0.0116 ± 0.0023 | 1.37 ± 0.32 | 3.0 |
