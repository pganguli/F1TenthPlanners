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
| small only | – | 8 | 0.07 | 0.27 ± 0.04 | 0.22 ± 0.23 | 0.94 ± 1.0 | 0.44 ± 0.06 | 4.6 ± 0.3 | 5.88 ± 0.78 |
| large only | 25 | 0 | 0.1 | 0.31 ± 0.03 | 0.11 ± 0.07 | 0.08 ± 0.06 | 0.56 ± 0.03 | 4.8 ± 0.3 | 5.83 ± 0.45 |
| both, every scan | 25 | 0 | 0.29 | 0.37 ± 0.01 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.55 ± 0.01 | 5.3 ± 0.2 | 6.37 ± 0.45 |
| round robin | 25 | 4 | 0.07 | 0.3 ± 0.04 | 0.27 ± 0.31 | 0.63 ± 0.86 | 0.47 ± 0.05 | 4.9 ± 0.3 | 7.29 ± 1.25 |
| far ahead | 25 | 0 | 0.35 | 0.4 ± 0.01 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.6 ± 0.0 | 4.5 ± 0.3 | 4.62 ± 0.5 |
| large only | 50 | 0 | 0.1 | 0.33 ± 0.04 | 0.18 ± 0.12 | 0.15 ± 0.1 | 0.54 ± 0.05 | 4.9 ± 0.3 | 5.23 ± 0.42 |
| both, every scan | 50 | 0 | 0.19 | 0.36 ± 0.02 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.54 ± 0.01 | 5.2 ± 0.3 | 6.54 ± 0.53 |
| round robin | 50 | 7 | 0.07 | 0.28 ± 0.05 | 0.32 ± 0.32 | 0.89 ± 0.93 | 0.44 ± 0.05 | 4.9 ± 0.3 | 7.47 ± 1.24 |
| far ahead | 50 | 0 | 0.19 | 0.37 ± 0.02 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.58 ± 0.01 | 4.5 ± 0.3 | 4.81 ± 0.57 |
| large only | 100 | 2 | 0.1 | 0.36 ± 0.04 | 0.28 ± 0.18 | 0.24 ± 0.17 | 0.53 ± 0.05 | 5.0 ± 0.4 | 4.82 ± 0.43 |
| both, every scan | 100 | 8 | 0.07 | 0.28 ± 0.05 | 0.33 ± 0.32 | 0.82 ± 0.85 | 0.44 ± 0.05 | 5.1 ± 0.3 | 7.93 ± 1.4 |
| round robin | 100 | 8 | 0.07 | 0.27 ± 0.05 | 0.36 ± 0.31 | 1.1 ± 1.14 | 0.42 ± 0.05 | 4.8 ± 0.3 | 7.6 ± 1.01 |
| far ahead | 100 | 7 | 0.07 | 0.3 ± 0.05 | 0.33 ± 0.34 | 0.87 ± 1.04 | 0.46 ± 0.06 | 4.5 ± 0.3 | 6.5 ± 1.65 |

### Against small only, run for run

Each run minus small only's run on the same track, seed and start (pp: percentage points). An
interval that excludes 0 is a difference the runs support.

| Configuration | Cloud latency [ms] | Closest to a wall [m] (more is safer) | Time within 0.2 m of a wall [pp] | Scans with time to collision under 0.5 s [pp] | Largest slip angle [°] | Scans planning on too much free distance [pp] |
|---|---|---|---|---|---|---|
| large only | 25 | 0.04 ± 0.03 | -0.11 ± 0.25 | -0.86 ± 1.01 | 0.2 ± 0.2 | -0.05 ± 0.59 |
| both, every scan | 25 | 0.1 ± 0.04 | -0.22 ± 0.23 | -0.94 ± 1.0 | 0.7 ± 0.2 | 0.49 ± 0.47 |
| round robin | 25 | 0.03 ± 0.01 | 0.05 ± 0.1 | -0.31 ± 0.27 | 0.3 ± 0.2 | 1.41 ± 0.74 |
| far ahead | 25 | 0.13 ± 0.03 | -0.22 ± 0.23 | -0.94 ± 1.0 | -0.1 ± 0.2 | -1.26 ± 0.37 |
| large only | 50 | 0.06 ± 0.03 | -0.04 ± 0.27 | -0.78 ± 1.02 | 0.3 ± 0.3 | -0.65 ± 0.61 |
| both, every scan | 50 | 0.09 ± 0.03 | -0.22 ± 0.23 | -0.93 ± 1.0 | 0.6 ± 0.2 | 0.66 ± 0.37 |
| round robin | 50 | 0.01 ± 0.01 | 0.1 ± 0.1 | -0.05 ± 0.11 | 0.3 ± 0.2 | 1.59 ± 0.68 |
| far ahead | 50 | 0.1 ± 0.03 | -0.22 ± 0.23 | -0.94 ± 1.0 | -0.1 ± 0.2 | -1.06 ± 0.31 |
| large only | 100 | 0.08 ± 0.04 | 0.06 ± 0.3 | -0.69 ± 1.03 | 0.4 ± 0.4 | -1.06 ± 0.48 |
| both, every scan | 100 | 0.01 ± 0.01 | 0.11 ± 0.1 | -0.11 ± 0.16 | 0.4 ± 0.2 | 2.05 ± 0.81 |
| round robin | 100 | 0.0 ± 0.01 | 0.14 ± 0.1 | 0.16 ± 0.16 | 0.2 ± 0.2 | 1.72 ± 0.37 |
| far ahead | 100 | 0.02 ± 0.01 | 0.11 ± 0.11 | -0.07 ± 0.12 | -0.1 ± 0.2 | 0.62 ± 1.07 |

## Lap times and tracking

| Configuration | Cloud latency [ms] | Laps finished | Lap time [s] | Lap time vs small only, paired [%] | e error [m] | ψe error [°] | Cloud calls per scan |
|---|---|---|---|---|---|---|---|
| small only | – | 32 / 40 | 78.76 ± 6.71 | – | 0.0132 ± 0.0035 | 1.92 ± 0.8 | 0.0 |
| large only | 25 | 40 / 40 | 75.35 ± 4.96 | -4.41 ± 0.25 | 0.01 ± 0.0008 | 1.75 ± 0.38 | 6.0 |
| both, every scan | 25 | 40 / 40 | 75.27 ± 5.01 | -4.64 ± 0.13 | 0.0089 ± 0.0006 | 1.22 ± 0.22 | 6.0 |
| round robin | 25 | 36 / 40 | 76.72 ± 5.73 | -2.8 ± 0.13 | 0.0106 ± 0.0018 | 1.39 ± 0.32 | 1.0 |
| far ahead | 25 | 40 / 40 | 78.14 ± 5.25 | -0.85 ± 0.11 | 0.0093 ± 0.0008 | 1.26 ± 0.22 | 3.0 |
| large only | 50 | 40 / 40 | 77.33 ± 5.12 | -1.87 ± 0.29 | 0.0123 ± 0.0009 | 1.96 ± 0.39 | 6.0 |
| both, every scan | 50 | 40 / 40 | 75.9 ± 5.07 | -3.92 ± 0.1 | 0.0092 ± 0.0007 | 1.25 ± 0.23 | 6.0 |
| round robin | 50 | 33 / 40 | 76.95 ± 6.32 | -2.35 ± 0.12 | 0.0113 ± 0.0021 | 1.44 ± 0.36 | 1.0 |
| far ahead | 50 | 40 / 40 | 78.29 ± 5.25 | -0.74 ± 0.12 | 0.0095 ± 0.0009 | 1.26 ± 0.22 | 3.0 |
| large only | 100 | 38 / 40 | 80.01 ± 5.54 | 1.42 ± 0.48 | 0.0183 ± 0.0011 | 2.51 ± 0.42 | 6.0 |
| both, every scan | 100 | 32 / 40 | 76.58 ± 6.5 | -2.74 ± 0.1 | 0.011 ± 0.0018 | 1.39 ± 0.31 | 6.0 |
| round robin | 100 | 32 / 40 | 77.47 ± 6.58 | -1.61 ± 0.12 | 0.0122 ± 0.0026 | 1.6 ± 0.49 | 1.0 |
| far ahead | 100 | 33 / 40 | 78.49 ± 6.47 | -0.42 ± 0.1 | 0.0114 ± 0.0022 | 1.39 ± 0.33 | 3.0 |
