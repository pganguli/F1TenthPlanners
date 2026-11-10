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
| small only | – | 8 | 0.07 | 0.29 ± 0.04 | 0.38 ± 0.34 | 1.14 ± 1.1 | 0.48 ± 0.05 | 5.1 ± 0.4 | 4.59 ± 0.91 |
| large only | 25 | 0 | 0.19 | 0.34 ± 0.02 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.58 ± 0.01 | 4.8 ± 0.3 | 5.12 ± 0.42 |
| both, every scan | 25 | 7 | 0.07 | 0.29 ± 0.04 | 0.4 ± 0.35 | 1.14 ± 1.17 | 0.53 ± 0.05 | 5.4 ± 0.5 | 4.11 ± 0.74 |
| round robin | 25 | 4 | 0.07 | 0.29 ± 0.04 | 0.61 ± 0.71 | 0.75 ± 0.82 | 0.51 ± 0.04 | 5.4 ± 0.5 | 4.47 ± 0.96 |
| far ahead | 25 | 3 | 0.07 | 0.29 ± 0.04 | 0.39 ± 0.27 | 0.54 ± 0.48 | 0.55 ± 0.05 | 5.3 ± 0.5 | 3.81 ± 0.52 |
| large only | 50 | 0 | 0.1 | 0.33 ± 0.04 | 0.14 ± 0.09 | 0.11 ± 0.08 | 0.54 ± 0.03 | 4.7 ± 0.3 | 4.88 ± 0.36 |
| both, every scan | 50 | 2 | 0.07 | 0.28 ± 0.04 | 0.45 ± 0.47 | 0.88 ± 1.21 | 0.54 ± 0.04 | 5.3 ± 0.5 | 4.24 ± 0.78 |
| round robin | 50 | 2 | 0.07 | 0.29 ± 0.04 | 0.44 ± 0.5 | 0.8 ± 1.15 | 0.52 ± 0.03 | 5.4 ± 0.4 | 4.39 ± 0.79 |
| far ahead | 50 | 0 | 0.1 | 0.31 ± 0.03 | 0.09 ± 0.06 | 0.16 ± 0.1 | 0.56 ± 0.03 | 5.3 ± 0.5 | 3.91 ± 0.48 |
| large only | 100 | 6 | 0.1 | 0.36 ± 0.04 | 0.2 ± 0.15 | 0.44 ± 0.34 | 0.5 ± 0.06 | 4.8 ± 0.3 | 4.73 ± 0.38 |
| both, every scan | 100 | 0 | 0.07 | 0.29 ± 0.04 | 0.26 ± 0.15 | 0.23 ± 0.13 | 0.53 ± 0.03 | 5.3 ± 0.5 | 4.09 ± 0.5 |
| round robin | 100 | 4 | 0.07 | 0.28 ± 0.04 | 0.57 ± 0.44 | 1.01 ± 1.13 | 0.49 ± 0.04 | 5.3 ± 0.4 | 4.42 ± 0.75 |
| far ahead | 100 | 1 | 0.07 | 0.28 ± 0.04 | 0.23 ± 0.14 | 0.25 ± 0.16 | 0.53 ± 0.04 | 5.4 ± 0.5 | 4.05 ± 0.48 |

### Against small only, run for run

Each run minus small only's run on the same track, seed and start (pp: percentage points). An
interval that excludes 0 is a difference the runs support.

| Configuration | Cloud latency [ms] | Closest to a wall [m] (more is safer) | Time within 0.2 m of a wall [pp] | Scans with time to collision under 0.5 s [pp] | Largest slip angle [°] | Scans planning on too much free distance [pp] |
|---|---|---|---|---|---|---|
| large only | 25 | 0.05 ± 0.03 | -0.38 ± 0.34 | -1.14 ± 1.1 | -0.3 ± 0.4 | 0.53 ± 0.61 |
| both, every scan | 25 | 0.0 ± 0.01 | 0.02 ± 0.11 | -0.0 ± 0.14 | 0.3 ± 0.3 | -0.48 ± 0.26 |
| round robin | 25 | -0.0 ± 0.01 | 0.23 ± 0.43 | -0.39 ± 0.38 | 0.3 ± 0.2 | -0.12 ± 0.23 |
| far ahead | 25 | -0.0 ± 0.01 | 0.01 ± 0.27 | -0.6 ± 0.98 | 0.3 ± 0.3 | -0.78 ± 0.55 |
| large only | 50 | 0.04 ± 0.04 | -0.24 ± 0.37 | -1.03 ± 1.11 | -0.3 ± 0.3 | 0.3 ± 0.66 |
| both, every scan | 50 | -0.01 ± 0.02 | 0.07 ± 0.22 | -0.27 ± 0.39 | 0.2 ± 0.3 | -0.35 ± 0.24 |
| round robin | 50 | 0.0 ± 0.01 | 0.06 ± 0.26 | -0.34 ± 0.38 | 0.3 ± 0.2 | -0.2 ± 0.2 |
| far ahead | 50 | 0.02 ± 0.01 | -0.29 ± 0.29 | -0.99 ± 1.04 | 0.3 ± 0.3 | -0.68 ± 0.53 |
| large only | 100 | 0.07 ± 0.05 | -0.18 ± 0.39 | -0.7 ± 1.19 | -0.3 ± 0.4 | 0.15 ± 0.67 |
| both, every scan | 100 | -0.0 ± 0.01 | -0.12 ± 0.27 | -0.92 ± 1.02 | 0.3 ± 0.2 | -0.5 ± 0.55 |
| round robin | 100 | -0.01 ± 0.01 | 0.19 ± 0.18 | -0.14 ± 0.21 | 0.2 ± 0.2 | -0.17 ± 0.26 |
| far ahead | 100 | -0.01 ± 0.02 | -0.15 ± 0.27 | -0.89 ± 1.01 | 0.3 ± 0.3 | -0.54 ± 0.57 |

## Lap times and tracking

| Configuration | Cloud latency [ms] | Laps finished | Lap time [s] | Lap time vs small only, paired [%] | e error [m] | ψe error [°] | Cloud calls per scan |
|---|---|---|---|---|---|---|---|
| small only | – | 32 / 40 | 77.95 ± 6.13 | – | 0.0143 ± 0.0041 | 2.44 ± 0.96 | 0.0 |
| large only | 25 | 40 / 40 | 75.35 ± 4.79 | -3.63 ± 0.38 | 0.0098 ± 0.0007 | 1.68 ± 0.34 | 6.0 |
| both, every scan | 25 | 33 / 40 | 78.03 ± 5.96 | -0.01 ± 0.23 | 0.0125 ± 0.0028 | 2.13 ± 0.72 | 6.0 |
| round robin | 25 | 36 / 40 | 78.24 ± 5.44 | 0.1 ± 0.12 | 0.012 ± 0.0025 | 2.12 ± 0.68 | 1.0 |
| far ahead | 25 | 37 / 40 | 78.14 ± 5.3 | -0.05 ± 0.2 | 0.0121 ± 0.0019 | 1.81 ± 0.37 | 3.0 |
| large only | 50 | 40 / 40 | 76.82 ± 4.9 | -1.75 ± 0.46 | 0.0118 ± 0.0008 | 1.9 ± 0.36 | 6.0 |
| both, every scan | 50 | 38 / 40 | 78.24 ± 5.16 | 0.02 ± 0.22 | 0.011 ± 0.0021 | 1.9 ± 0.58 | 6.0 |
| round robin | 50 | 38 / 40 | 78.35 ± 5.16 | 0.1 ± 0.15 | 0.0121 ± 0.0028 | 2.1 ± 0.74 | 1.0 |
| far ahead | 50 | 40 / 40 | 78.22 ± 4.91 | -0.1 ± 0.18 | 0.0111 ± 0.0012 | 1.62 ± 0.28 | 3.0 |
| large only | 100 | 34 / 40 | 79.87 ± 5.93 | 1.51 ± 0.53 | 0.0175 ± 0.001 | 2.54 ± 0.4 | 6.0 |
| both, every scan | 100 | 40 / 40 | 78.41 ± 4.89 | 0.1 ± 0.16 | 0.0102 ± 0.001 | 1.73 ± 0.32 | 6.0 |
| round robin | 100 | 36 / 40 | 78.32 ± 5.45 | 0.16 ± 0.14 | 0.0129 ± 0.0032 | 2.27 ± 0.83 | 1.0 |
| far ahead | 100 | 39 / 40 | 78.34 ± 5.02 | 0.1 ± 0.17 | 0.0117 ± 0.0014 | 1.72 ± 0.32 | 3.0 |
