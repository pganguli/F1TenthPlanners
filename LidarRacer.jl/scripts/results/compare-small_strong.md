# Racing the held-out tracks: safety first (small_strong on the car)

Networks: `small_strong` on the car (the small networks), `large` in the cloud, from
`../../LidarLaneNets.jl/weights`.

Tracks: Spa, Monza, Silverstone, Oschersleben, YasMarina. Each configuration: one lap of each track per seed, 8 seeds
(40 runs). A seed sets the LiDAR's noise and where the lap starts. Values are means over
the runs with 95% confidence intervals. Every run's numbers are in [runs-small_strong.csv](runs-small_strong.csv).

## Safety

From the true map and state. Time to collision: the true free distance along the arc the car is
driving, over its speed (above 1 m/s); least over each run. Planning on too much free distance: the
free distance ahead the speed was set for exceeds the true one, along the same arcs from where the
scan was taken, by more than 0.25 m.

| Configuration | Cloud latency [ms] | Crashes | Closest to a wall, worst run [m] | Closest to a wall [m] | Time within 0.2 m of a wall [%] | Scans with time to collision under 0.5 s [%] | Least time to collision [s] | Largest slip angle [°] | Scans planning on too much free distance [%] |
|---|---|---|---|---|---|---|---|---|---|
| small only | – | 0 | 0.14 | 0.29 ± 0.03 | 0.11 ± 0.08 | 0.13 ± 0.06 | 0.51 ± 0.03 | 5.2 ± 0.4 | 5.27 ± 0.47 |
| large only | 25 | 0 | 0.19 | 0.34 ± 0.02 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.58 ± 0.01 | 4.8 ± 0.3 | 5.12 ± 0.42 |
| both, every scan | 25 | 0 | 0.15 | 0.3 ± 0.03 | 0.03 ± 0.03 | 0.09 ± 0.04 | 0.53 ± 0.03 | 4.8 ± 0.3 | 4.88 ± 0.44 |
| round robin | 25 | 0 | 0.14 | 0.28 ± 0.03 | 0.13 ± 0.07 | 0.14 ± 0.06 | 0.51 ± 0.03 | 5.1 ± 0.4 | 5.16 ± 0.46 |
| far ahead | 25 | 0 | 0.16 | 0.3 ± 0.03 | 0.04 ± 0.03 | 0.13 ± 0.05 | 0.52 ± 0.03 | 4.9 ± 0.4 | 4.84 ± 0.42 |
| large only | 50 | 0 | 0.1 | 0.33 ± 0.04 | 0.14 ± 0.09 | 0.11 ± 0.08 | 0.54 ± 0.03 | 4.7 ± 0.3 | 4.88 ± 0.36 |
| both, every scan | 50 | 0 | 0.16 | 0.3 ± 0.03 | 0.02 ± 0.02 | 0.07 ± 0.04 | 0.53 ± 0.03 | 4.8 ± 0.3 | 4.86 ± 0.43 |
| round robin | 50 | 0 | 0.14 | 0.28 ± 0.03 | 0.14 ± 0.08 | 0.14 ± 0.06 | 0.51 ± 0.03 | 5.2 ± 0.4 | 5.2 ± 0.46 |
| far ahead | 50 | 0 | 0.16 | 0.3 ± 0.02 | 0.02 ± 0.02 | 0.07 ± 0.04 | 0.53 ± 0.03 | 4.8 ± 0.3 | 4.86 ± 0.43 |
| large only | 100 | 6 | 0.1 | 0.36 ± 0.04 | 0.2 ± 0.15 | 0.44 ± 0.34 | 0.5 ± 0.06 | 4.8 ± 0.3 | 4.73 ± 0.38 |
| both, every scan | 100 | 0 | 0.14 | 0.29 ± 0.03 | 0.15 ± 0.1 | 0.13 ± 0.06 | 0.52 ± 0.03 | 5.1 ± 0.3 | 5.14 ± 0.44 |
| round robin | 100 | 0 | 0.1 | 0.28 ± 0.03 | 0.19 ± 0.11 | 0.17 ± 0.08 | 0.51 ± 0.03 | 5.3 ± 0.4 | 5.27 ± 0.43 |
| far ahead | 100 | 0 | 0.14 | 0.29 ± 0.03 | 0.14 ± 0.09 | 0.13 ± 0.06 | 0.52 ± 0.03 | 5.0 ± 0.4 | 5.12 ± 0.43 |

### Against small only, run for run

Each run minus small only's run on the same track, seed and start (pp: percentage points). An
interval that excludes 0 is a difference the runs support.

| Configuration | Cloud latency [ms] | Closest to a wall [m] (more is safer) | Time within 0.2 m of a wall [pp] | Scans with time to collision under 0.5 s [pp] | Largest slip angle [°] | Scans planning on too much free distance [pp] |
|---|---|---|---|---|---|---|
| large only | 25 | 0.05 ± 0.02 | -0.11 ± 0.08 | -0.13 ± 0.06 | -0.4 ± 0.3 | -0.15 ± 0.12 |
| both, every scan | 25 | 0.01 ± 0.01 | -0.08 ± 0.08 | -0.04 ± 0.05 | -0.4 ± 0.2 | -0.39 ± 0.15 |
| round robin | 25 | -0.0 ± 0.01 | 0.02 ± 0.08 | 0.01 ± 0.03 | -0.0 ± 0.1 | -0.11 ± 0.1 |
| far ahead | 25 | 0.01 ± 0.01 | -0.07 ± 0.07 | -0.0 ± 0.03 | -0.3 ± 0.1 | -0.43 ± 0.17 |
| large only | 50 | 0.04 ± 0.03 | 0.03 ± 0.1 | -0.02 ± 0.08 | -0.4 ± 0.3 | -0.39 ± 0.21 |
| both, every scan | 50 | 0.01 ± 0.01 | -0.09 ± 0.08 | -0.06 ± 0.05 | -0.4 ± 0.2 | -0.41 ± 0.13 |
| round robin | 50 | -0.0 ± 0.01 | 0.02 ± 0.09 | 0.01 ± 0.03 | 0.0 ± 0.1 | -0.07 ± 0.09 |
| far ahead | 50 | 0.02 ± 0.01 | -0.09 ± 0.07 | -0.06 ± 0.05 | -0.4 ± 0.2 | -0.41 ± 0.13 |
| large only | 100 | 0.07 ± 0.03 | 0.09 ± 0.15 | 0.31 ± 0.32 | -0.4 ± 0.4 | -0.54 ± 0.29 |
| both, every scan | 100 | -0.0 ± 0.01 | 0.04 ± 0.1 | -0.0 ± 0.06 | -0.1 ± 0.2 | -0.14 ± 0.12 |
| round robin | 100 | -0.01 ± 0.01 | 0.08 ± 0.11 | 0.04 ± 0.05 | 0.1 ± 0.1 | 0.0 ± 0.1 |
| far ahead | 100 | 0.0 ± 0.01 | 0.03 ± 0.1 | 0.0 ± 0.05 | -0.1 ± 0.2 | -0.15 ± 0.1 |

## Lap times and tracking

| Configuration | Cloud latency [ms] | Laps finished | Lap time [s] | Lap time vs small only, paired [%] | e error [m] | ψe error [°] | Cloud calls per scan |
|---|---|---|---|---|---|---|---|
| small only | – | 40 / 40 | 75.44 ± 4.74 | – | 0.0085 ± 0.0007 | 1.53 ± 0.31 | 0.0 |
| large only | 25 | 40 / 40 | 75.35 ± 4.79 | -0.16 ± 0.14 | 0.0098 ± 0.0007 | 1.68 ± 0.34 | 6.0 |
| both, every scan | 25 | 40 / 40 | 75.25 ± 4.76 | -0.27 ± 0.08 | 0.0085 ± 0.0006 | 1.52 ± 0.27 | 6.0 |
| round robin | 25 | 40 / 40 | 75.4 ± 4.75 | -0.06 ± 0.04 | 0.0086 ± 0.0007 | 1.55 ± 0.3 | 1.0 |
| far ahead | 25 | 40 / 40 | 75.27 ± 4.76 | -0.24 ± 0.07 | 0.0088 ± 0.0008 | 1.61 ± 0.31 | 3.0 |
| large only | 50 | 40 / 40 | 76.82 ± 4.9 | 1.76 ± 0.19 | 0.0118 ± 0.0008 | 1.9 ± 0.36 | 6.0 |
| both, every scan | 50 | 40 / 40 | 75.27 ± 4.75 | -0.23 ± 0.07 | 0.0084 ± 0.0006 | 1.56 ± 0.29 | 6.0 |
| round robin | 50 | 40 / 40 | 75.42 ± 4.74 | -0.02 ± 0.04 | 0.0086 ± 0.0007 | 1.57 ± 0.31 | 1.0 |
| far ahead | 50 | 40 / 40 | 75.29 ± 4.76 | -0.21 ± 0.07 | 0.0086 ± 0.0007 | 1.61 ± 0.31 | 3.0 |
| large only | 100 | 34 / 40 | 79.87 ± 5.93 | 5.56 ± 0.22 | 0.0175 ± 0.001 | 2.54 ± 0.4 | 6.0 |
| both, every scan | 100 | 40 / 40 | 75.37 ± 4.75 | -0.1 ± 0.06 | 0.0087 ± 0.0007 | 1.63 ± 0.31 | 6.0 |
| round robin | 100 | 40 / 40 | 75.47 ± 4.75 | 0.03 ± 0.04 | 0.0086 ± 0.0007 | 1.57 ± 0.32 | 1.0 |
| far ahead | 100 | 40 / 40 | 75.39 ± 4.75 | -0.07 ± 0.06 | 0.0088 ± 0.0008 | 1.65 ± 0.32 | 3.0 |
