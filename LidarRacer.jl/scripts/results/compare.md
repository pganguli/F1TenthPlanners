# Racing the held-out tracks

Tracks: Spa, Monza, Silverstone, Oschersleben, YasMarina, one lap each. Mean lap time over the laps finished.

| Configuration | Cloud latency [ms] | Laps finished | Mean lap time [s] | Mean speed [m/s] | Closest to a wall [m] | e error [m] | ψe error [°] | Cloud calls per scan |
|---|---|---|---|---|---|---|---|---|
| small only | – | 5 / 5 | 75.78 | 5.46 | 0.19 | 0.008 | 1.54 | 0.00 |
| large only | 25 | 5 / 5 | 75.65 | 5.46 | 0.19 | 0.010 | 1.65 | 6.00 |
| both, every scan | 25 | 5 / 5 | 75.56 | 5.47 | 0.19 | 0.008 | 1.53 | 6.00 |
| round robin | 25 | 5 / 5 | 75.73 | 5.46 | 0.14 | 0.009 | 1.56 | 1.00 |
| far ahead | 25 | 5 / 5 | 75.55 | 5.47 | 0.19 | 0.008 | 1.61 | 3.00 |
| large only | 50 | 5 / 5 | 77.08 | 5.37 | 0.10 | 0.012 | 1.88 | 6.00 |
| both, every scan | 50 | 5 / 5 | 75.60 | 5.47 | 0.19 | 0.008 | 1.56 | 6.00 |
| round robin | 50 | 5 / 5 | 75.74 | 5.46 | 0.14 | 0.008 | 1.57 | 1.00 |
| far ahead | 50 | 5 / 5 | 75.59 | 5.47 | 0.16 | 0.009 | 1.63 | 3.00 |
| large only | 100 | 4 / 5 | 80.11 | 5.11 | 0.10 | 0.018 | 2.85 | 6.00 |
| both, every scan | 100 | 5 / 5 | 75.71 | 5.46 | 0.14 | 0.009 | 1.62 | 6.00 |
| round robin | 100 | 5 / 5 | 75.76 | 5.46 | 0.19 | 0.009 | 1.57 | 1.00 |
| far ahead | 100 | 5 / 5 | 75.69 | 5.46 | 0.19 | 0.009 | 1.62 | 3.00 |

## Per track

**small only**:

| Track | Result | Time [s] | Speed [m/s] | Closest to a wall [m] |
|---|---|---|---|---|
| Spa | lap | 96.02 | 5.69 | 0.38 |
| Monza | lap | 74.48 | 5.94 | 0.19 |
| Silverstone | lap | 81.87 | 5.49 | 0.39 |
| Oschersleben | lap | 49.56 | 5.10 | 0.31 |
| YasMarina | lap | 76.98 | 5.06 | 0.21 |

**large only**, 25 ms:

| Track | Result | Time [s] | Speed [m/s] | Closest to a wall [m] |
|---|---|---|---|---|
| Spa | lap | 96.17 | 5.68 | 0.41 |
| Monza | lap | 74.54 | 5.93 | 0.19 |
| Silverstone | lap | 81.92 | 5.49 | 0.39 |
| Oschersleben | lap | 49.26 | 5.13 | 0.34 |
| YasMarina | lap | 76.38 | 5.08 | 0.31 |

**both, every scan**, 25 ms:

| Track | Result | Time [s] | Speed [m/s] | Closest to a wall [m] |
|---|---|---|---|---|
| Spa | lap | 95.87 | 5.70 | 0.38 |
| Monza | lap | 74.31 | 5.95 | 0.19 |
| Silverstone | lap | 81.73 | 5.50 | 0.39 |
| Oschersleben | lap | 49.32 | 5.12 | 0.31 |
| YasMarina | lap | 76.55 | 5.09 | 0.21 |

**round robin**, 25 ms:

| Track | Result | Time [s] | Speed [m/s] | Closest to a wall [m] |
|---|---|---|---|---|
| Spa | lap | 95.86 | 5.70 | 0.38 |
| Monza | lap | 74.55 | 5.93 | 0.14 |
| Silverstone | lap | 81.83 | 5.49 | 0.39 |
| Oschersleben | lap | 49.52 | 5.10 | 0.30 |
| YasMarina | lap | 76.91 | 5.07 | 0.16 |

**far ahead**, 25 ms:

| Track | Result | Time [s] | Speed [m/s] | Closest to a wall [m] |
|---|---|---|---|---|
| Spa | lap | 95.79 | 5.70 | 0.38 |
| Monza | lap | 74.37 | 5.95 | 0.19 |
| Silverstone | lap | 81.69 | 5.50 | 0.39 |
| Oschersleben | lap | 49.25 | 5.13 | 0.30 |
| YasMarina | lap | 76.64 | 5.08 | 0.21 |

**large only**, 50 ms:

| Track | Result | Time [s] | Speed [m/s] | Closest to a wall [m] |
|---|---|---|---|---|
| Spa | lap | 98.06 | 5.57 | 0.46 |
| Monza | lap | 76.04 | 5.82 | 0.10 |
| Silverstone | lap | 83.58 | 5.38 | 0.42 |
| Oschersleben | lap | 49.94 | 5.07 | 0.35 |
| YasMarina | lap | 77.79 | 5.00 | 0.33 |

**both, every scan**, 50 ms:

| Track | Result | Time [s] | Speed [m/s] | Closest to a wall [m] |
|---|---|---|---|---|
| Spa | lap | 95.89 | 5.70 | 0.38 |
| Monza | lap | 74.46 | 5.94 | 0.19 |
| Silverstone | lap | 81.72 | 5.50 | 0.39 |
| Oschersleben | lap | 49.32 | 5.12 | 0.30 |
| YasMarina | lap | 76.60 | 5.09 | 0.21 |

**round robin**, 50 ms:

| Track | Result | Time [s] | Speed [m/s] | Closest to a wall [m] |
|---|---|---|---|---|
| Spa | lap | 96.03 | 5.69 | 0.38 |
| Monza | lap | 74.57 | 5.93 | 0.14 |
| Silverstone | lap | 81.78 | 5.50 | 0.39 |
| Oschersleben | lap | 49.54 | 5.10 | 0.30 |
| YasMarina | lap | 76.78 | 5.08 | 0.16 |

**far ahead**, 50 ms:

| Track | Result | Time [s] | Speed [m/s] | Closest to a wall [m] |
|---|---|---|---|---|
| Spa | lap | 95.83 | 5.70 | 0.38 |
| Monza | lap | 74.37 | 5.95 | 0.19 |
| Silverstone | lap | 81.74 | 5.50 | 0.39 |
| Oschersleben | lap | 49.30 | 5.12 | 0.30 |
| YasMarina | lap | 76.69 | 5.08 | 0.16 |

**large only**, 100 ms:

| Track | Result | Time [s] | Speed [m/s] | Closest to a wall [m] |
|---|---|---|---|---|
| Spa | lap | 101.21 | 5.40 | 0.46 |
| Monza | crash | 14.50 | 5.23 | 0.10 |
| Silverstone | lap | 86.28 | 5.22 | 0.45 |
| Oschersleben | lap | 52.04 | 4.88 | 0.43 |
| YasMarina | lap | 80.93 | 4.82 | 0.37 |

**both, every scan**, 100 ms:

| Track | Result | Time [s] | Speed [m/s] | Closest to a wall [m] |
|---|---|---|---|---|
| Spa | lap | 95.90 | 5.70 | 0.38 |
| Monza | lap | 74.63 | 5.93 | 0.14 |
| Silverstone | lap | 81.77 | 5.50 | 0.39 |
| Oschersleben | lap | 49.47 | 5.11 | 0.31 |
| YasMarina | lap | 76.77 | 5.08 | 0.16 |

**round robin**, 100 ms:

| Track | Result | Time [s] | Speed [m/s] | Closest to a wall [m] |
|---|---|---|---|---|
| Spa | lap | 95.95 | 5.69 | 0.38 |
| Monza | lap | 74.53 | 5.93 | 0.19 |
| Silverstone | lap | 81.89 | 5.49 | 0.39 |
| Oschersleben | lap | 49.50 | 5.10 | 0.30 |
| YasMarina | lap | 76.92 | 5.07 | 0.21 |

**far ahead**, 100 ms:

| Track | Result | Time [s] | Speed [m/s] | Closest to a wall [m] |
|---|---|---|---|---|
| Spa | lap | 95.91 | 5.69 | 0.38 |
| Monza | lap | 74.45 | 5.94 | 0.19 |
| Silverstone | lap | 81.79 | 5.49 | 0.39 |
| Oschersleben | lap | 49.47 | 5.11 | 0.31 |
| YasMarina | lap | 76.85 | 5.07 | 0.21 |
