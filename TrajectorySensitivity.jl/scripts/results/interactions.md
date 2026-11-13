# How the estimates' errors add up and interact

Every estimate errs at once: the car network's recorded errors, replayed together (from the same
recorded scans, so they err together as the networks do), each times its own gain, drawn uniformly
from 0 (true) to 2 (twice the car network's). Variance-based (Sobol) indices split each outcome's
variance among the estimates (Saltelli's design; Saltelli 2010's first-order and Jansen's total
estimators). From [`interactions.jl`](../interactions.jl), summed up by [`report.jl`](../report.jl);
95% intervals in brackets (bootstrap over the samples). A sum of first-order indices well under 1,
or totals well over the first-order ones, means the estimates' errors interact.

## The representative track

1 course, 512 samples
each (4608 runs). First-order index: the share of the outcome's variance an estimate's
errors cause alone; total: with all their interactions; interactions: the difference. `seed`: the
replay's randomness (which recorded errors come when), not an estimate.


### RMS deviation

| Factor | First-order | Total | Interactions |
|---|---|---|---|
| `e` | -0.014 (-0.061–0.031) | 0.028 (0.023–0.035) | 0.041 |
| `ψe` | 0.013 (-0.042–0.062) | 0.033 (0.028–0.04) | 0.02 |
| `κ_profile` | 1.0 (0.74–1.3) | 1.0 (0.95–1.1) | 0.039 |
| `widths` | -0.011 (-0.039–0.019) | 0.012 (0.0089–0.015) | 0.023 |
| `sight` | 0.0029 (-0.045–0.045) | 0.036 (0.03–0.042) | 0.033 |
| `clearance` | -0.0016 (-0.051–0.051) | 0.045 (0.038–0.053) | 0.047 |
| `seed` | 0.012 (-0.029–0.05) | 0.03 (0.024–0.037) | 0.018 |
| sum | 1.0 | 1.2 | |

### Least gap to a wall

| Factor | First-order | Total | Interactions |
|---|---|---|---|
| `e` | 0.026 (-0.056–0.1) | 0.13 (0.1–0.16) | 0.1 |
| `ψe` | 0.056 (-0.03–0.15) | 0.18 (0.15–0.21) | 0.12 |
| `κ_profile` | 0.83 (0.6–1.0) | 1.0 (0.92–1.1) | 0.18 |
| `widths` | -0.047 (-0.1–0.024) | 0.074 (0.058–0.091) | 0.12 |
| `sight` | 0.056 (-0.021–0.14) | 0.13 (0.11–0.15) | 0.074 |
| `clearance` | 0.013 (-0.087–0.12) | 0.21 (0.18–0.25) | 0.2 |
| `seed` | -0.0072 (-0.092–0.074) | 0.15 (0.12–0.17) | 0.15 |
| sum | 0.92 | 1.9 | |

### Crashed

| Factor | First-order | Total | Interactions |
|---|---|---|---|
| `e` | -0.12 (-0.2–-0.042) | 0.25 (0.18–0.32) | 0.37 |
| `ψe` | -0.054 (-0.15–0.043) | 0.32 (0.25–0.4) | 0.38 |
| `κ_profile` | 0.2 (0.059–0.34) | 0.82 (0.72–0.95) | 0.62 |
| `widths` | -0.054 (-0.13–0.02) | 0.2 (0.14–0.26) | 0.25 |
| `sight` | -0.064 (-0.13–0.0) | 0.23 (0.16–0.29) | 0.29 |
| `clearance` | 0.19 (0.094–0.3) | 0.68 (0.58–0.79) | 0.49 |
| `seed` | 0.0 (-0.061–0.062) | 0.22 (0.15–0.29) | 0.22 |
| sum | 0.11 | 2.7 | |

### Crashes: `κ_profile` and `clearance` together

The two estimates whose errors interact the most in crashes (total less first-order index). Share of
runs that crashed (of how many), by the gain on `κ_profile`'s errors, and whether `clearance`'s are at most the car
network's (gain ≤ 1) or larger; the other estimates' gains anywhere from 0 to 2.

| `κ_profile` gain | `clearance` gain ≤ 1 | `clearance` gain > 1 |
|---|---|---|
| 0.0–0.5 | 0% (126) | 0% (151) |
| 0.5–1.0 | 1% (123) | 11% (120) |
| 1.0–1.5 | 5% (123) | 65% (119) |
| 1.5–2.0 | 27% (139) | 90% (123) |

### RMS deviation, by situation: total indices

| Factor | straight | gentle entry | gentle middle | gentle exit | medium entry | medium middle | medium exit | tight entry | tight middle | tight exit |
|---|---|---|---|---|---|---|---|---|---|---|
| `e` | 0.076 | 0.32 | 0.35 | 0.11 | 0.071 | 0.15 | 0.071 | 0.03 | 0.02 | 0.034 |
| `ψe` | 0.076 | 0.3 | 0.4 | 0.17 | 0.058 | 0.21 | 0.1 | 0.034 | 0.027 | 0.038 |
| `κ_profile` | 1.1 | 0.99 | 0.71 | 0.98 | 1.0 | 0.98 | 1.0 | 0.91 | 0.99 | 1.0 |
| `widths` | 0.044 | 0.14 | 0.11 | 0.046 | 0.029 | 0.076 | 0.018 | 0.013 | 0.009 | 0.013 |
| `sight` | 0.065 | 0.23 | 0.39 | 0.11 | 0.05 | 0.14 | 0.063 | 0.096 | 0.089 | 0.063 |
| `clearance` | 0.065 | 0.28 | 0.77 | 0.16 | 0.098 | 0.36 | 0.085 | 0.14 | 0.061 | 0.068 |
| `seed` | 0.077 | 0.32 | 0.36 | 0.14 | 0.064 | 0.12 | 0.084 | 0.034 | 0.023 | 0.038 |

## Every track

23 courses (indices pooled, each course weighted by its variance), 64 samples
each (13248 runs). First-order index: the share of the outcome's variance an estimate's
errors cause alone; total: with all their interactions; interactions: the difference. `seed`: the
replay's randomness (which recorded errors come when), not an estimate.


### RMS deviation

| Factor | First-order | Total | Interactions |
|---|---|---|---|
| `e` | 0.018 (-0.024–0.063) | 0.047 (0.035–0.062) | 0.028 |
| `ψe` | 0.032 (-0.019–0.087) | 0.057 (0.045–0.071) | 0.025 |
| `κ_profile` | 0.99 (0.8–1.2) | 0.96 (0.9–1.0) | -0.03 |
| `widths` | 0.1 (0.04–0.16) | 0.07 (0.059–0.082) | -0.03 |
| `sight` | 0.014 (-0.025–0.058) | 0.039 (0.031–0.047) | 0.025 |
| `clearance` | 0.034 (-0.023–0.095) | 0.078 (0.065–0.095) | 0.044 |
| `seed` | 0.018 (-0.025–0.067) | 0.052 (0.04–0.065) | 0.034 |
| sum | 1.2 | 1.3 | |

### Least gap to a wall

| Factor | First-order | Total | Interactions |
|---|---|---|---|
| `e` | -0.071 (-0.18–0.026) | 0.15 (0.13–0.17) | 0.22 |
| `ψe` | -0.068 (-0.19–0.039) | 0.19 (0.17–0.22) | 0.26 |
| `κ_profile` | 0.51 (0.27–0.73) | 0.97 (0.9–1.1) | 0.46 |
| `widths` | -0.071 (-0.2–0.041) | 0.17 (0.14–0.2) | 0.24 |
| `sight` | -0.02 (-0.13–0.089) | 0.16 (0.14–0.19) | 0.18 |
| `clearance` | -0.02 (-0.14–0.12) | 0.21 (0.18–0.24) | 0.23 |
| `seed` | -0.093 (-0.21–0.016) | 0.17 (0.14–0.2) | 0.26 |
| sum | 0.17 | 2.0 | |

### Crashed

| Factor | First-order | Total | Interactions |
|---|---|---|---|
| `e` | 0.032 (-0.033–0.1) | 0.3 (0.23–0.37) | 0.27 |
| `ψe` | 0.032 (-0.031–0.1) | 0.28 (0.21–0.35) | 0.24 |
| `κ_profile` | 0.52 (0.38–0.69) | 1.0 (0.95–1.1) | 0.52 |
| `widths` | -2.4e-18 (-0.069–0.063) | 0.26 (0.2–0.34) | 0.26 |
| `sight` | 0.064 (0.0–0.13) | 0.23 (0.17–0.29) | 0.16 |
| `clearance` | 0.095 (0.011–0.17) | 0.4 (0.32–0.48) | 0.3 |
| `seed` | 0.064 (0.011–0.12) | 0.32 (0.25–0.39) | 0.25 |
| sum | 0.81 | 2.8 | |

### Crashes: `κ_profile` and `clearance` together

The two estimates whose errors interact the most in crashes (total less first-order index). Share of
runs that crashed (of how many), by the gain on `κ_profile`'s errors, and whether `clearance`'s are at most the car
network's (gain ≤ 1) or larger; the other estimates' gains anywhere from 0 to 2.

| `κ_profile` gain | `clearance` gain ≤ 1 | `clearance` gain > 1 |
|---|---|---|
| 0.0–0.5 | 0% (353) | 0% (385) |
| 0.5–1.0 | 0% (344) | 4% (395) |
| 1.0–1.5 | 11% (370) | 16% (376) |
| 1.5–2.0 | 24% (350) | 30% (371) |

### RMS deviation, by situation: total indices

| Factor | straight | gentle entry | gentle middle | gentle exit | medium entry | medium middle | medium exit | tight entry | tight middle | tight exit |
|---|---|---|---|---|---|---|---|---|---|---|
| `e` | 0.083 | 0.18 | 0.13 | 0.24 | 0.13 | 0.04 | 0.068 | 0.08 | 0.08 | 0.071 |
| `ψe` | 0.1 | 0.17 | 0.12 | 0.24 | 0.14 | 0.068 | 0.097 | 0.076 | 0.15 | 0.11 |
| `κ_profile` | 0.96 | 0.89 | 0.86 | 1.0 | 0.96 | 0.87 | 0.97 | 0.92 | 0.95 | 1.0 |
| `widths` | 0.085 | 0.069 | 0.072 | 0.12 | 0.074 | 0.026 | 0.051 | 0.18 | 0.067 | 0.097 |
| `sight` | 0.073 | 0.16 | 0.087 | 0.17 | 0.12 | 0.039 | 0.073 | 0.073 | 0.12 | 0.072 |
| `clearance` | 0.12 | 0.18 | 0.22 | 0.36 | 0.21 | 0.14 | 0.15 | 0.094 | 0.16 | 0.11 |
| `seed` | 0.092 | 0.19 | 0.11 | 0.2 | 0.12 | 0.041 | 0.08 | 0.071 | 0.091 | 0.096 |
