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
| `e` | -0.0028 (-0.058–0.048) | 0.044 (0.036–0.054) | 0.047 |
| `ψe` | -0.015 (-0.072–0.04) | 0.049 (0.038–0.06) | 0.064 |
| `κ_profile` | 1.1 (0.87–1.4) | 1.1 (1.0–1.2) | -0.026 |
| `widths` | 0.0057 (-0.027–0.04) | 0.014 (0.011–0.018) | 0.0086 |
| `sight` | -0.013 (-0.068–0.04) | 0.043 (0.037–0.051) | 0.056 |
| `clearance` | 0.065 (-0.0053–0.14) | 0.07 (0.059–0.081) | 0.005 |
| `seed` | -0.049 (-0.1–-0.00026) | 0.043 (0.034–0.051) | 0.092 |
| sum | 1.1 | 1.4 | |

### Least gap to a wall

| Factor | First-order | Total | Interactions |
|---|---|---|---|
| `e` | -0.035 (-0.13–0.039) | 0.12 (0.094–0.15) | 0.16 |
| `ψe` | 0.022 (-0.059–0.1) | 0.15 (0.12–0.18) | 0.13 |
| `κ_profile` | 0.87 (0.62–1.1) | 1.0 (0.94–1.1) | 0.16 |
| `widths` | -0.0095 (-0.077–0.057) | 0.073 (0.055–0.094) | 0.082 |
| `sight` | -0.0011 (-0.086–0.083) | 0.13 (0.11–0.16) | 0.13 |
| `clearance` | -0.043 (-0.14–0.037) | 0.13 (0.1–0.16) | 0.17 |
| `seed` | -0.034 (-0.12–0.048) | 0.12 (0.1–0.15) | 0.16 |
| sum | 0.77 | 1.8 | |

### Crashed

| Factor | First-order | Total | Interactions |
|---|---|---|---|
| `e` | -0.0093 (-0.083–0.072) | 0.34 (0.26–0.41) | 0.35 |
| `ψe` | 0.0 (-0.08–0.093) | 0.38 (0.31–0.46) | 0.38 |
| `κ_profile` | 0.56 (0.41–0.72) | 0.95 (0.85–1.0) | 0.39 |
| `widths` | -0.037 (-0.11–0.028) | 0.23 (0.17–0.3) | 0.27 |
| `sight` | -0.028 (-0.12–0.047) | 0.34 (0.27–0.42) | 0.37 |
| `clearance` | 0.047 (-0.037–0.13) | 0.38 (0.31–0.46) | 0.34 |
| `seed` | -0.0093 (-0.073–0.059) | 0.32 (0.25–0.39) | 0.33 |
| sum | 0.52 | 2.9 | |

### Crashes: `κ_profile` and `ψe` together

The two estimates whose errors interact the most in crashes (total less first-order index). Share of
runs that crashed (of how many), by the gain on `κ_profile`'s errors, and whether `ψe`'s are at most the car
network's (gain ≤ 1) or larger; the other estimates' gains anywhere from 0 to 2.

| `κ_profile` gain | `ψe` gain ≤ 1 | `ψe` gain > 1 |
|---|---|---|
| 0.0–0.5 | 0% (134) | 0% (143) |
| 0.5–1.0 | 14% (123) | 0% (120) |
| 1.0–1.5 | 36% (121) | 16% (121) |
| 1.5–2.0 | 87% (144) | 85% (118) |

### RMS deviation, by situation: total indices

| Factor | straight | gentle entry | gentle middle | gentle exit | medium entry | medium middle | medium exit | tight entry | tight middle | tight exit |
|---|---|---|---|---|---|---|---|---|---|---|
| `e` | 0.42 | 0.24 | 0.27 | 0.31 | 0.16 | 0.2 | 0.09 | 0.11 | 0.13 | 0.058 |
| `ψe` | 0.43 | 0.26 | 0.22 | 0.4 | 0.2 | 0.37 | 0.091 | 0.13 | 0.16 | 0.062 |
| `κ_profile` | 1.3 | 1.1 | 0.85 | 1.1 | 0.99 | 1.0 | 1.1 | 1.0 | 1.0 | 1.1 |
| `widths` | 0.14 | 0.1 | 0.084 | 0.11 | 0.058 | 0.08 | 0.018 | 0.03 | 0.062 | 0.014 |
| `sight` | 0.35 | 0.29 | 0.23 | 0.29 | 0.17 | 0.18 | 0.064 | 0.13 | 0.15 | 0.051 |
| `clearance` | 0.42 | 0.26 | 0.51 | 0.34 | 0.18 | 0.32 | 0.097 | 0.14 | 0.2 | 0.083 |
| `seed` | 0.39 | 0.27 | 0.24 | 0.28 | 0.17 | 0.22 | 0.089 | 0.097 | 0.15 | 0.055 |

## Every track

24 courses (indices pooled, each course weighted by its variance), 64 samples
each (13824 runs). First-order index: the share of the outcome's variance an estimate's
errors cause alone; total: with all their interactions; interactions: the difference. `seed`: the
replay's randomness (which recorded errors come when), not an estimate.


### RMS deviation

| Factor | First-order | Total | Interactions |
|---|---|---|---|
| `e` | 0.017 (-0.029–0.065) | 0.06 (0.049–0.074) | 0.043 |
| `ψe` | 0.031 (-0.019–0.083) | 0.066 (0.054–0.078) | 0.035 |
| `κ_profile` | 1.0 (0.83–1.2) | 0.95 (0.9–1.0) | -0.075 |
| `widths` | 0.19 (0.12–0.26) | 0.097 (0.081–0.12) | -0.093 |
| `sight` | 0.018 (-0.027–0.061) | 0.053 (0.043–0.063) | 0.034 |
| `clearance` | 0.032 (-0.024–0.088) | 0.073 (0.063–0.085) | 0.041 |
| `seed` | 0.072 (0.028–0.12) | 0.054 (0.043–0.066) | -0.018 |
| sum | 1.4 | 1.4 | |

### Least gap to a wall

| Factor | First-order | Total | Interactions |
|---|---|---|---|
| `e` | 0.0062 (-0.13–0.13) | 0.21 (0.17–0.25) | 0.2 |
| `ψe` | 0.072 (-0.062–0.22) | 0.28 (0.24–0.33) | 0.21 |
| `κ_profile` | 0.63 (0.39–0.88) | 0.96 (0.89–1.0) | 0.33 |
| `widths` | 0.018 (-0.12–0.16) | 0.25 (0.2–0.3) | 0.23 |
| `sight` | -0.007 (-0.14–0.12) | 0.2 (0.17–0.24) | 0.21 |
| `clearance` | 0.096 (-0.058–0.24) | 0.26 (0.22–0.31) | 0.16 |
| `seed` | 0.1 (-0.037–0.24) | 0.23 (0.19–0.27) | 0.12 |
| sum | 0.91 | 2.4 | |

### Crashed

| Factor | First-order | Total | Interactions |
|---|---|---|---|
| `e` | -0.041 (-0.14–0.059) | 0.39 (0.3–0.48) | 0.44 |
| `ψe` | 0.02 (-0.091–0.13) | 0.49 (0.4–0.58) | 0.47 |
| `κ_profile` | 0.37 (0.22–0.53) | 0.87 (0.78–0.97) | 0.5 |
| `widths` | -0.02 (-0.11–0.07) | 0.37 (0.29–0.44) | 0.39 |
| `sight` | -0.041 (-0.12–0.032) | 0.32 (0.23–0.38) | 0.36 |
| `clearance` | -4.5e-18 (-0.098–0.097) | 0.38 (0.31–0.47) | 0.38 |
| `seed` | -0.051 (-0.14–0.051) | 0.35 (0.28–0.44) | 0.4 |
| sum | 0.24 | 3.2 | |

### Crashes: `κ_profile` and `ψe` together

The two estimates whose errors interact the most in crashes (total less first-order index). Share of
runs that crashed (of how many), by the gain on `κ_profile`'s errors, and whether `ψe`'s are at most the car
network's (gain ≤ 1) or larger; the other estimates' gains anywhere from 0 to 2.

| `κ_profile` gain | `ψe` gain ≤ 1 | `ψe` gain > 1 |
|---|---|---|
| 0.0–0.5 | 0% (366) | 3% (404) |
| 0.5–1.0 | 4% (390) | 4% (382) |
| 1.0–1.5 | 12% (395) | 14% (389) |
| 1.5–2.0 | 29% (376) | 25% (370) |

### RMS deviation, by situation: total indices

| Factor | straight | gentle entry | gentle middle | gentle exit | medium entry | medium middle | medium exit | tight entry | tight middle | tight exit |
|---|---|---|---|---|---|---|---|---|---|---|
| `e` | 0.19 | 0.22 | 0.13 | 0.29 | 0.12 | 0.079 | 0.13 | 0.077 | 0.17 | 0.14 |
| `ψe` | 0.17 | 0.28 | 0.14 | 0.16 | 0.16 | 0.09 | 0.082 | 0.082 | 0.2 | 0.15 |
| `κ_profile` | 0.9 | 0.85 | 0.95 | 0.82 | 0.98 | 0.9 | 0.92 | 0.83 | 0.99 | 0.94 |
| `widths` | 0.15 | 0.22 | 0.1 | 0.13 | 0.088 | 0.051 | 0.05 | 0.25 | 0.16 | 0.13 |
| `sight` | 0.14 | 0.16 | 0.13 | 0.2 | 0.096 | 0.066 | 0.067 | 0.08 | 0.18 | 0.099 |
| `clearance` | 0.18 | 0.26 | 0.2 | 0.23 | 0.18 | 0.16 | 0.096 | 0.091 | 0.23 | 0.15 |
| `seed` | 0.17 | 0.25 | 0.11 | 0.2 | 0.15 | 0.072 | 0.075 | 0.082 | 0.15 | 0.13 |
