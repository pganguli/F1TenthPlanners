# The oracle against the real networks

Each estimate from the real network on the car (the others true), against the oracle with that
network's recorded errors (replayed by the true lane, never from the run's own track: what
sensitivity.jl and cloud.jl use) and with its modeled errors (error_model.jl), on the tracks the
networks never trained on and the representative track (Hockenheim, Silverstone, YasMarina, Spa, Sepang, Oschersleben, Representative, Monza), 1 lap each (2 on the
representative track), 5 seeds (the real networks: the LiDAR's noise;
the oracle: its draws). From [`calibrate.jl`](../calibrate.jl), summed up by [`report.jl`](../report.jl);
95% intervals in brackets. The oracle is right where its intervals overlap the real network's.


### The car's networks (small_weak)

| Estimate | Errors from | Runs | Crashes per km | RMS deviation [m] | Least gap [m] | Scans with TTC < 0.5 s [%] |
|---|---|---|---|---|---|---|
| `e` | real network | 40 | 0.0 (0.0–0.22) | 0.005 (0.0046–0.0053) | 0.39 (0.36–0.41) | 0.053 (0.019–0.1) |
| `e` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.004 (0.0038–0.0043) | 0.39 (0.36–0.41) | 0.041 (0.015–0.079) |
| `e` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.0055 (0.0053–0.0057) | 0.38 (0.36–0.41) | 0.036 (0.012–0.069) |
| `ψe` | real network | 40 | 0.0 (0.0–0.22) | 0.013 (0.012–0.014) | 0.39 (0.37–0.41) | 0.072 (0.03–0.13) |
| `ψe` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.012 (0.011–0.013) | 0.38 (0.36–0.4) | 0.09 (0.033–0.16) |
| `ψe` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.016 (0.016–0.017) | 0.38 (0.36–0.41) | 0.077 (0.034–0.13) |
| `κ_profile` | real network | 40 | 1.1 (0.62–1.9) | 0.067 (0.059–0.076) | 0.24 (0.2–0.29) | 0.82 (0.48–1.3) |
| `κ_profile` | oracle, recorded errors | 40 | 0.061 (0.0015–0.34) | 0.055 (0.051–0.06) | 0.28 (0.24–0.33) | 0.59 (0.38–0.84) |
| `κ_profile` | oracle, error model | 40 | 1.9 (1.2–2.8) | 0.13 (0.12–0.14) | 0.096 (0.079–0.12) | 2.8 (2.5–3.2) |
| `widths` | real network | 40 | 0.0 (0.0–0.22) | 0.00038 (0.00018–0.00059) | 0.39 (0.36–0.41) | 0.043 (0.0088–0.08) |
| `widths` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.0047 (0.0024–0.0073) | 0.38 (0.36–0.41) | 0.043 (0.0087–0.08) |
| `widths` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.013 (0.0082–0.017) | 0.38 (0.36–0.41) | 0.064 (0.026–0.11) |
| `sight` | real network | 40 | 0.0 (0.0–0.22) | 0.0071 (0.0064–0.0078) | 0.39 (0.37–0.41) | 0.04 (0.011–0.075) |
| `sight` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.0093 (0.008–0.011) | 0.39 (0.37–0.41) | 0.04 (0.013–0.073) |
| `sight` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.02 (0.017–0.023) | 0.39 (0.36–0.42) | 0.055 (0.017–0.11) |
| `clearance` | real network | 40 | 0.0 (0.0–0.22) | 0.017 (0.017–0.018) | 0.4 (0.38–0.43) | 0.011 (0.0031–0.021) |
| `clearance` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.015 (0.014–0.015) | 0.39 (0.37–0.42) | 0.022 (0.005–0.042) |
| `clearance` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.015 (0.015–0.016) | 0.4 (0.37–0.42) | 0.011 (0.0017–0.022) |

### The cloud's networks (large), on the car without delay

| Estimate | Errors from | Runs | Crashes per km | RMS deviation [m] | Least gap [m] | Scans with TTC < 0.5 s [%] |
|---|---|---|---|---|---|---|
| `e` | real network | 40 | 0.0 (0.0–0.22) | 0.0047 (0.0044–0.0051) | 0.39 (0.36–0.41) | 0.052 (0.019–0.099) |
| `e` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.0039 (0.0036–0.0041) | 0.39 (0.36–0.41) | 0.038 (0.014–0.072) |
| `e` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.0049 (0.0047–0.0051) | 0.39 (0.36–0.41) | 0.036 (0.012–0.07) |
| `ψe` | real network | 40 | 0.0 (0.0–0.22) | 0.011 (0.011–0.012) | 0.38 (0.36–0.41) | 0.058 (0.025–0.1) |
| `ψe` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.011 (0.01–0.011) | 0.38 (0.36–0.4) | 0.069 (0.021–0.13) |
| `ψe` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.012 (0.011–0.012) | 0.38 (0.36–0.41) | 0.057 (0.022–0.097) |
| `κ_profile` | real network | 40 | 0.0 (0.0–0.22) | 0.025 (0.022–0.027) | 0.36 (0.33–0.38) | 0.092 (0.045–0.15) |
| `κ_profile` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.019 (0.018–0.02) | 0.38 (0.35–0.4) | 0.044 (0.016–0.078) |
| `κ_profile` | oracle, error model | 40 | 1.0 (0.55–1.7) | 0.093 (0.086–0.1) | 0.18 (0.15–0.22) | 1.4 (1.2–1.7) |
| `widths` | real network | 40 | 0.0 (0.0–0.22) | 0.00031 (0.00015–0.00049) | 0.39 (0.36–0.41) | 0.043 (0.0088–0.08) |
| `widths` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.0051 (0.0025–0.0079) | 0.39 (0.36–0.41) | 0.043 (0.0087–0.08) |
| `widths` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.012 (0.0077–0.016) | 0.38 (0.36–0.41) | 0.066 (0.024–0.12) |
| `sight` | real network | 40 | 0.0 (0.0–0.22) | 0.0039 (0.0036–0.0042) | 0.38 (0.36–0.41) | 0.036 (0.0092–0.066) |
| `sight` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.0051 (0.0045–0.0057) | 0.38 (0.36–0.41) | 0.039 (0.0079–0.072) |
| `sight` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.01 (0.0084–0.012) | 0.38 (0.36–0.41) | 0.042 (0.015–0.075) |
| `clearance` | real network | 40 | 0.0 (0.0–0.22) | 0.0089 (0.0082–0.0097) | 0.39 (0.37–0.41) | 0.037 (0.0078–0.069) |
| `clearance` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.0093 (0.0084–0.01) | 0.39 (0.37–0.42) | 0.033 (0.0077–0.062) |
| `clearance` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.0069 (0.0065–0.0074) | 0.39 (0.36–0.41) | 0.034 (0.0097–0.064) |
