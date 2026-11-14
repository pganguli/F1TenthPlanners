# The oracle against the real networks

Each estimate from the real network on the car (the others true), against the oracle with that
network's recorded errors (replayed by the true lane, never from the run's own track: what
sensitivity.jl and cloud.jl use), with them including the run's own track's (a check of the replay
itself: whether it reproduces the network where it has that track's errors), and with its modeled
errors (error_model.jl), on the tracks the
networks never trained on and the representative track (Hockenheim, Oschersleben, Representative, YasMarina, Monza, Silverstone, Sepang, Spa), 1 lap each (2 on the
representative track), 5 seeds (the real networks: the LiDAR's noise;
the oracle: its draws). From [`calibrate.jl`](../calibrate.jl), summed up by [`report.jl`](../report.jl);
95% intervals in brackets. The oracle is right where its intervals overlap the real network's.


### The car's networks (small_weak)

| Estimate | Errors from | Runs | Crashes per km | RMS deviation [m] | Least gap [m] | Scans with TTC < 0.5 s [%] |
|---|---|---|---|---|---|---|
| `e` | real network | 40 | 0.0 (0.0–0.22) | 0.0051 (0.0047–0.0056) | 0.4 (0.38–0.43) | 0.031 (0.011–0.06) |
| `e` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.0045 (0.0042–0.0047) | 0.4 (0.38–0.43) | 0.0058 (0.00079–0.014) |
| `e` | oracle, recorded errors with the track's own | 40 | 0.0 (0.0–0.22) | 0.005 (0.0046–0.0055) | 0.4 (0.38–0.43) | 0.0058 (0.00079–0.014) |
| `e` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.0053 (0.0051–0.0056) | 0.4 (0.37–0.42) | 0.011 (0.0017–0.022) |
| `ψe` | real network | 40 | 0.0 (0.0–0.22) | 0.013 (0.012–0.014) | 0.4 (0.38–0.43) | 0.049 (0.012–0.094) |
| `ψe` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.012 (0.011–0.013) | 0.38 (0.36–0.41) | 0.051 (0.011–0.099) |
| `ψe` | oracle, recorded errors with the track's own | 40 | 0.0 (0.0–0.22) | 0.012 (0.011–0.013) | 0.4 (0.38–0.43) | 0.051 (0.011–0.099) |
| `ψe` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.016 (0.015–0.017) | 0.4 (0.37–0.43) | 0.029 (0.011–0.051) |
| `κ_profile` | real network | 40 | 1.1 (0.62–1.9) | 0.067 (0.059–0.075) | 0.27 (0.22–0.32) | 0.84 (0.49–1.3) |
| `κ_profile` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.053 (0.049–0.057) | 0.3 (0.25–0.34) | 0.45 (0.28–0.63) |
| `κ_profile` | oracle, recorded errors with the track's own | 40 | 0.57 (0.25–1.1) | 0.053 (0.048–0.058) | 0.28 (0.23–0.32) | 0.71 (0.43–1.0) |
| `κ_profile` | oracle, error model | 40 | 2.6 (1.7–3.8) | 0.12 (0.11–0.13) | 0.087 (0.075–0.1) | 2.4 (2.0–2.8) |
| `widths` | real network | 40 | 0.0 (0.0–0.22) | 0.00076 (0.00043–0.0011) | 0.39 (0.37–0.42) | 0.0082 (0.0017–0.016) |
| `widths` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.007 (0.004–0.01) | 0.39 (0.37–0.42) | 0.011 (0.0032–0.021) |
| `widths` | oracle, recorded errors with the track's own | 40 | 0.0 (0.0–0.22) | 0.0047 (0.0015–0.0087) | 0.39 (0.37–0.42) | 0.016 (0.0047–0.033) |
| `widths` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.011 (0.0067–0.015) | 0.4 (0.37–0.42) | 0.025 (0.0063–0.05) |
| `sight` | real network | 40 | 0.0 (0.0–0.22) | 0.0061 (0.0056–0.0067) | 0.41 (0.38–0.43) | 0.009 (0.0024–0.018) |
| `sight` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.0074 (0.0064–0.0085) | 0.4 (0.38–0.43) | 0.0098 (0.0023–0.02) |
| `sight` | oracle, recorded errors with the track's own | 40 | 0.0 (0.0–0.22) | 0.0065 (0.0059–0.0072) | 0.41 (0.38–0.43) | 0.0098 (0.0023–0.02) |
| `sight` | oracle, error model | 40 | 0.061 (0.0015–0.34) | 0.016 (0.013–0.019) | 0.39 (0.36–0.43) | 0.041 (0.0048–0.1) |
| `clearance` | real network | 40 | 0.0 (0.0–0.22) | 0.012 (0.012–0.013) | 0.41 (0.39–0.44) | 0.0056 (0.00082–0.012) |
| `clearance` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.012 (0.011–0.012) | 0.41 (0.39–0.43) | 0.0 (0.0–0.0) |
| `clearance` | oracle, recorded errors with the track's own | 40 | 0.0 (0.0–0.22) | 0.012 (0.011–0.013) | 0.42 (0.4–0.45) | 0.0 (0.0–0.0) |
| `clearance` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.012 (0.011–0.012) | 0.4 (0.38–0.43) | 0.004 (0.00078–0.0087) |

### The cloud's networks (large), on the car without delay

| Estimate | Errors from | Runs | Crashes per km | RMS deviation [m] | Least gap [m] | Scans with TTC < 0.5 s [%] |
|---|---|---|---|---|---|---|
| `e` | real network | 40 | 0.0 (0.0–0.22) | 0.0045 (0.0041–0.0048) | 0.4 (0.38–0.43) | 0.025 (0.005–0.049) |
| `e` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.0042 (0.0039–0.0044) | 0.4 (0.38–0.43) | 0.0074 (0.0016–0.016) |
| `e` | oracle, recorded errors with the track's own | 40 | 0.0 (0.0–0.22) | 0.0046 (0.0042–0.0049) | 0.4 (0.38–0.43) | 0.0074 (0.0016–0.016) |
| `e` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.0047 (0.0045–0.0049) | 0.4 (0.37–0.43) | 0.0082 (0.0008–0.019) |
| `ψe` | real network | 40 | 0.0 (0.0–0.22) | 0.01 (0.0099–0.011) | 0.4 (0.37–0.42) | 0.037 (0.0078–0.073) |
| `ψe` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.0099 (0.0094–0.01) | 0.39 (0.36–0.41) | 0.048 (0.01–0.093) |
| `ψe` | oracle, recorded errors with the track's own | 40 | 0.0 (0.0–0.22) | 0.01 (0.0097–0.011) | 0.4 (0.37–0.42) | 0.048 (0.01–0.094) |
| `ψe` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.012 (0.011–0.012) | 0.4 (0.38–0.43) | 0.022 (0.0069–0.042) |
| `κ_profile` | real network | 40 | 0.0 (0.0–0.22) | 0.025 (0.022–0.027) | 0.36 (0.33–0.39) | 0.072 (0.034–0.12) |
| `κ_profile` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.019 (0.017–0.02) | 0.39 (0.36–0.42) | 0.029 (0.012–0.05) |
| `κ_profile` | oracle, recorded errors with the track's own | 40 | 0.0 (0.0–0.22) | 0.023 (0.021–0.026) | 0.36 (0.33–0.39) | 0.045 (0.019–0.077) |
| `κ_profile` | oracle, error model | 40 | 0.59 (0.27–1.1) | 0.088 (0.083–0.094) | 0.2 (0.16–0.23) | 1.0 (0.87–1.2) |
| `widths` | real network | 40 | 0.0 (0.0–0.22) | 0.00078 (0.00045–0.0011) | 0.39 (0.37–0.42) | 0.0082 (0.0017–0.016) |
| `widths` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.0066 (0.0041–0.0093) | 0.39 (0.37–0.42) | 0.011 (0.0025–0.021) |
| `widths` | oracle, recorded errors with the track's own | 40 | 0.0 (0.0–0.22) | 0.0037 (0.0012–0.0066) | 0.39 (0.37–0.42) | 0.016 (0.0048–0.032) |
| `widths` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.01 (0.0062–0.014) | 0.4 (0.37–0.42) | 0.025 (0.0065–0.046) |
| `sight` | real network | 40 | 0.0 (0.0–0.22) | 0.004 (0.0036–0.0043) | 0.39 (0.37–0.42) | 0.0082 (0.0016–0.018) |
| `sight` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.0051 (0.0041–0.0065) | 0.39 (0.37–0.42) | 0.011 (0.0031–0.021) |
| `sight` | oracle, recorded errors with the track's own | 40 | 0.0 (0.0–0.22) | 0.0048 (0.0039–0.0061) | 0.4 (0.37–0.43) | 0.011 (0.0031–0.021) |
| `sight` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.01 (0.0079–0.014) | 0.39 (0.35–0.42) | 0.054 (0.0059–0.13) |
| `clearance` | real network | 40 | 0.0 (0.0–0.22) | 0.0078 (0.0072–0.0083) | 0.4 (0.38–0.43) | 0.012 (0.0032–0.024) |
| `clearance` | oracle, recorded errors | 40 | 0.0 (0.0–0.22) | 0.0085 (0.0077–0.0092) | 0.4 (0.38–0.43) | 0.0057 (0.0016–0.012) |
| `clearance` | oracle, recorded errors with the track's own | 40 | 0.0 (0.0–0.22) | 0.0078 (0.0072–0.0085) | 0.4 (0.38–0.43) | 0.0057 (0.0016–0.012) |
| `clearance` | oracle, error model | 40 | 0.0 (0.0–0.22) | 0.0062 (0.006–0.0065) | 0.4 (0.38–0.43) | 0.011 (0.0023–0.024) |
