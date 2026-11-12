# The error models against the networks' real errors

Each network's errors as the oracle draws them (TrajectorySensitivity.jl's `NetworkErrors`,
parameters in [error_model.toml](error_model.toml)) against the network's real errors, over the
same drives on the held-out tracks (Spa, Monza, Silverstone, Oschersleben, YasMarina) and the same entries (profiles: those
the car can see). The model's errors are drawn from scratch (stationary from the start of each
drive), not replayed. Each cell: real / model. |error| quantiles and RMS in the estimate's unit;
"calm" and "hard": the median |error| in the calmest half of the situations and the hardest 3%
(`situation`: each entry's true value, for e and ψe the bend beside the car; for κ_profile,
straight against sharply bending lane); "within 1σ": how often the error is smaller than the
network's own σ; ρ: the errors' correlation
1 and 20 scans apart (25 ms and 0.5 s); "1 m apart": their correlation between entries 1 m apart
along a profile, or 4 arcs apart on the clearance fan. From [`error_model.jl`](../error_model.jl).

| Estimate | Network | median | 90% | 99% | calm, median | hard, median | RMS | within 1σ | ρ 1 scan | ρ 20 scans | 1 m apart |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `e` | small_weak | 0.00475 / 0.00469 | 0.0126 / 0.0126 | 0.0298 / 0.0274 | 0.00442 / 0.00435 | 0.0109 / 0.0102 | 0.00967 / 0.00851 | 66% / 66% | 0.65 / 0.68 | -0.01 / -0.02 | – |
| `ψe` | small_weak | 0.00665 / 0.00671 | 0.0203 / 0.0212 | 0.117 / 0.0785 | 0.0053 / 0.0053 | 0.0424 / 0.0389 | 0.0312 / 0.0207 | 67% / 67% | 0.77 / 0.58 | -0.05 / 0.03 | – |
| `κ_profile` | small_weak | 0.00762 / 0.00747 | 0.0704 / 0.0699 | 0.395 / 0.502 | 0.0036 / 0.00371 | 0.261 / 0.225 | 0.0916 / 0.173 | 81% / 79% | 0.95 / 0.86 | -0.13 / 0.06 | 0.18 / 0.23 |
| `widths` | small_weak | 0.0192 / 0.0183 | 0.0478 / 0.0492 | 0.0805 / 0.106 | 0.02 / 0.0192 | 0.0485 / 0.0515 | 0.0995 / 0.139 | 63% / 62% | 0.67 / 0.70 | 0.00 / 0.05 | 0.25 / 0.12 |
| `sight` | small_weak | 0.0294 / 0.0289 | 0.245 / 0.258 | 0.777 / 1.15 | 0.0174 / 0.0177 | 0.143 / 0.139 | 0.222 / 0.484 | 82% / 81% | 0.69 / 0.65 | 0.01 / 0.03 | – |
| `clearance` | small_weak | 0.0285 / 0.0286 | 0.14 / 0.138 | 1.38 / 1.1 | 0.0227 / 0.0228 | 0.149 / 0.139 | 0.371 / 0.606 | 65% / 61% | 0.90 / 0.82 | 0.09 / 0.13 | -0.00 / 0.00 |
| `e` | large | 0.00394 / 0.00393 | 0.0109 / 0.0109 | 0.025 / 0.0232 | 0.00371 / 0.00361 | 0.01 / 0.00907 | 0.00789 / 0.00722 | 65% / 65% | 0.68 / 0.72 | 0.02 / 0.01 | – |
| `ψe` | large | 0.00505 / 0.00512 | 0.0161 / 0.017 | 0.0899 / 0.0649 | 0.00393 / 0.00398 | 0.0359 / 0.0329 | 0.025 / 0.0179 | 69% / 68% | 0.71 / 0.43 | -0.02 / 0.00 | – |
| `κ_profile` | large | 0.0053 / 0.00534 | 0.0508 / 0.0483 | 0.286 / 0.318 | 0.00247 / 0.00258 | 0.153 / 0.129 | 0.0714 / 0.0984 | 79% / 78% | 0.93 / 0.77 | -0.11 / 0.03 | -0.06 / 0.16 |
| `widths` | large | 0.017 / 0.0158 | 0.0447 / 0.0465 | 0.0759 / 0.105 | 0.0178 / 0.0172 | 0.044 / 0.0468 | 0.0985 / 0.146 | 65% / 65% | 0.67 / 0.64 | -0.00 / 0.04 | 0.22 / 0.08 |
| `sight` | large | 0.0222 / 0.0219 | 0.137 / 0.141 | 0.468 / 0.635 | 0.0141 / 0.0144 | 0.0787 / 0.0765 | 0.168 / 0.182 | 64% / 59% | 0.65 / 0.50 | -0.00 / -0.01 | – |
| `clearance` | large | 0.0239 / 0.0238 | 0.0733 / 0.0754 | 0.65 / 0.455 | 0.0201 / 0.0202 | 0.0292 / 0.028 | 0.218 / 0.302 | 60% / 58% | 0.74 / 0.71 | -0.01 / 0.05 | 0.00 / 0.00 |
