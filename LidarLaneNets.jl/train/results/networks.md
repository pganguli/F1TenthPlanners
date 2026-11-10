# The networks: cost and accuracy

Every network set against the hand-written estimator (LidarLaneEstimation.jl), on the tracks never
seen in training (Spa, Monza, Silverstone, Oschersleben, YasMarina; 24000 scans while driving). Validation:
Hockenheim, Sepang (14600 scans). From [`evaluate.jl`](../evaluate.jl), which
explains each column. The hand-written estimator found no lane in 0.0% of the
test scans (it then gives zeros). The error distributions are in [errors.md](errors.md).

| Network set | Runs on | Weights | Beams in | MACs per scan, all six | Parameters, all six |
|---|---|---|---|---|---|
| small_weak | the car (edge) | [`weights/small_weak_*.jld2`](../../weights) | 136 | 106,912 | 37,014 |
| small_strong | (for comparison) | [`weights/small_strong_*.jld2`](../../weights) | 271 | 778,496 | 236,230 |
| large | the cloud | [`weights/large_*.jld2`](../../weights) | 1081 | 25,480,800 | 4,285,804 |

| Estimate | Estimator | Weights | Beams in | MACs per scan | Parameters | Validation R² | Test RMS error | Unit | Entries given | Within 1σ | Error correlation with large |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `e` | hand-written | – | 1081 | – | – | – | 0.05126 | m | 100% | – | – |
| `e` | small_weak | [`small_weak_e.jld2`](../../weights/small_weak_e.jld2) | 136 | 16,656 | 4,970 | 0.9982 | 0.009667 | m | 100% | 66% | 0.73 |
| `e` | small_strong | [`small_strong_e.jld2`](../../weights/small_strong_e.jld2) | 271 | 127,424 | 37,010 | 0.9986 | 0.00806 | m | 100% | 68% | 0.99 |
| `e` | large | [`large_e.jld2`](../../weights/large_e.jld2) | 1081 | 4,239,824 | 707,252 | 0.9986 | 0.007892 | m | 100% | 65% | – |
| `ψe` | hand-written | – | 1081 | – | – | – | 0.08705 | rad | 100% | – | – |
| `ψe` | small_weak | [`small_weak_psi_e.jld2`](../../weights/small_weak_psi_e.jld2) | 136 | 16,656 | 4,970 | 0.9267 | 0.03121 | rad | 100% | 67% | 0.88 |
| `ψe` | small_strong | [`small_strong_psi_e.jld2`](../../weights/small_strong_psi_e.jld2) | 271 | 127,424 | 37,010 | 0.9323 | 0.02528 | rad | 100% | 68% | 0.99 |
| `ψe` | large | [`large_psi_e.jld2`](../../weights/large_psi_e.jld2) | 1081 | 4,239,824 | 707,252 | 0.933 | 0.02496 | rad | 100% | 69% | – |
| `κ_profile` | hand-written | – | 1081 | – | – | – | 0.0572 | 1/m | 95% | – | – |
| `κ_profile` | small_weak | [`small_weak_kappa_profile.jld2`](../../weights/small_weak_kappa_profile.jld2) | 136 | 18,704 | 7,082 | 0.6907 | 0.09164 | 1/m | 100% | 81% | 0.83 |
| `κ_profile` | small_strong | [`small_strong_kappa_profile.jld2`](../../weights/small_strong_kappa_profile.jld2) | 271 | 131,520 | 41,170 | 0.7848 | 0.07986 | 1/m | 100% | 83% | 0.93 |
| `κ_profile` | large | [`large_kappa_profile.jld2`](../../weights/large_kappa_profile.jld2) | 1081 | 4,252,112 | 719,668 | 0.8117 | 0.07144 | 1/m | 100% | 79% | – |
| `widths` | hand-written | – | 1081 | – | – | – | 0.1014 | m | 95% | – | – |
| `widths` | small_weak | [`small_weak_widths.jld2`](../../weights/small_weak_widths.jld2) | 136 | 20,816 | 9,260 | 0.702 | 0.09949 | m | 100% | 63% | 0.99 |
| `widths` | small_strong | [`small_strong_widths.jld2`](../../weights/small_strong_widths.jld2) | 271 | 135,744 | 45,460 | 0.7108 | 0.09844 | m | 100% | 64% | 1.0 |
| `widths` | large | [`large_widths.jld2`](../../weights/large_widths.jld2) | 1081 | 4,264,784 | 732,472 | 0.7112 | 0.0985 | m | 100% | 65% | – |
| `sight` | hand-written | – | 1081 | – | – | – | 1.726 | m | 100% | – | – |
| `sight` | small_weak | [`small_weak_sight.jld2`](../../weights/small_weak_sight.jld2) | 136 | 16,656 | 4,970 | 0.9858 | 0.222 | m | 100% | 82% | 0.75 |
| `sight` | small_strong | [`small_strong_sight.jld2`](../../weights/small_strong_sight.jld2) | 271 | 127,424 | 37,010 | 0.9905 | 0.1857 | m | 100% | 73% | 0.93 |
| `sight` | large | [`large_sight.jld2`](../../weights/large_sight.jld2) | 1081 | 4,239,824 | 707,252 | 0.992 | 0.1685 | m | 100% | 64% | – |
| `clearance` | hand-written | – | 1081 | – | – | – | 0.8553 | m | 100% | – | – |
| `clearance` | small_weak | [`small_weak_clearance.jld2`](../../weights/small_weak_clearance.jld2) | 136 | 17,424 | 5,762 | 0.7182 | 0.3715 | m | 100% | 65% | 0.61 |
| `clearance` | small_strong | [`small_strong_clearance.jld2`](../../weights/small_strong_clearance.jld2) | 271 | 128,960 | 38,570 | 0.8435 | 0.2815 | m | 100% | 65% | 0.84 |
| `clearance` | large | [`large_clearance.jld2`](../../weights/large_clearance.jld2) | 1081 | 4,244,432 | 711,908 | 0.8997 | 0.2179 | m | 100% | 60% | – |
