# Per-scan accuracy

Test tracks: Spa, Monza, Silverstone, Oschersleben, YasMarina (24000 scans while driving). RMS error against the true values; profiles over the entries the car can see.

| Estimate | Unit | Hand-written | (entries given) | Small DNN | Large DNN | Small within 1σ | Large within 1σ | Error correlation, small vs large |
|---|---|---|---|---|---|---|---|---|
| `e` | m | 0.0513 | 100% | 0.0081 | 0.0079 | 68% | 65% | 0.99 |
| `ψe` | rad | 0.0870 | 100% | 0.0253 | 0.0250 | 68% | 69% | 0.99 |
| `κ_profile` | 1/m | 0.0572 | 95% | 0.0799 | 0.0714 | 83% | 79% | 0.93 |
| `widths` | m | 0.1014 | 95% | 0.0984 | 0.0985 | 64% | 65% | 1.00 |
| `sight` | m | 1.7258 | 100% | 0.1857 | 0.1685 | 73% | 64% | 0.93 |
| `clearance` | m | 0.8553 | 100% | 0.2815 | 0.2179 | 65% | 60% | 0.84 |

The hand-written estimator found no lane in 0.0% of all test scans (it then gives zeros).
