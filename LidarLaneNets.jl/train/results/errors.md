# How the networks' errors are distributed

Errors (estimate − truth) on the held-out tracks (Spa, Monza, Silverstone, Oschersleben, YasMarina) while driving, over the
same entries as the RMS errors (profiles: the entries the car can see, pooled over the distance
ahead; at most 200000 errors per fit, a random subset). Each candidate fitted by maximum
likelihood. Student t: location μ, scale s, degrees of freedom ν (tails heavier the smaller ν; a
Gaussian as ν → ∞, a Cauchy at ν = 1). AIC differences: how much better than the Gaussian (larger is
better; differences above ~10 are decisive). Tail check: how often each fitted model puts an error
beyond the observed 99.9th percentile of |error − μ|; a model whose tails are right says 0.1%.
From [`errors.jl`](../errors.jl). The networks are those of [networks.md](networks.md).

## The errors

| Estimate | Network | Best fit (AIC) | Gaussian | Laplace | Student t | AIC better than Gaussian by: Laplace / t | Beyond the observed 99.9th percentile, % (Gaussian / Laplace / t; 0.1 is right) | Unit |
|---|---|---|---|---|---|---|---|---|
| `e` | small_strong | Student t | μ -0.000277, σ 0.00806 | μ -0.000196, b 0.00541 | μ -0.000185, s 0.00531, **ν 3.9** | 5970.0 / 7100.0 | 2.4e-16 / 0.00022 / 0.022 | m |
| `e` | small_weak | Student t | μ -0.000155, σ 0.00967 | μ -0.000186, b 0.00632 | μ -0.000195, s 0.00616, **ν 3.9** | 7250.0 / 8700.0 | 1.2e-17 / 9.4e-5 / 0.017 | m |
| `e` | large | Student t | μ -0.000314, σ 0.00789 | μ -0.00016, b 0.0053 | μ -0.000154, s 0.0052, **ν 3.9** | 5890.0 / 6980.0 | 1.3e-14 / 0.00045 / 0.027 | m |
| `ψe` | small_strong | Student t | μ 0.00256, σ 0.0252 | μ 0.00279, b 0.00937 | μ 0.00278, s 0.00628, **ν 2.1** | 34200.0 / 41600.0 | 4.1e-40 / 1.4e-14 / 0.027 | rad |
| `ψe` | small_weak | Student t | μ 0.000311, σ 0.0312 | μ 0.000592, b 0.0118 | μ 0.000616, s 0.00781, **ν 2.1** | 33600.0 / 41200.0 | 3.2e-38 / 5.2e-14 / 0.029 | rad |
| `ψe` | large | Student t | μ -0.000407, σ 0.025 | μ -0.000405, b 0.00912 | μ -0.000373, s 0.00596, **ν 2.0** | 35200.0 / 42800.0 | 5.2e-40 / 7.2e-15 / 0.032 | rad |
| `κ_profile` | small_strong | Student t | μ -0.00149, σ 0.0806 | μ -5.68e-5, b 0.0258 | μ -3.57e-5, s 0.0049, **ν 0.77** | 346000.0 / 537000.0 | 1.6e-34 / 7.5e-16 / 1.0 | 1/m |
| `κ_profile` | small_weak | Student t | μ -0.00308, σ 0.0926 | μ -0.00022, b 0.0313 | μ -0.000135, s 0.00695, **ν 0.82** | 324000.0 / 488000.0 | 6.0999999999999995e-33 / 1.4e-14 / 0.95 | 1/m |
| `κ_profile` | large | Student t | μ -0.000723, σ 0.0717 | μ 0.000105, b 0.0229 | μ 0.00011, s 0.00471, **ν 0.79** | 346000.0 / 525000.0 | 4.6999999999999995e-38 / 1.1e-16 / 0.94 | 1/m |
| `widths` | small_strong | Student t | μ -0.00319, σ 0.103 | μ -0.000688, b 0.0244 | μ -0.000573, s 0.0225, **ν 4.6** | 466000.0 / 518000.0 | 3.9e-12 / 1.4e-12 / 8.7e-5 | m |
| `widths` | small_weak | Student t | μ -0.00262, σ 0.101 | μ 0.000491, b 0.0263 | μ 0.000557, s 0.0251, **ν 5.2** | 428000.0 / 477000.0 | 2.0e-11 / 5.5e-11 / 5.5e-5 | m |
| `widths` | large | Student t | μ -0.00122, σ 0.103 | μ 0.00133, b 0.0245 | μ 0.00147, s 0.0226, **ν 4.6** | 466000.0 / 518000.0 | 3.8e-12 / 1.4e-12 / 8.7e-5 | m |
| `sight` | small_strong | Student t | μ -0.000285, σ 0.186 | μ -0.00777, b 0.0702 | μ -0.00803, s 0.0327, **ν 1.3** | 33500.0 / 41300.0 | 3.1e-42 / 9.3e-15 / 0.28 | m |
| `sight` | small_weak | Student t | μ -0.00203, σ 0.222 | μ -0.00852, b 0.0926 | μ -0.00818, s 0.0278, **ν 0.88** | 28800.0 / 39500.0 | 4.7e-29 / 8.6e-11 / 1.2 | m |
| `sight` | large | Student t | μ 0.000307, σ 0.168 | μ -0.00432, b 0.0583 | μ -0.00423, s 0.024, **ν 1.1** | 37700.0 / 46600.0 | 6.1e-40 / 9.6e-16 / 0.39 | m |
| `clearance` | small_strong | Student t | μ -0.000314, σ 0.281 | μ 0.00412, b 0.0613 | μ 0.00365, s 0.026, **ν 1.6** | 500000.0 / 671000.0 | 2.8e-56 / 8.499999999999999e-31 / 0.025 | m |
| `clearance` | small_weak | Student t | μ -0.00968, σ 0.37 | μ 0.00596, b 0.0921 | μ 0.0055, s 0.0299, **ν 1.2** | 447000.0 / 643000.0 | 7.299999999999999e-55 / 1.8e-26 / 0.11 | m |
| `clearance` | large | Student t | μ 0.013, σ 0.218 | μ 0.0105, b 0.0517 | μ 0.0101, s 0.026, **ν 1.8** | 466000.0 / 606000.0 | 2.1999999999999998e-48 / 4.5e-26 / 0.015 | m |

## The errors over the network's own standard deviation

What the selector and the Kalman filter assume is a standard Gaussian (μ 0, σ 1).

| Estimate | Network | Best fit (AIC) | Gaussian | Laplace | Student t | AIC better than Gaussian by: Laplace / t | Beyond the observed 99.9th percentile, % (Gaussian / Laplace / t; 0.1 is right) |
|---|---|---|---|---|---|---|---|
| `e` | small_strong | Student t | μ -0.0355, σ 1.1 | μ -0.0324, b 0.839 | μ -0.0302, s 0.937, **ν 7.1** | -43.2 / 859.0 | 0.001 / 0.3 / 0.12 |
| `e` | small_weak | Student t | μ -0.0352, σ 1.12 | μ -0.0273, b 0.869 | μ -0.0304, s 0.998, **ν 9.4** | -812.0 / 482.0 | 0.0027 / 0.44 / 0.095 |
| `e` | large | Student t | μ -0.035, σ 1.18 | μ -0.0301, b 0.899 | μ -0.0277, s 1.0, **ν 7.1** | -10.8 / 877.0 | 0.0013 / 0.32 / 0.13 |
| `ψe` | small_strong | Student t | μ 0.31, σ 1.06 | μ 0.328, b 0.793 | μ 0.322, s 0.864, **ν 6.0** | 539.0 / 1300.0 | 9.0e-5 / 0.14 / 0.098 |
| `ψe` | small_weak | Student t | μ 0.0577, σ 1.14 | μ 0.0589, b 0.854 | μ 0.0621, s 0.933, **ν 6.2** | 529.0 / 1390.0 | 8.3e-5 / 0.14 / 0.086 |
| `ψe` | large | Student t | μ -0.0599, σ 1.11 | μ -0.0525, b 0.827 | μ -0.0516, s 0.886, **ν 5.4** | 1050.0 / 1620.0 | 0.00014 / 0.15 / 0.14 |
| `κ_profile` | small_strong | Student t | μ 0.00273, σ 0.864 | μ -0.00575, b 0.602 | μ -0.00309, s 0.574, **ν 3.4** | 35200.0 / 38600.0 | 1.3e-7 / 0.016 / 0.17 |
| `κ_profile` | small_weak | Student t | μ -0.0113, σ 1.0 | μ -0.0174, b 0.658 | μ -0.0166, s 0.601, **ν 3.0** | 58900.0 / 65300.0 | 2.4e-12 / 0.00089 / 0.097 |
| `κ_profile` | large | Student t | μ 0.0156, σ 0.984 | μ 0.0129, b 0.685 | μ 0.0131, s 0.654, **ν 3.4** | 35300.0 / 38300.0 | 1.4e-7 / 0.017 / 0.17 |
| `widths` | small_strong | Student t | μ -0.0321, σ 1.39 | μ -0.0234, b 0.906 | μ -0.0391, s 0.996, **ν 6.0** | 62500.0 / 64900.0 | 0.0037 / 0.18 / 0.12 |
| `widths` | small_weak | Laplace | μ -0.0238, σ 1.44 | μ 0.0136, b 0.968 | μ -0.0092, s 0.958, **ν 3.6** | 48600.0 / 46800.0 | 6.4e-8 / 0.01 / 0.12 |
| `widths` | large | Student t | μ 0.0573, σ 1.26 | μ 0.0434, b 0.902 | μ 0.0436, s 0.994, **ν 6.0** | 24300.0 / 25500.0 | 0.00094 / 0.2 / 0.14 |
| `sight` | small_strong | Student t | μ -0.127, σ 1.02 | μ -0.177, b 0.747 | μ -0.17, s 0.781, **ν 5.0** | 1950.0 / 2890.0 | 5.6e-7 / 0.034 / 0.061 |
| `sight` | small_weak | Student t | μ -0.0809, σ 0.998 | μ -0.154, b 0.621 | μ -0.163, s 0.548, **ν 2.8** | 9640.0 / 10800.0 | 7.1e-11 / 0.00097 / 0.13 |
| `sight` | large | Student t | μ -0.131, σ 1.24 | μ -0.162, b 0.915 | μ -0.155, s 0.986, **ν 5.8** | 1580.0 / 2530.0 | 4.6e-7 / 0.035 / 0.038 |
| `clearance` | small_strong | Student t | μ 0.13, σ 1.16 | μ 0.125, b 0.881 | μ 0.125, s 0.99, **ν 7.4** | 1010.0 / 8510.0 | 0.0011 / 0.31 / 0.11 |
| `clearance` | small_weak | Student t | μ 0.157, σ 1.18 | μ 0.153, b 0.894 | μ 0.147, s 1.0, **ν 7.1** | 562.0 / 7060.0 | 0.0022 / 0.38 / 0.15 |
| `clearance` | large | Student t | μ 0.378, σ 1.22 | μ 0.355, b 0.944 | μ 0.369, s 1.1, **ν 11.0** | -7720.0 / 3740.0 | 0.0059 / 0.56 / 0.094 |
