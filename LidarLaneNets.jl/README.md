# LidarLaneNets.jl

Lane estimation from a 2D LiDAR scan with neural networks, one per estimate
and tier:

- a **small** network for each estimate, run on the car (the edge) at every
  scan;
- a **large** one, run in the cloud when a schedule calls it: more accurate,
  but its result comes back later, and each call costs.

A Kalman filter combines the results over time, counting each at the time of
its scan, so a late cloud result is put where it belongs, and learning from
the cloud's results the car network's bias, which it takes off the car's. The networks are
trained on the simulator's ground truth, not on a hand-written estimator,
so they can beat one.

[LidarRacer.jl](../LidarRacer.jl) races with them. The design is in
[`dnn-estimators/README.md`](../../dnn-estimators/README.md) (outside this
repository).

## The estimates

Each from one scan, relative to the lane center, at the front axle, in the
car's frame (x forward, y left, angles counterclockwise):

| Estimate | Physical meaning | Unit | Size |
|---|---|---|---|
| `e` | cross-track error: sideways distance from the lane center to the car (`> 0`: center to the left) | m | 1 |
| `ψe` | heading error: the lane's direction minus the car's heading | rad | 1 |
| `κ_profile` | lane-center curvature every 0.25 m from 0 to 8 m ahead (`PROFILE_S`); the first is κ beside the car | 1/m | 33 |
| `widths` | free room from the lane center to the left wall, then the right, at `PROFILE_S` | m | 66 |
| `sight` | how far ahead along the lane it is seen (at most 10 m) | m | 1 |
| `clearance` | free distance ahead along each of 13 arcs of curvature −1.2 to 1.2 1/m (`CLEARANCE_K`), at most 10 m | m | 13 |

## The networks

1D convolutional networks over the scan's ranges (clipped to 10 m, divided
by 10), each giving a mean and a standard deviation for every number of its
estimate.

Three sets are trained (`NETWORKS`), all in [`weights/`](weights) as
`<set>_<estimate>.jld2` (Git LFS); the car runs `small_weak` and the cloud
`large` by default:

| Set | Runs on | Input | Layers (stride-2 convolutions, kernel 5) | Each output sees | MACs per scan (each network) | Parameters (each) | Inference, one scan (CPU) |
|---|---|---|---|---|---|---|---|
| `small_weak` | the car (edge) | every 8th beam (136) | 3 convolutions (4, 8, 8 channels), dense 32 | ~58° of the scan | 17–21 k | 5.0–9.3 k | 0.03 ms |
| `small_strong` | (for comparison) | every 4th beam (271) | 3 convolutions (8, 16, 16), dense 64 | ~29° | 127–136 k | 37–45 k | 0.04 ms |
| `large` | the cloud | every beam (1081) | a `small_strong` network, plus a correction: 6 convolutions (16, 32, 32, 64, 64, 128), dense 256, 128 | ~63° (the correction) | 4.24–4.26 M | 707–732 k | 0.27 ms |

A large network is a `small_strong` network plus a correction computed from
every beam. The correction's last layer starts at zero, so training starts
from exactly the small network, and the epoch kept is the best on the
validation tracks, counting that starting point: **a large network is never
less accurate than its small one there** (so neither than `small_weak`,
which is less accurate still).

## Use

```julia
using LidarLaneNets

nets = load_nets("weights")      # (small=(e=…, …), large=(…)): small_weak on the car, large in the cloud
est = TieredEstimator(nets; schedule=RoundRobin())
noise = load_noise("weights")                     # measured for small_weak and large
tracker = LaneTracker(0.257; cfg=TrackerConfig(; noise))   # front axle 0.257 m ahead of the rear axle

# every scan, at time t [s], with the car's speed and yaw rate:
meas = observe!(est, ranges, t; lane, v, yaw_rate)   # small results now, cloud results as they arrive
lane = track!(tracker, meas, v, yaw_rate, t)         # a LaneState: e, ψe, κ, profiles, sight, clearance, status
```

### When to call the cloud

The small networks always run. The only decision is which estimates, if
any, to also ask the cloud for at each scan. A `CloudSchedule` makes it: any
type with a method `cloud_calls(schedule, ctx)` returning estimate names,
where `ctx = (; t, lane, v, yaw_rate)`. Given:

| Schedule | Calls |
|---|---|
| `NoCalls()` | never: the small networks alone |
| `AllCalls()` | every estimate, every scan |
| `FixedCalls(:κ_profile, :sight)` | the same estimates every scan |
| `RoundRobin(; per_scan=1)` (default) | `per_scan` estimates per scan, in turn |

An online schedule (say, from a trajectory sensitivity analysis) is one more
such type. `est.calls` counts the calls per estimate (the cost);
`set_schedule!` changes the schedule.

`SelectorConfig` sets the simulated network (`remote_latency`,
`remote_drop`: the chance a scan's cloud request is lost, with all its
results) and which results are used: a result is dropped if it is unsure
(mean σ above `max_σ`) or late (older than `max_age`). A cloud result that
disagrees with the car's is kept: the filter learns the car's bias from the
difference. With `use_small=false` the small results are never used: a car
on the cloud alone.

### Combining the car's and the cloud's results

Both tiers measure the same quantities, but their errors differ in kind.
The car network's error is mostly the **same from one scan to the next**
(the scene barely changes in 25 ms): on the validation drives, 55–100% of
its variance persists, for 70–270 ms. So the tracker models

    car:   z = x + b + v      b: the car network's bias (Gauss-Markov), v: white
    cloud: z = x + w          w: white

and a cloud result, compared with the car's for the same scan, reveals the
car network's current bias, which the tracker takes off the car's following
results until it has drifted.

- **e, ψe, κ**: an unscented Kalman filter over them and the car's biases on
  them; the motion between scans from the speed and yaw rate.
- **κ_profile, the widths, sight, clearance** (no good motion model: a corner
  changes them within a few scans): per number, a Kalman filter on the car's
  bias alone, fed by every scan both tiers measured (z_car − z_cloud); the
  estimate is the car's latest result less that bias, or, without one, the
  cloud's latest, aged by the distance driven since its scan (the profiles,
  which stay put on the ground, also shifted).
- **Long tails**: an update is a robust (Student t) one, with the tail
  weights measured for each network set, except where both tiers measured
  the same number at the same scan: there a difference larger than expected
  is evidence that the car's bias is larger than usual (its tails are long
  too), so the bias's variance is raised to match it (adaptive covariance
  inflation) and the bias can jump to what the cloud shows. Without this, a
  rare large bias (0.3 rad of heading error, deep in a YasMarina hairpin,
  where the usual size is 0.02) read as a wild cloud result, and the car
  crashed there on every run. It still does when the cloud's results come
  too rarely or too late (see LidarRacer.jl's results).
- **Late results** count at their own scan: the tracker keeps the last 10
  scans and filters again from the late one's (the exact solution of the
  out-of-sequence measurement problem by reprocessing; Bar-Shalom's
  algorithms, 2002 and with Chen and Mallick 2004, reach it without storing
  the history, which only pays for large states).

The noise parameters (how much and how long the car's error persists, its
size, the tail weights) are measured on the validation tracks by
[`train/fusion.jl`](train/fusion.jl) into [`weights/fusion.toml`](weights/fusion.toml)
and loaded with `load_noise`.

On 2400 recorded driving scans (Spa), RMS error of the tracked estimates:

| | e [m] | ψe [rad] | sight [m] | clearance ahead [m] |
|---|---|---|---|---|
| car only (`small_weak`) | 0.0076 | 0.0111 | 0.155 | 0.274 |
| cloud only, 25 / 50 / 100 ms | 0.0075 / 0.0098 / 0.0165 | 0.0095 / 0.0104 / 0.0144 | 0.146 / 0.238 / 0.413 | 0.241 / 0.434 / 0.769 |
| car + cloud, 25 / 50 / 100 ms | 0.0074 / 0.0074 / 0.0076 | 0.0102 / 0.0103 / 0.0107 | 0.137 / 0.143 / 0.152 | 0.177 / 0.227 / 0.268 |

Combined, the estimates are at least as good as the car's alone at every
latency, and worsen steadily as the cloud's results come later; sight and clearance
are better than either tier's alone at every latency. (An earlier version
kept the newest result, which was always the car's, and so used the cloud's
sight and clearance not at all.)

## Cost and accuracy

Every network against the hand-written estimator (LidarLaneEstimation.jl), on
the five tracks never seen in training (24 000 scans while driving), is in
[`train/results/networks.md`](train/results/networks.md), with links to each
network's weights (from [`train/evaluate.jl`](train/evaluate.jl)), and
drawn in [`train/results/accuracy_vs_macs.pdf`](train/results/accuracy_vs_macs.pdf).
In short:

| Network set | MACs per scan, all six | Parameters, all six |
|---|---|---|
| `small_weak` (the car) | 0.11 M | 37 k |
| `small_strong` | 0.78 M | 236 k |
| `large` (the cloud) | 25.5 M | 4.29 M |

| Estimate | Unit | Hand-written | `small_weak` | `small_strong` | `large` | Validation R²: `small_weak` / `small_strong` / `large` |
|---|---|---|---|---|---|---|
| `e` | m | 0.051 | 0.0097 | 0.0081 | 0.0079 | 0.9982 / 0.9986 / 0.9986 |
| `ψe` | rad | 0.087 | 0.031 | 0.025 | 0.025 | 0.927 / 0.932 / 0.933 |
| `κ_profile` | 1/m | **0.057** (gives 95% of the entries) | 0.092 | 0.080 | 0.071 | 0.691 / 0.785 / 0.812 |
| `widths` | m | 0.101 (95%) | 0.099 | 0.098 | 0.099 | 0.702 / 0.711 / 0.711 |
| `sight` | m | 1.73 | 0.22 | 0.19 | 0.17 | 0.986 / 0.991 / 0.992 |
| `clearance` | m | 0.86 | 0.37 | 0.28 | 0.22 | 0.718 / 0.844 / 0.900 |

(RMS errors on the held-out tracks; profiles over the entries the car can
see.)

- The cloud's ~240× more MACs (over the car's) buy the most on clearance
  (−41%), sight (−24%), κ_profile (−22%), ψe and e (−20%); the widths are
  about what any of them gets from the scan.
- The networks beat the hand-written estimator except on the curvature
  ahead, which it smooths over 2 m (the labels use 1 m) and leaves out where
  it is hardest. Part of its e error is a different idea of the center: the
  track files' centerlines sit about 2 cm (RMS) from the middle between the
  walls, which the networks learn and it doesn't.
- The standard deviations are calibrated: 60–83% of errors are within one
  (68% if exactly Gaussian).
- The car's and the cloud's errors for the same scan are correlated (0.61
  to 0.99): they share a part that neither can remove, which cancels in the
  difference z_car − z_cloud the tracker learns the car's bias from.

## Error distributions

The errors are long-tailed, and one distribution fits them all: a **Student
t** had the least AIC (against a Gaussian and a Laplace) for every network's
errors, and for 17 of 18 of the errors divided by the network's own σ (a
Laplace, barely, for `small_weak`'s widths). Its location μ, scale s and
degrees of freedom ν for every network are in
[`train/results/errors.md`](train/results/errors.md) (from
[`train/errors.jl`](train/errors.jl)), and drawn in
[`train/results/error_distributions.pdf`](train/results/error_distributions.pdf).
ν, smaller for heavier tails (a Gaussian as ν → ∞, a Cauchy at 1):

| Estimate | Errors: ν (`small_weak` / `small_strong` / `large`) | Errors over the network's own σ: ν |
|---|---|---|
| `e` | 3.9 / 3.9 / 3.9 | 9.4 / 7.1 / 7.1 |
| `ψe` | 2.1 / 2.1 / 2.0 | 6.2 / 6.0 / 5.4 |
| `κ_profile` | 0.82 / 0.77 / 0.79 (pooled over the distance ahead) | 3.0 / 3.4 / 3.4 |
| `widths` | 5.0 / 4.8 / 4.6 | 3.6 / 6.2 / 6.0 |
| `sight` | 0.88 / 1.3 / 1.1 | 2.8 / 5.0 / 5.8 |
| `clearance` | 1.2 / 1.6 / 1.8 | 7.1 / 7.4 / 11 |

Divided by the network's own standard deviation, the errors are much closer
to Gaussian (scale about 1): the networks' σ accounts for most of the tails.
The clearance errors are biased toward too much free distance, by 0.13
(small) to 0.37 (large) of their σ.

## Training

Everything in [`train/`](train), its own environment:

```sh
cd train
julia --project=. -e 'import Pkg; Pkg.instantiate()'
julia -t auto --project=. collect.jl      # data/: labeled scans from every racetrack (~15 s)
julia -t auto --project=. train.jl        # weights/: all 18 networks (small_weak, small_strong, large)
julia -t auto --project=. evaluate.jl     # results/networks.md (and .toml): cost and accuracy, on the held-out tracks
julia -t auto --project=. errors.jl       # results/errors.md (and .toml): the error distributions
julia -t auto --project=. fusion.jl       # weights/fusion.toml: the noise the tracker combines the tiers with
julia --project=. plots.jl                # results/*.pdf: accuracy against MACs, the error distributions (from the .toml)
```

- **Shared**: [`tracks.jl`](train/tracks.jl) (the track split, centerlines,
  distances to the walls; LidarRacer.jl's race scripts use it too),
  [`common.jl`](train/common.jl) (paths, loading the data, running a network
  over it) and [`tdist.jl`](train/tdist.jl) (fitting a Student t).
- **Labels** ([`truth.jl`](train/truth.jl)): the true value of every
  estimate, from the map and the track's centerline, at the LiDAR's pose at
  the middle of its sweep.
- **Data** ([`collect.jl`](train/collect.jl)): deterministic, so it is
  regenerated (768 MB, about 15 s) rather than committed: each track has a
  fixed seed, and the same code gives bit-identical files. Per track, 2 drives of 60 s
  on the true estimates with slowly wandering errors added and a random
  speed limit (so the car also drives off the ideal line; the scans carry
  the real motion distortion), plus 2500 scans standing at random places
  and headings. 7300 scans per track.
- **Split**: 16 tracks to train on, 2 to choose the epoch (Hockenheim,
  Sepang), 5 never seen (Spa, Monza, Silverstone, Oschersleben, YasMarina).
- **Loss** ([`train.jl`](train/train.jl)): the β-negative log-likelihood of
  a Gaussian (β = 0.5), on targets normalized per output; Adam,
  learning rate 1e-3 with cosine decay, 40 epochs; the epoch with the least
  validation error is kept. Half of each batch is mirrored left to right
  (the LiDAR is symmetric), which doubles the data.
- **Large networks** start from the trained `small_strong` ones (so
  `train.jl` trains the small sets first), with a learning rate of 3e-4 for
  20 epochs. `small_weak` is paired with the same large networks: the cloud
  needs only the scan, so making the car's networks smaller doesn't call for
  new cloud ones (large networks built on `small_weak` came out less
  accurate than these).
- **What didn't work** for the large networks, trained on their own: four
  convolutions into a 4352-wide dense layer (each output saw only ~12° of
  the scan), six convolutions seeing ~63° (better on κ_profile, worse on e),
  dropout (about 6× worse on the training data itself), batch normalization
  (2.4× slower per epoch on a CPU). A larger network that is less accurate
  than the small one is no use as a tier, hence the residual design.
- **GPU**: if `LuxCUDA` (NVIDIA), `AMDGPU` or `Metal` is added to the train
  environment, `train.jl` loads it and trains there; otherwise on the CPU
  (here, a 6-core laptop: about 1 h for all 18; one set:
  `train.jl small_weak`).

The weights in [`weights/`](weights) are stored with Git LFS.

## Tests

```sh
julia --project=. -e 'import Pkg; Pkg.test()'
```
