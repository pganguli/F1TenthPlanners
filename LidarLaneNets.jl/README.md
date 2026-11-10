# LidarLaneNets.jl

Lane estimation from a 2D LiDAR scan with neural networks, one per estimate
and tier:

- a **small** network for each estimate, run on the car at every scan;
- a **large** one, run "in the cloud" when a schedule calls it: more
  accurate, but its result comes back later, and each call costs.

A Kalman filter combines the results over time, counting each at the time of
its scan, so a late cloud result is put where it belongs. The networks are
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

A large network is its small one plus a correction computed from every
beam. The correction's last layer starts at zero, so training starts from
exactly the small network, and the epoch kept is the best on the
validation tracks, counting that starting point: **a large network is never
less accurate than its small one there**, and the cloud refines what the
car already computes.

| Tier | Input | Layers (stride-2 convolutions, kernel 5) | Each output sees | Parameters | Inference, one scan (CPU) |
|---|---|---|---|---|---|
| small | every 4th beam (271) | 3 convolutions (8, 16, 16 channels), dense 64 | ~29° of the scan | 37–45 k | ~0.12 ms |
| large | every beam (1081) | the small network, plus a correction: 6 convolutions (16, 32, 32, 64, 64, 128), dense 256, 128 | ~63° (the correction) | 707–732 k | ~0.55 ms |

## Use

```julia
using LidarLaneNets

nets = load_nets("weights")                       # (small=(e=…, …), large=(…))
est = TieredEstimator(nets; schedule=RoundRobin())
tracker = LaneTracker(0.257)                      # front axle 0.257 m ahead of the rear axle

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
`remote_drop`) and which results are used: a result is dropped if it is
unsure (mean σ above `max_σ`), late (older than `max_age`), or, from the
cloud, too far from the small result for the same scan (`agree` σ). With
`use_small=false` the small results are never used: a car on the cloud
alone.

### Merging a late cloud result

The small result for scan k is used at once. When the cloud's result for
scan k arrives, it **replaces** the small one for that scan, and the filter
is run again from scan k to now (it keeps the last 10 scans' priors and
measurements). This is the exact solution of the out-of-sequence
measurement problem by reprocessing; Bar-Shalom's algorithms (2002; with
Chen and Mallick, 2004) reach it without storing the history, which only
pays for large states. The profiles are replayed the same way.

Replacing rather than fusing the two: both come from the same scan, so
their errors are correlated, and fusing them as independent would make the
filter overconfident. See the measured correlations below.

## Accuracy

Per scan, on the five tracks never seen in training (24 000 scans while
driving), RMS error against the true values; profiles over the entries the
car can see. From [`train/evaluate.jl`](train/evaluate.jl); the full table
(with calibration) is in [`train/results/accuracy.md`](train/results/accuracy.md).

| Estimate | Unit | Hand-written (LidarLaneEstimation.jl) | Small DNN | Large DNN |
|---|---|---|---|---|
| `e` | m | 0.051 | 0.0081 | 0.0079 |
| `ψe` | rad | 0.087 | 0.0253 | 0.0250 |
| `κ_profile` | 1/m | **0.057** (gives 95% of the entries) | 0.080 | 0.071 |
| `widths` | m | 0.101 (95%) | 0.098 | 0.099 |
| `sight` | m | 1.73 | 0.186 | 0.169 |
| `clearance` | m | 0.86 | 0.28 | 0.22 |

- The large networks are at least as accurate as the small ones (by
  construction on the validation tracks; here too), most of all where it is
  hard to see: clearance (−23%), κ_profile (−11%), sight (−9%). On e, ψe and
  the widths they add little.
- The networks beat the hand-written estimator except on the curvature
  ahead, which it smooths over 2 m (the labels use 1 m) and leaves out where
  it is hardest. Part of its e error is a different idea of the center: the
  track files' centerlines sit about 2 cm (RMS) from the middle between the
  walls, which the networks learn and it doesn't.
- The standard deviations are calibrated: 60–83% of errors are within one
  (68% if exactly Gaussian).
- The small and large networks' errors are strongly correlated (0.84 for
  clearance to 1.00 for the widths), which is why a cloud result replaces
  the small one for its scan rather than being fused with it.

## Training

Everything in [`train/`](train), its own environment:

```sh
cd train
julia --project=. -e 'import Pkg; Pkg.instantiate()'
julia -t auto --project=. collect.jl      # data/: labeled scans from every racetrack (~15 s)
julia -t auto --project=. train.jl        # weights/: all 12 networks
julia -t auto --project=. evaluate.jl     # results/accuracy.md: on the held-out tracks
```

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
- **Large networks** start from the trained small ones (so `train.jl`
  trains the small tier first), with a learning rate of 3e-4 for 20 epochs.
- **What didn't work** for the large networks, trained on their own: four
  convolutions into a 4352-wide dense layer (each output saw only ~12° of
  the scan), six convolutions seeing ~63° (better on κ_profile, worse on e),
  dropout (about 6× worse on the training data itself), batch normalization
  (2.4× slower per epoch on a CPU). A larger network that is less accurate
  than the small one is no use as a tier, hence the residual design.
- **GPU**: if `LuxCUDA` (NVIDIA), `AMDGPU` or `Metal` is added to the train
  environment, `train.jl` loads it and trains there; otherwise on the CPU
  (here, a 6-core laptop: about 1 h for all 12).

The weights in [`weights/`](weights) are stored with Git LFS.

## Tests

```sh
julia --project=. -e 'import Pkg; Pkg.test()'
```
