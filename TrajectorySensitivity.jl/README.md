# TrajectorySensitivity.jl

How much each lane estimate's errors and delays move the car's trajectory.
It runs the whole pipeline (the car and its LiDAR in a simulator,
LidarLaneNets.jl's selector and Kalman filter, LidarRacer.jl's planners and
controller), with the networks replaced by an **oracle**: the true estimates,
plus errors drawn the way the networks err, delivered as the car's or the
cloud's networks deliver them. It changes one estimate at a time and keeps
the others true and on time. It compares each run with the reference run,
where every estimate is true and on time.

The package does not run the networks: no scan goes through a DNN. It takes
from the networks only how they err, measured once
([scripts/error_model.jl](scripts/error_model.jl)). Like LidarRacer.jl, it
doesn't depend on a simulator: the simulator only comes in through the
scripts in [scripts/](scripts/).

## The experiments

| Experiment | The estimate changed | The other five |
|---|---|---|
| **Accuracy** (`accuracy`) | its true value plus a network's errors times a gain (of `small_weak`: the car's network; of `large`: the cloud's), from the car at every scan, without delay | true, from the car at every scan |
| **Latency** (`latency`) | its true value, but only from the cloud, a delay after its scan: as LidarLaneNets.jl's selector and filter handle a late cloud result (counted at its scan's time; dropped past `max_age`, 100 ms) | true, from the car at every scan |
| **Cloud call** (`cloud_call`) | the car's network's errors at every scan, and the cloud's network's errors (from the same recorded scans) a delay after its scan: what calling the cloud for this estimate buys | true, from the car at every scan |
| **Real network** (`real_network`) | the real network (no oracle), on the car at every scan: the check the oracle is calibrated against | true, from the car at every scan |

The car's own delays stay in every run, the reference too: the LiDAR's
sweep, the 7.5 ms from a scan to its command, the servo and the VESC speed
loop.

Every run is summed up per **kind of situation** (`SITUATIONS`,
`situation_class`): straight, or a gentle, medium or tight bend, on its
entry, in its middle or on its exit, from how much the lane bends beside the
car and 1 to 3 m ahead.

## How the oracle errs

A network gives a value and its own σ for each entry of its estimate (one
for `e`, 33 for `κ_profile`, 66 for the widths, ...). The oracle draws both,
scan after scan, from one of two sources.

**Replay** (the default, [`ReplayDraws`](src/replay.jl)): the networks'
own errors and σ, recorded scan by scan by
[scripts/error_library.jl](scripts/error_library.jl). The racer drives the
seven tracks the networks never trained on (the validation and held-out
tracks) on the true estimates while all twelve networks (the car's and the
cloud's, for each estimate) run on every scan; their errors are stored with
each scan's true lane (`situation_features`: `e`, `ψe`, the curvature 0 to
5 m ahead, the room to each side, the sight).

A network's error belongs to the scene it looks at: `widths`' errors are
three quarters a function of the true width (the network pulls it toward a
typical width), `clearance`'s and `κ_profile`'s half, and `ψe`'s depend on
which way the lane bends. So the oracle gives each scan the errors of a
recorded scan whose true lane is like the car's (each feature scaled by its
spread), mirrored when the recorded lane bends the other way (`e`, `ψe` and
`κ_profile` change sign, the sides' widths swap, the clearance fan reverses).
It goes on through that recorded drive, scan after scan, while the recorded
lane stays like the car's (for at most 40 scans, 1 s), then starts again at
one of the 8 nearest recorded scans. Everything else about the errors stays
as it was: their distribution, how they persist from scan to scan, how they
correlate across a profile's entries, how σ follows them, and how the car's
and the cloud's errors on the same scan relate (a cloud call takes both
from the same recorded scan). A run never replays errors recorded on its own
track (`without`).

The errors are recorded along the racer's own path, not on LidarLaneNets.jl's
training drives, which wander off the line on purpose and where some networks
err twice as much. Replaying them by kind of situation only (the bend's size
and phase, at random within it) made `ψe`'s errors twice as harmful as the
real network's and `widths`' twenty times: they were tied to the wrong
scenes.

**Parametric** ([`NetworkErrors`](src/errors.jl), `source=:parametric`):
a model of the same errors, measured by
[scripts/error_model.jl](scripts/error_model.jl) on the training data:

- **the error**: the network's measured error distribution (pooled over the
  entries, each scaled by its own median error), **scaled with the
  situation** (each entry's true value; for `e` and `ψe`, how much the lane
  bends beside the car);
- **correlated** across the entries (a Gaussian copula with the measured
  correlation) and **over time** (fitted to the errors' scan-to-scan
  correlation);
- **the σ the network gives**: log-normal per entry, scaled with the
  situation, and tied to the error's size by the measured correlation.

[scripts/results/error_model.md](scripts/results/error_model.md) checks the
draws against the real errors.

A gain k multiplies both the error and σ: a network k times as wrong that
knows it. The Kalman filter's noise parameters for that estimate (the
persisting error's σ_bias) are scaled with it, as
[fusion.jl](../LidarLaneNets.jl/train/fusion.jl) would measure them for such
a network. A true estimate comes with 1% of its network's typical σ: a
network that is right and knows it.

## Use

```julia
using LidarLaneNets, LidarRacer, TrajectorySensitivity

models = Dict(n => load_errors("scripts/results/error_model.toml"; network=n) for n in (:small_weak, :large))
library = load_library("scripts/results/error_library.jld2")   # from scripts/error_library.jl

# the controller for one experiment, and the oracle its networks ask
p = accuracy(:ψe, 2.0; seed=3)          # or reference(), latency(:sight, 0.05), cloud_call(:κ_profile, 0.05)
racer, oracle = oracle_racer(car, beams, p, models; library)

# on each new scan: the true estimates first (a LaneLabels), then the controller as usual
set_truth!(oracle, labels, scan_time)
steer, speed, lane = control!(racer, ranges, v, yaw_rate, dt)

# the trajectory, along the track's centerline
lg = TrajectoryLog(center)               # a closed StanleyController Path
log!(lg, t, x, y, θ, v, gap)
c = compare(lg, reference_log)           # Δd, Δψ, Δv, Δt along the track; max_Δd, Δlap, crashed, ...
tb = tube(logs; distance)                # the band all runs lie in
```

`oracle_racer` takes the `Racer`'s other keywords (`selector`, `tracking`,
`stanley`, `margin`, `k_σ`, `latency`, `v_max`), except what the
experiment sets: the filter's noise parameters, the cloud schedule and the
cloud's delay.

### Tracks

`TrackDesign` builds a closed track from pieces of known curvature and
width: `straight`, `corner` and `bends` (arcs entered and left by
clothoids), `spiral` (a tightening or opening corner). One arc and two
straights are left free, and `build_track` finds their lengths so that the
loop closes. `track_problems` checks for walls that fold over or parts of
the track that come too close. `write_track` writes the map as the
f1tenth_racetracks maps are (thin walls on free ground, a yaml, the
centerline), for F1Tenth.jl's `F110Env(map=…, map_ext=".pgm")`.

## Scripts

From [scripts/](scripts/), set up once with
`julia --project=. -e 'using Pkg; Pkg.instantiate()'`:

| Script | Does | Writes |
|---|---|---|
| [`error_model.jl`](scripts/error_model.jl) | measures how the networks err, for the parametric model (runs them on LidarLaneNets.jl's recorded scans: `train/data`, from its `collect.jl`) | `results/error_model.toml`, `.md` |
| [`error_library.jl`](scripts/error_library.jl) | records the networks' errors for replay, along the racer's path on the seven tracks the networks never trained on (`julia -t 4 --project=. error_library.jl`) | `results/error_library.jld2` (not committed) |
| [`track.jl`](scripts/track.jl) | builds the representative track, with every kind of situation of the 23 f1tenth_racetracks tracks, and compares their geometry | `track/`, `results/track.md` |
| [`calibrate.jl`](scripts/calibrate.jl) | the oracle (replay and parametric) against the real networks, one estimate at a time, on the seven tracks the networks never trained on (each replaying only the others' errors) and the representative track | `results/calibration.csv` |
| [`sensitivity.jl`](scripts/sensitivity.jl) | the reference, accuracy and latency for every estimate, on the 23 racetracks and the representative track, 10 seeds (`julia -t auto --project=. sensitivity.jl`; `--seeds`, `--tracks`) | `results/sensitivity.csv` |
| [`cloud.jl`](scripts/cloud.jl) | the car's network alone against a cloud call at 25, 50 and 100 ms, for every estimate | `results/cloud.csv` |
| [`interactions.jl`](scripts/interactions.jl) | every estimate errs at once, each with its own gain (0 to 2); variance-based (Sobol) indices: what each causes alone and with the others (`julia -t auto --project=. interactions.jl`: the representative track, N = 512; `--n 64 --tracks all`: every track) | `results/interactions.csv`, `interactions_all.csv` |
| [`report.jl`](scripts/report.jl) | sums the campaigns up with 95% confidence intervals, per kind of situation | `results/calibration.md`, `sensitivity.md`, `cloud.md`, `cloud_benefit.png`, `interactions.md` |
| [`validate.jl`](scripts/validate.jl) | the real networks (car alone, cloud alone) on the representative track | `results/validate.md` |
| [`runs.jl`](scripts/runs.jl) | what the campaigns share: driving a course, the safety measures, the CSV rows | |

The campaigns' CSV files (every run, tens of MB) are not committed: the
scripts regenerate them, and the reports summing them up are in
[scripts/results/](scripts/results/).

Each run is measured against its track's reference run: crashes per km, the
RMS lateral deviation from the reference, the least gap to a wall, the share
of the time within 0.2 m of a wall, the scans with time to collision under
0.5 s, and the tube (the band a group of seeds lies in).

## Results

In [scripts/results/](scripts/results/): [sensitivity.md](scripts/results/sensitivity.md),
[cloud.md](scripts/results/cloud.md), [interactions.md](scripts/results/interactions.md),
[calibration.md](scripts/results/calibration.md).
On 22 racetracks and the representative track (Shanghai is left out: the car crashes
there even with every estimate true), 10 seeds:

| Estimate | Accuracy: RMS deviation with the car's network [m] | per unit of gain [m] | Latency: per 25 ms late [m] | Cloud call at 50 ms: change in least gap [m] |
|---|---|---|---|---|
| `κ_profile` | 0.050 (0.13 crashes/km) | 0.049 | 0.0032 | +0.081 (crashes 0.13/km → 0) |
| `clearance` | 0.016 | 0.015 | 0.0036 | −0.004 (none) |
| `ψe` | 0.012 | 0.011 | 0.0032 | 0.000 (none) |
| `sight` | 0.0079 | 0.0086 | 0.0027 | −0.002 (none) |
| `widths` | 0.0052 | 0.011 (grows faster than the gain) | 0.0025 | 0.000 (none) |
| `e` | 0.0042 | 0.0038 | 0.0024 | −0.001 (none) |

- **Accuracy matters most for `κ_profile`**, 3 to 12 times more than for the others,
  and it is the only estimate whose car network crashes the car (mostly in tight bends:
  19 crashes/km in their middle). Its crashes grow fast with the error: 1.1/km at twice
  the car network's errors, 5.5/km at four times.
- **Latency costs about the same for every estimate, 2.4 to 3.6 mm per scan late**, up
  to the selector's 100 ms. Past that the estimate never comes, and the car can't drive
  without `e` or `ψe`.
- **Errors interact in crashes, not in the deviation**
  ([interactions.md](scripts/results/interactions.md): all six err at once, Sobol
  indices). The deviation is `κ_profile`'s alone (its first-order index about 1, the
  others' totals under 0.1). But crashes come from combinations: `clearance`'s errors
  alone never crash the car (not even 16 times the car network's), yet with `κ_profile`'s
  they do: on the representative track, with `κ_profile` at 1 to 1.5 times the car
  network's errors, 5% of runs crash when `clearance`'s are at most the car network's,
  65% when they are larger (`clearance` sets the speed, through the free distance ahead).
- **Calling the cloud pays only for `κ_profile`**: it takes the car from 0.13
  crashes/km to none and 8 cm farther from the walls, at any delay up to 100 ms (most on
  straights and at bend exits). For the other five it changes nothing measurable.

How far to trust them, from [calibration.md](scripts/results/calibration.md) (the oracle
against the real networks in the loop, on the seven tracks they never trained on and the
representative track): the replayed errors move the car as much as the real networks do
within about 30% for `e`, `ψe`, `κ_profile`, `sight` and `clearance` (`e` and
`κ_profile` some 20% less, `sight` 30% more). `widths`' replayed errors move it about
5 mm, the real network's under 1 mm: both are negligible.

## Limits

- The real `κ_profile` network crashes the car in three particular tight bends (on
  Monza, Yas Marina and the representative track), every seed. Replayed errors come from
  other tracks' scenes (never the run's own), and they don't make those crashes: its
  crash rates here are lower bounds (on the calibration tracks, the real network: 1.1/km;
  the replay: 0.06).
- The errors are replayed by the true lane only: the matching doesn't see the car's
  speed or anything else in the scan (other walls, openings).
- They were recorded along the racer's own path on true estimates. Where an erring
  estimate takes the car far from that path, the scenes differ from any recorded one, and
  the nearest are replayed.
- The tube is the band a finite number of runs lie in: a sampled, inner estimate of the
  reachable set, not a bound.
- The interactions are measured for accuracy only: the selector has one cloud delay for
  every estimate, so joint latencies are not.
- The parametric error model (`source=:parametric`) is two to four times as harsh as the
  real networks for `κ_profile` and `sight`
  ([calibration.md](scripts/results/calibration.md)). It is kept for experiments on a
  model, not used by the campaigns.
