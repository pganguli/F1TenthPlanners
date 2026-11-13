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

## The two tests

| Test | The estimate changed | The other five |
|---|---|---|
| **Accuracy** | its true value plus a network's errors times a gain (gain 1 of `small_weak`: the car's network; of `large`: the cloud's), from the car at every scan, without delay | true, from the car at every scan |
| **Latency** | its true value, but only from the cloud, a delay after its scan: as LidarLaneNets.jl's selector and filter handle a late cloud result (counted at its scan's time; dropped past `max_age`, 100 ms) | true, from the car at every scan |

The car's own delays stay in every run, the reference too: the LiDAR's
sweep, the 7.5 ms from a scan to its command, the servo and the VESC speed
loop.

## How the oracle errs

A network gives a value and its own σ for each entry of its estimate (one
for `e`, 33 for `κ_profile`, 66 for the widths, ...).
[`NetworkErrors`](src/errors.jl) draws both, scan after scan:

- **the error**: the network's measured error distribution (pooled over the
  entries, each scaled by its own median error, since a profile's errors
  grow with the distance ahead), **scaled with the situation**: the
  networks err far more in some situations than in others. `κ_profile`'s
  median error is some 70 times larger where the lane bends sharply than
  where it is straight, and `ψe`'s and `clearance`'s 8 times. The situation
  is each entry's true value (for `e` and `ψe`: how much the lane bends
  beside the car), and the scale is measured per bin of it. The Student t fitted in LidarLaneNets.jl's
  [errors.md](../LidarLaneNets.jl/train/results/errors.md) matches its bulk.
  For some estimates it has ν < 1, and its tails then reach far past any
  error the networks make, so it is not used to draw from.
- **correlated** across the entries (a Gaussian copula with the measured
  correlation) and **over time** (fitted so that the errors persist from
  scan to scan as the network's do, the correlation
  [fusion.jl](../LidarLaneNets.jl/train/fusion.jl) measures);
- **the σ the network gives**: log-normal per entry, scaled with the
  situation as measured, and tied to the error's size by the measured
  correlation.

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

# the controller for one experiment, and the oracle its networks ask
p = accuracy(:ψe, 2.0; seed=3)          # or reference(), latency(:sight, 0.05)
racer, oracle = oracle_racer(car, beams, p, models)

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
| [`error_model.jl`](scripts/error_model.jl) | measures how the networks err (runs them on LidarLaneNets.jl's recorded scans: `train/data`, from its `collect.jl`) | `results/error_model.toml`, `.md` |
| [`track.jl`](scripts/track.jl) | builds the representative track, with every kind of situation of the 23 f1tenth_racetracks tracks, and compares their geometry | `track/`, `results/track.md` |
| [`sensitivity.jl`](scripts/sensitivity.jl) | the reference, and both tests for every estimate (`julia -t auto --project=. sensitivity.jl`) | `results/sensitivity.md`, `runs.csv` (`runs.jld2`, `tubes.jld2`: every trajectory, not committed) |
| [`plots.jl`](scripts/plots.jl) | plots them | `results/accuracy.png`, `latency.png`, `along_track.png` |
| [`analytic.jl`](scripts/analytic.jl) | the same questions from the linearized closed loop, without simulation or map (see below) | `results/analytic.md` |
| [`matched.jl`](scripts/matched.jl) | the simulation's own numbers from the linearized closed loop with the Kalman filter, without simulation or map (see below) | `results/matched.md` |
| [`validate.jl`](scripts/validate.jl) | a check outside the analysis: the real networks (car alone, cloud alone) on the same track | `results/validate.md` |

The results are in [scripts/results/sensitivity.md](scripts/results/sensitivity.md).

## Without simulation

[`analytic.jl`](scripts/analytic.jl) asks the same questions without a
simulator or a map. It linearizes the closed loop about a situation: the
car in steady state on a lane of constant curvature and width, with the
lane's center line, walls and what the LiDAR sees computed from the
geometry. One situation in detail (0.1 1/m, 2.26 m), then a grid: curvature
0 to 0.5 1/m, width 1.6, 2.26 and 3.2 m. The pieces:

- the car (F1Tenth.jl's `vehicle_dynamics_st`), the servo and the VESC
  speed loop, sampled every scan with the command delay;
- LidarRacer.jl's `drive`, by its exact Jacobian, fed the estimates
  directly, with no Kalman filter and no age limit on late results;
- in tight bends (from 0.15 1/m), the speed held at the steady state's: the
  speed loop is unstable in small signal there (through the free distance
  ahead, which the speed changes through the yaw rate), and on the car the
  command limiter's ramps hold it to a chatter of a few cm/s.

It then computes the sets the loop can reach (`Zonotope`):

- **accuracy**: the final set for a constant bias anywhere in a box (± the
  car network's median error), as `bias_set` computes it; and the finite
  one, by solving the nonlinear steady state at the box's extremes, which
  sees what a bias of that size does where the controller steps (a profile
  point appearing) or reaches a bound;
- **latency**: the set of the difference a delay of n scans makes, over
  every road whose curvature varies within the racetracks' statistics, as
  `disturbance_set` computes it, plus the delay margin.

What the grid shows: `κ_profile`'s bias moves the car the most everywhere
(0.07 m on a straight, 0.15 to 1 m in bends); `sight`, `widths` and
`clearance` have no effect for small biases, but a median-sized one moves
the car in tight bends (up to 0.1 m for the widths on a wide lane), which
only the finite steady state shows; `ψe`'s delay margin is 100 to 225 ms
everywhere, the shortest of any estimate (125 ms on a straight, at 7 m/s);
at 0.5 1/m the controller holds no steady state.

Results: [scripts/results/analytic.md](scripts/results/analytic.md).

### Matched to the simulation

[`matched.jl`](scripts/matched.jl) answers the simulation's questions in its
terms, still without a simulator or a map: the RMS deviation from the
reference with the car network's errors on one estimate, and its growth per
25 ms of delay. Against analytic.jl, it:

- puts LidarLaneNets.jl's Kalman filter in the loop, as the car runs it: on
  `e`, `ψe` and the curvature beside the car at its steady-state gain
  (`kalman_gain`, the Riccati equation), its motion model's mismatch a
  constant; late results counted at their scan and carried forward by the
  motion model (or, for profiles, shifted, their far end lost);
- replaces worst-case sets with statistics: random roads (each situation's
  lane bending about its curvature as much as the racetracks' do ahead of the
  car there) and the network's recorded errors (their mean, covariance,
  scan-to-scan correlation and dependence on the true value, from the error
  library, at each situation), and the loop's stationary covariance
  (`stationary_covariance`, the Lyapunov equation), against a reference loop
  on the same road;
- takes the measured entries' errors through the planner exactly, not by its
  derivative (statistical linearization: the mean, covariance and
  correlation of the commands they cause, the controller's steps and its
  k_σ σ offsets included);
- adds bends ahead: within 10 m of the car (clearance's and sight's range),
  a straight with a bend ahead or a bend with a straight ahead, the scene held
  as it is about the car, besides lanes of constant curvature; and weights
  every situation by how much of the 23 racetracks is like it;
- in tight bends, where the speed loop is unstable in small signal, passes the
  speed command through the command limiter's ramp's describing function (a
  low-pass, the shortest that makes the loop just stable), so the speed still
  follows its mean command, as on the car.

Against the simulation ([matched.md](scripts/results/matched.md)), the
analytic values come within a factor of about 2 for every estimate,
accuracy and latency (within 30% for latency), and in the same order, as
far as the simulation tells the estimates apart: by accuracy `κ_profile`,
`clearance`, `ψe`, `sight`, then `e` and `widths` (which the simulation
can't tell apart; its `widths` is its error replay's, harsher than the
network, which moves the car under 1 mm, as here); by latency all six
between 2.5 and 4.3 mm per 25 ms (the simulation's between 2.4 and 3.6,
their intervals overlapping, as the analytic values are within a few percent
where the two orders differ).

It needs the error library (the simulation branch's scripts/error_library.jl
writes it; `ERROR_LIBRARY=path`, by default `scripts/results/error_library.jld2`).

## Limits

- The errors depend on the situation only through each entry's own true
  value (or, for `e` and `ψe`, the bend beside the car), not on speed or
  what else the scan shows.
- A Gaussian copula can't make the large errors persist more than the small
  ones, as the networks' do. Fitted to the errors' scan-to-scan correlation,
  it matches it within about 0.1 for most estimates, but `ψe`'s is 0.57
  against 0.77 a scan apart. Neighboring entries of the widths correlate
  less than the network's (0.08 against 0.25, 1 m apart; see
  [error_model.md](scripts/results/error_model.md)).
- Profile entries past the true sight get the true value plus the error
  model. A real network's guesses there may be worse.
- The tube is the band a finite number of runs lie in: a sampled, inner
  estimate of the reachable set, not a bound.
- One estimate at a time: interactions between estimates are not measured.
- On the representative track's narrow chicane, the oracle's `κ_profile`
  errors are pessimistic: they crash the car in most runs with either
  network's errors, but the real cloud networks never crash there
  ([validate.md](scripts/results/validate.md)).
