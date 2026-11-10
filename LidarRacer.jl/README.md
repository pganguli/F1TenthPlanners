# LidarRacer.jl

Races an F1TENTH car around a track using only its LiDAR. It has no map and
no pose. The walls are the lane boundaries. On every scan, the controller
plans the line through the lane that bends least, steers along it, and goes
as fast as it can still slow down in time for what it sees. The line is a
local raceline, re-planned on every scan as far ahead as the LiDAR sees. It
is not a global raceline computed ahead of time from a map.

On this branch (`dnn-estimators`) the lane is estimated by neural networks
instead of the hand-written LidarLaneEstimation.jl: a small network per
estimate on the car at every scan, and a large one "in the cloud" when a
schedule calls it. The controller combines the work of four sibling
packages:

| Step | Package |
|---|---|
| estimates the lane relative to its center (cross-track and heading error at the front axle, the curvature and free room on each side ahead, how far ahead it is seen, the free distance along a fan of arcs), and combines the estimates over time with a Kalman filter | [LidarLaneNets.jl](../LidarLaneNets.jl) |
| plans the line that bends least through the lane rebuilt from those estimates | [MinimumCurvaturePlanner.jl](../MinimumCurvaturePlanner.jl) |
| measures the car against that line and turns the errors into a steering command | [StanleyController.jl](../StanleyController.jl) |
| sets the speed | [VelocityProfilePlanner.jl](../VelocityProfilePlanner.jl) |

It doesn't depend on a simulator. The simulator only comes in through the
scripts in [scripts/](scripts/).

## Use

```julia
using LidarLaneNets, LidarRacer

# the car and its LiDAR; CarParams(p) also reads these from any object that
# has them, such as F1Tenth.jl's VehicleParams
car = CarParams(lf=0.130, lr=0.127, width=0.228, s_max=0.4189,
    steer_cmd_min=-0.327, steer_cmd_max=0.292, speed_cmd_max=14.375,
    decel_cmd=2.5, scan_rate=40.0, scan_range=10.0)

# beam angles [rad], how far the LiDAR sits ahead of the center of gravity [m],
# the networks, and when to call the cloud
nets = load_nets("../LidarLaneNets.jl/weights")   # small_weak on the car, large in the cloud
racer = Racer(car, range(-3π / 4, 3π / 4; length=1081), 0.143, nets; schedule=RoundRobin())

# on each new scan: measured speed and yaw rate, time since the last scan
steer, speed, lane = control!(racer, ranges, v, yaw_rate, dt)
```

`Racer` also takes `selector` and `tracking` (LidarLaneNets.jl's
`SelectorConfig` and `TrackerConfig`: the simulated network, which results
to use, the filter's tuning), and `stanley`, `margin`, `k_σ`, `latency` and
`v_max` to replace its defaults (see `src/LidarRacer.jl`). `racer.plan`
holds the last line planned, in the front axle's frame, and `racer.lane` the
last lane state. When the lane is lost (no cross-track or heading error for
10 scans), the car brakes to a stop and holds its steering.

[`drive`](src/LidarRacer.jl) does everything after the estimates from any
`LaneState`: the true one in a simulator, say (LidarLaneNets.jl's training
data is collected that way).

### From the estimates to a command

1. **The lane, rebuilt**: its center starts `e` to the side of the front
   axle, in direction `ψe`, and bends by `κ_profile`, every 0.25 m out to
   `sight`; the room to each side is `w_left` and `w_right`, less `margin`.
2. **The line**: the minimum-curvature path through that room.
3. **Steering**: the car measured against the line (Stanley's `e`, `ψe` and
   the line's curvature), through the Stanley law.
4. **Speed**: as below, along the line's curvature out to `sight`, and the
   `clearance` ahead along the arc the car turns on or along the line. Both
   `sight` and `clearance` are taken `k_σ` standard deviations short of
   their estimates.

## Racing F1Tenth.jl maps

[scripts/race.jl](scripts/race.jl) drives an
[F1Tenth.jl](https://github.com/pganguli/F1Tenth.jl) car around a map. The
controller gets only the LiDAR scan, speed and yaw rate. It never sees the
car's pose or the map. It needs F1Tenth.jl's main branch (not
`archive/python-compat`), whose car has the real actuator chain (steering
limits, servo lag, VESC speed loop) and a 40 Hz LiDAR.

```sh
julia --project=scripts -e 'using Pkg; Pkg.instantiate()'
julia --project=scripts scripts/race.jl Spa               # one lap, prints a score
julia --project=scripts scripts/race.jl Spa --render      # watch it
julia -t auto --project=scripts scripts/race.jl all       # all 23 racetracks, in parallel
julia --project=scripts scripts/race.jl Spa --speed 3     # cap the speed
julia --project=scripts scripts/race.jl path/to/map --pose X Y THETA
julia -t auto --project=scripts scripts/race.jl test --cloud none    # held-out tracks, small networks only
julia --project=scripts scripts/race.jl Spa --cloud all --cloud-only --cloud-latency 100
julia -t auto --project=scripts scripts/compare.jl --seeds 8 50 100   # every configuration (results/compare-small_weak.md)
```

`--cloud` picks when to call the cloud's large networks (`none`, `all`,
`roundrobin` (the default), or a list of estimates to call every scan);
`--cloud-only` drops the small networks; `--cloud-latency` (ms, default 25)
and `--cloud-drop` set the simulated network; `--small` the car's networks
(`small_weak`, the default, or `small_strong`; the cloud always runs
`large`). The weights come from `../LidarLaneNets.jl/weights` (Git LFS).

The controller runs on each new scan (every 25 ms). Each command takes
effect 7.5 ms after its scan is published. That covers computing it and
passing it on: it is ForzaETH's measured latency for their whole race stack
on NUC5's computer, a cautious value for this lighter controller. The
command is held until the next one and goes to the car's own low-level
controllers (in F1Tenth.jl). The simulator steps every 2.5 ms, so that
latency is timed closely. (The cloud's latency is separate: it delays only
the cloud's results, which the Kalman filter puts at their own scan.)

There is no cruise speed. The speed is the fastest from which the car,
braking at the command limiter's 2.5 m/s² after a two-scan delay, reaches
every point of the line it sees no faster than that point's curve allows
(lateral acceleration `a_lat`), and also a curve as sharp as the car can
turn just past the end of what it sees. It is also capped so that speed ×
yaw rate stays within `a_lat`, and so that it can slow down short of a wall
ahead. That wall is looked for along the arc the car is turning on and along
the line it follows, whichever is clear for longer, not straight ahead: in a
turn, the wall straight ahead is the outer wall the car is turning away
from, and braking for it cost 5.5% of lap time. For the racetracks, the
track's centerline file is used only to place the car on the start line and
to score the run.

### Results on the held-out tracks: safety

The networks never saw these five tracks (Spa, Monza, Silverstone,
Oschersleben, YasMarina) in training. Every configuration ran one lap of
each, 8 times (a seed sets the LiDAR's noise and where the lap starts): 40
runs. Safety is measured from the true map and state. The full tables,
with 95% confidence intervals, run-for-run differences against the car's
networks alone, and lap times, are in
[`scripts/results/compare-small_weak.md`](scripts/results/compare-small_weak.md)
and [`scripts/results/compare-small_strong.md`](scripts/results/compare-small_strong.md);
every run is in the CSVs beside them.

**With `small_weak` on the car** (the default; the cloud runs `large`).
Crashes in 40 runs:

| Estimates from | Cloud calls per scan | 25 ms cloud latency | 50 ms | 100 ms |
|---|---|---|---|---|
| the car's networks only | 0 | 8 | 8 | 8 |
| the cloud's only | 6 | **0** | **0** | 6 |
| the car's, and the cloud's for every estimate | 6 | 7 | 2 | **0** |
| the car's, and the cloud's in turn (round robin) | 1 | 4 | 2 | 4 |
| the car's, and the cloud's far ahead (κ_profile, widths, sight) | 3 | 3 | **0** | 1 |

- At 25 ms the cloud alone is the safest: the car's weak estimates, used
  at once, cost more than the cloud's one-scan delay.
- At 50 ms (a public cloud over 5G) the car's networks with the cloud's far
  ahead are as safe as the cloud alone (no crashes, the same 0.10 m
  closest approach), with half the calls and fewer scans planning on too
  much free distance (3.9% against 4.9%); both are far safer than the car
  alone.
- At 100 ms a combination is safer than either tier alone: the car's
  networks with the cloud's for every estimate never crashed, against 6
  (the cloud alone) and 8 (the car alone).

No fixed configuration is the safest at every latency, which is the case
for switching by latency (a `CloudSchedule` that reads it).

**With `small_strong` on the car**, the car's networks alone never crash;
the cloud alone is again the safest at 25 ms and crashes 6 times at 100 ms,
and keeping the car's networks on is the safest from 50 ms on.

In about 4–5% of scans, in every configuration, the speed is set for a free
distance ahead more than 0.25 m longer than the true one (the clearance
networks' errors are biased toward too much room; see LidarLaneNets.jl).

Lap times barely differ between configurations that finish (within about
2%; the true estimates give 74.0 s on average, against about 75–78 s here).

For reference, the hand-written estimator on `main` laps these tracks in
74.6 s on average (one seed), with its own planner, which plans out to 10 m
from the scan's points (here the profiles reach 8 m), so the comparison is
approximate.

The planner, Stanley and speed settings were tuned with the hand-written
estimator on `main` (on 12 racetracks, checked on the other 11) and are
kept. The Kalman filter's process noise is the same; its measurement noise
is each network's own standard deviation for each scan.

## Tests

```sh
julia --project=. -e 'import Pkg; Pkg.test()'
```

The tests use synthetic lane states and untrained networks, and need
neither F1Tenth.jl nor the trained weights.
