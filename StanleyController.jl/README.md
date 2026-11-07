# StanleyController.jl

The Stanley steering law for path tracking (Hoffmann, Tomlin, Montemerlo and
Thrun, 2007; Section 2.3 of Snider, *Automatic Steering Methods for
Autonomous Automobile Path Tracking*, 2009). It doesn't depend on any
vehicle, simulator or sensor.

```
δ = ψe + atan(k ⋅ e / (k_soft + |v|)) − k_yaw ⋅ (r − r_path)
```

## Signs

Angles are counterclockwise-positive and `δ > 0` steers left. Both errors are
measured at the **front axle**:

- `e`, the cross-track error, is `> 0` when the path is to the **left** of the axle.
- `ψe`, the heading error, is the path heading minus the vehicle heading, and is `> 0` when the path points **left**.

## Use

```julia
using StanleyController

c = Stanley(k=1.0, k_soft=1.0, k_yaw=0.0, max_steer=0.4189)

# errors you estimated yourself (from a camera, a LiDAR, ...)
δ = steering_angle(c, e, ψe, v; yaw_rate=r, path_yaw_rate=κ * v)

# or against a known path
path = Path(xs, ys; closed=true)
err = tracking_error(path, front_axle(x, y, θ, lf)..., θ)   # e, ψe, κ, s, segment
δ = steering_angle(c, err, v; yaw_rate=r)

```

| Parameter | Meaning |
|---|---|
| `k` | Cross-track gain [1/s]. Larger values track more tightly; the paper warns that too large a value causes instability. |
| `k_soft` | Softening speed [m/s], so the law stays gentle near standstill. Set it to `0` for the paper's exact Eq. 5. |
| `k_yaw` | Yaw-rate damping [s] (Hoffmann et al.). Off by default. |
| `max_steer` | Output clamp [rad]. |

`tracking_error` searches every segment by default. Pass `hint` (the last
segment) and `window` to search near it only. Past the ends of an open path,
the end segments are extended.

## Example: LiDAR lane keeping on F1Tenth.jl

[examples/lane_keep.jl](examples/lane_keep.jl) drives an
[F1Tenth.jl](https://github.com/pganguli/F1Tenth.jl) car around a map. It
treats the walls as lane boundaries and follows the line through the lane
that bends least, as fast as it can still slow down in time for what it
sees. The controller gets
only the LiDAR scan, speed and yaw rate. It never sees the car's pose or the
map. It needs F1Tenth.jl's main branch (not `archive/python-compat`), whose
car has the real actuator chain (steering limits, servo lag, VESC speed loop)
and a 40 Hz LiDAR.

```sh
julia --project=examples -e 'using Pkg; Pkg.instantiate()'
julia --project=examples examples/lane_keep.jl Spa               # one lap, prints a score
julia --project=examples examples/lane_keep.jl Spa --render      # watch it
julia -t auto --project=examples examples/lane_keep.jl all       # all 23 racetracks, in parallel
julia --project=examples examples/lane_keep.jl Spa --speed 3     # cap the speed
julia --project=examples examples/lane_keep.jl path/to/map --pose X Y THETA
```

On each new scan (every 25 ms), the on-board controller (`LaneKeeper`)
runs, using sibling packages: [LidarLaneEstimation.jl](../LidarLaneEstimation.jl)
finds the lane and the room around its center;
[MinimumCurvaturePlanner.jl](../MinimumCurvaturePlanner.jl) plans the path
through it that bends least; LidarLaneEstimation.jl measures the car against
that path and smooths the result with an unscented Kalman filter, giving
`e`, `ψe` and the curvature at the front axle; `steering_angle` turns them
into a steering command; and
[VelocityProfilePlanner.jl](../VelocityProfilePlanner.jl) sets the speed.
Each command takes effect 7.5 ms after its scan is published (computing and
passing it on: ForzaETH's measured latency for their whole race stack on
NUC5's computer, a cautious value for this lighter controller), is held
until the next one, and goes to the car's own low-level controllers (in
F1Tenth.jl). The simulator steps every 2.5 ms, so that latency is timed
closely. The controller tolerates up to 50 ms of latency (laps 2.3%
slower); at 100 ms it starts to hit walls.

There is no cruise speed. The speed is the fastest from which the car,
braking at the command limiter's 2.5 m/s² after a two-scan delay, reaches
every point of the path it sees no faster than that point's curve allows
(lateral acceleration `a_lat`), and also a curve as sharp as the car can
turn just past the end of what it sees. It is also capped so that speed ×
yaw rate stays within `a_lat`, and by the free distance straight ahead. For
the racetracks, the track's centerline file is used only to place the car on
the start line and to score the run.

Every run on the 23 f1tenth_racetracks maps finishes a lap, at average
speeds of 4.8–5.9 m/s, with the car's body at least 0.23 m from the walls.
The speed is limited mostly by how far ahead the LiDAR sees the lane and the
2.5 m/s² braking, more than by grip.

The settings come from the car or the sensor where they can (wheelbase,
width, steering and braking limits, scan rate, the LiDAR's 10 m detection
range), are measured where they can't (the filter's scan noise, from the
scan's errors against the true lane center), and the rest (Stanley's gains,
the filter's process noise, the gap that ends a wall) were tuned on 12
racetracks with two LiDAR noise seeds each, then checked on the other 11:
every lap there finished too, with the same mean wall clearance (0.50 m
against 0.48 m). Each tuned value sits in the middle of a range of values
that all finish every lap. The remaining perception and filter settings
changed nothing measurable on these tracks and keep their defaults. See the
comments at `default_stanley`, `default_perception` and `default_smoothing`
in the example.

## Tests

```sh
julia --project=. -e 'import Pkg; Pkg.test()'
```
