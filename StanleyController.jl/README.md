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

c = Stanley(k=6.0, k_soft=1.0, k_yaw=0.05, max_steer=0.4189)

# errors you estimated yourself (from a camera, a LiDAR, ...)
δ = steering_angle(c, e, ψe, v; yaw_rate=r, path_yaw_rate=κ * v)

# or against a known path
path = Path(xs, ys; closed=true)
err = tracking_error(path, front_axle(x, y, θ, lf)..., θ)   # e, ψe, κ, s, segment
δ = steering_angle(c, err, v; yaw_rate=r)

# speed for a curve: lateral acceleration ≤ a_lat
v_cmd = curvature_speed(κ; v_max=5.0, a_lat=4.0, v_min=1.0)
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
treats the walls as lane boundaries and keeps the car in the middle at a
cruise speed, slowing for curves. The controller gets only the LiDAR scan,
speed and yaw rate. It never sees the car's pose or the map.

```sh
julia --project=examples -e 'using Pkg; Pkg.instantiate()'
julia --project=examples examples/lane_keep.jl Spa               # one lap, prints a score
julia --project=examples examples/lane_keep.jl Spa --render      # watch it
julia --project=examples examples/lane_keep.jl all --speed 7     # all 23 racetracks
julia --project=examples examples/lane_keep.jl path/to/map --pose X Y THETA
```

Each step, the scan is turned into the Stanley inputs:

1. Convert the ranges to points in the car frame.
2. Split the points into a left wall and a right wall. Each starts at the
   nearest return beside the car and is followed beam by beam until a gap,
   so a wall that curves across the front of the car stays one wall.
3. Pair each point with the nearest point on the other wall. The midpoints,
   in order along the wall, trace the lane center.
4. Smooth the midpoints into a local `Path` and call `tracking_error` at the
   front axle to get `e`, `ψe` and the curvature.
5. Combine this with the earlier scans in an unscented Kalman filter (Wan and
   van der Merwe, 2000, from [KalmanFilters.jl](https://github.com/JuliaGNSS/KalmanFilters.jl)).
   Between scans it predicts how `e`, `ψe` and the curvature change from the
   car's speed and yaw rate, then moves part of the way toward the new scan.

Without step 5, the heading error from a single scan is off by about 4°. The
Stanley law passes that straight to the wheels, and since the sideways jolt
grows with the square of speed, the car twitches at higher speeds. Turns show
up in the yaw rate first, so the filter predicts them instead of lagging behind.
A scan far from the prediction is ignored, unless that keeps happening (for
example, a wall came into view). Then the filter restarts from the scan.

Speed is the cruise speed, capped by `curvature_speed` using the largest
curvature in the next 4 m, and by the free distance straight ahead. That
curvature profile is also averaged over scans, which stops noise from
slowing the car on straights.

The script handles three edge cases:

- **One wall visible:** it follows that wall at half the lane width. The
  width is a median over the last 10 s, so a short wide stretch doesn't
  change it.
- **Deep in a hairpin:** only the outer wall is visible, wrapping around the
  car, while the inner wall is in the scan's rear blind spot. The script
  follows that wall and keeps the lane direction from the previous step.
- **Racetrack scoring:** the track's centerline file is used only to place
  the car on the start line and to score the run.

On all 23 f1tenth_racetracks maps, every run finishes a lap at cruise speeds
of 3, 5, 7, 10 and 15 m/s with no wall contact. The car's average distance
from the centerline is 2–5 cm, the largest is under 0.2 m, and the heading
error estimate is off by about 1.6° on average. The one exception is
Shanghai's hairpin, where the map's rounded end is much wider than the track
and the car passes up to 0.78 m from the reference line.

## Tests

```sh
julia --project=. -e 'import Pkg; Pkg.test()'
```
