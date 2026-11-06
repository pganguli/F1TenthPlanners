# LidarLaneEstimation.jl

Finds the lane between two walls in a 2D LiDAR scan, and where the vehicle
is in it, for any lane-keeping or path-tracking controller. It needs only the
scan, plus the vehicle's speed and yaw rate for smoothing over time. It never
uses a map or the vehicle's pose.

Each estimate gives:

- the lane's center line ahead, as points in the vehicle frame;
- the cross-track error `e`, heading error `ψe` and curvature `κ` at a
  reference point you choose: the front axle for the Stanley law, the rear
  axle for pure pursuit, or the center of gravity for most LQR and MPC models;
- the sharpest curvature ahead and the free distance straight ahead, for
  choosing a speed.

## Use

```julia
using LidarLaneEstimation

# 1080 beams over 4.7 rad; the reference point 0.16 m ahead of the LiDAR
perceive = LanePerception(1080, 4.7, 0.16)
# the same reference point is 0.33 m ahead of the rear axle
smooth = LaneFilter(0.33)

# every scan (ranges in meters), with the vehicle's speed and yaw rate
lane = track!(smooth, estimate_lane!(perceive, ranges), v, yaw_rate, dt)
lane.e, lane.ψe, lane.κ          # where the vehicle is in the lane
lane.κ_ahead, lane.clearance     # for the speed
perceive.cx, perceive.cy         # the center line, in the LiDAR frame
```

Signs: x is forward and y to the left, and angles are counterclockwise.
`e > 0` when the lane center is to the left, `ψe > 0` when the lane points
left, and `κ > 0` when it turns left. `PerceptionConfig` and `FilterConfig`
hold the tuning, with each setting's effect described next to it.

## How it works

`estimate_lane!` works on one scan:

1. Convert the ranges to points.
2. Split the points into a left and a right wall. Each starts at the nearest
   return beside the vehicle and is followed beam by beam until a gap, so a
   wall that curves across the front of the vehicle stays one wall.
3. Pair each point with the nearest point on the other wall. The midpoints,
   in order along the wall, trace the lane center.
4. Smooth them into a line and measure the reference point against it.

It also handles two edge cases:

- **One wall visible:** the center is half a lane width in from that wall.
  The width is a median over the last 10 s, so a short wide stretch doesn't
  change it.
- **Deep in a hairpin:** only the outer wall is visible, wrapping around the
  vehicle, while the inner wall is in the scan's rear blind spot. It follows
  that wall and keeps the lane direction from the previous scan.

One scan's heading error is off by a few degrees, which makes a vehicle
twitch at speed. `track!` combines the scans with an unscented Kalman filter
(Wan and van der Merwe, 2000, from
[KalmanFilters.jl](https://github.com/JuliaGNSS/KalmanFilters.jl)). Between
scans it predicts how `e`, `ψe` and `κ` change from the speed and yaw rate,
then moves part of the way toward the new scan. Turns show up in the yaw
rate first, so they are predicted rather than delayed. A scan far from the
prediction is ignored, unless that keeps happening (for example, a wall came
into view); then the filter restarts from the scan.

For an example, see `examples/lane_keep.jl` in the sibling package
StanleyController.jl, which drives an F1Tenth.jl car around 23 racetracks
with the Stanley law.

## Tests

```sh
julia --project=. -e 'import Pkg; Pkg.test()'
```
