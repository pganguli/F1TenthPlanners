# LidarRacer.jl

Races an F1TENTH car around a track using only its LiDAR. It has no map and
no pose. The walls are the lane boundaries. On every scan, the controller
plans the line through the lane that bends least, steers along it, and goes
as fast as it can still slow down in time for what it sees. The line is a
local raceline, re-planned on every scan as far ahead as the LiDAR sees. It
is not a global raceline computed ahead of time from a map.

The controller only combines the work of four sibling packages:

| Step | Package |
|---|---|
| finds the lane and the room around its center | [LidarLaneEstimation.jl](../LidarLaneEstimation.jl) |
| plans the line through it that bends least | [MinimumCurvaturePlanner.jl](../MinimumCurvaturePlanner.jl) |
| measures the car against that line and smooths the result with an unscented Kalman filter, giving `e`, `ψe` and the curvature at the front axle | [LidarLaneEstimation.jl](../LidarLaneEstimation.jl) |
| turns `e` and `ψe` into a steering command | [StanleyController.jl](../StanleyController.jl) |
| sets the speed | [VelocityProfilePlanner.jl](../VelocityProfilePlanner.jl) |

It doesn't depend on a simulator. The simulator only comes in through the
script in [scripts/](scripts/).

## Use

```julia
using LidarRacer

# the car and its LiDAR; CarParams(p) also reads these from any object that
# has them, such as F1Tenth.jl's VehicleParams
car = CarParams(lf=0.130, lr=0.127, width=0.228, s_max=0.4189,
    steer_cmd_min=-0.327, steer_cmd_max=0.292, speed_cmd_max=14.375,
    decel_cmd=2.5, scan_rate=40.0, scan_range=10.0)

# beam angles [rad], and how far the LiDAR sits ahead of the center of gravity [m]
racer = Racer(car, range(-3π / 4, 3π / 4; length=1081), 0.143)

# on each new scan: measured speed and yaw rate, time since the last scan
steer, speed, lane = control!(racer, ranges, v, yaw_rate, dt)
```

`Racer` takes `stanley`, `perception`, `smoothing`, `margin`, `latency` and
`v_max` keywords to replace its defaults (see `src/LidarRacer.jl`).
`racer.path_x` and `racer.path_y` hold the last line planned, in the LiDAR's
frame.

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
```

The controller runs on each new scan (every 25 ms). Each command takes
effect 7.5 ms after its scan is published. That covers computing it and
passing it on: it is ForzaETH's measured latency for their whole race stack
on NUC5's computer, a cautious value for this lighter controller. The
command is held until the next one and goes to the car's own low-level
controllers (in F1Tenth.jl). The simulator steps every 2.5 ms, so that
latency is timed closely. The controller tolerates up to 50 ms of latency
(laps 2.3% slower); at 100 ms it starts to hit walls.

There is no cruise speed. The speed is the fastest from which the car,
braking at the command limiter's 2.5 m/s² after a two-scan delay, reaches
every point of the line it sees no faster than that point's curve allows
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
in `src/LidarRacer.jl`.

## Tests

```sh
julia --project=. -e 'import Pkg; Pkg.test()'
```

The tests use synthetic scans and don't need F1Tenth.jl.
