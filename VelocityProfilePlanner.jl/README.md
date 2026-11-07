# VelocityProfilePlanner.jl

Velocity profiles along a path by the forward-backward method: the
three-pass algorithm of Kapania, Subosits and Gerdes, "A sequential two-step
algorithm for fast generation of vehicle racing trajectories" (ASME Journal of
Dynamic Systems, Measurement, and Control 138(9), 2016), also called a
quasi-steady-state velocity profile. Nothing is tied to a vehicle or a path
planner: give it the arc length and curvature at points along a path and the
vehicle's acceleration limits.

## Use

```julia
using VelocityProfilePlanner

# points along a path: arc length s [m] (nondecreasing) and curvature κ [1/m]
v = velocity_profile(s, κ; a_lat=5.0, a_brake=2.5)

# starting from the current speed, ending in a stop, tires shared between
# cornering and braking or accelerating
v = velocity_profile(s, κ; a_lat=5.0, a_brake=2.5, a_accel=2.5, a_grip=9.8,
                     v_start=v_now, v_end=0.0)

# other limits at points (an obstacle to stop for, a speed zone)
v = velocity_profile(s, κ; a_lat=5.0, a_brake=2.5, v_cap=caps)

# the fastest speed through one curve
cornering_speed(κ; a_lat=5.0, v_max=10.0)
```

## How it works

1. **Cornering.** At each point, the fastest speed with lateral acceleration
   `v² |κ|` at most `a_lat`, and at most `v_max` and `v_cap[i]`.
2. **Backward pass.** From the last point (at most `v_end`, what lies past
   the path allows) toward the first, each point's speed is lowered until the
   vehicle can brake from it to the next point's speed at `a_brake`:
   `v_i ≤ √(v_{i+1}² + 2 a_brake (s_{i+1} - s_i))`.
3. **Forward pass** (when `v_start` is given). From the first point on, each
   point's speed is lowered until the vehicle can reach it from the one
   before at `a_accel`.

With a finite `a_grip`, braking and accelerating may only use the grip
cornering leaves, `√(a_grip² - (v² κ)²)` (a friction circle), as in Kapania
et al. Without it, the limits are independent, as for a vehicle whose
braking and acceleration are limited by its drivetrain or controller well
inside the tires' grip.

The profile is quasi-steady-state: it treats each point as if the vehicle had
settled into it, with no transients and no load transfer.

## Tests

```sh
julia --project=. -e 'import Pkg; Pkg.test()'
```
