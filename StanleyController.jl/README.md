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

## Used by

[LidarRacer.jl](../LidarRacer.jl) uses this law to steer an F1Tenth.jl car
around racetracks using only its LiDAR.

## Tests

```sh
julia --project=. -e 'import Pkg; Pkg.test()'
```
