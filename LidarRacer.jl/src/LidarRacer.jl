"""
    LidarRacer

Races a car around a track using only its LiDAR, without a map or a pose.
The walls are the lane boundaries. On every scan, a [`Racer`](@ref):

- **perception**: LidarLaneEstimation.jl finds the lane and the room around
  its center.
- **path**: MinimumCurvaturePlanner.jl plans the line through that room
  that bends least (wide before a corner, close to the inside at its apex).
  This is a local raceline, re-planned on every scan as far as the LiDAR
  sees.
- **estimation**: LidarLaneEstimation.jl measures the car against that line
  and smooths the result with a Kalman filter. This gives the cross-track
  error e and heading error ψe at the front axle, the line's curvature ahead,
  and how far ahead it is seen.
- **steering**: StanleyController.jl turns e and ψe into a steering angle.
- **speed**: VelocityProfilePlanner.jl sets it: as fast as the car can still
  slow down in time for what it sees, and for what it may not see yet.

It doesn't depend on a simulator. A car is described by
[`CarParams`](@ref).
"""
module LidarRacer

using LidarLaneEstimation
using MinimumCurvaturePlanner
using StanleyController
using VelocityProfilePlanner

export CarParams, SpeedConfig, speed_command, Racer, control!

# ---------------------------------------------------------------------------
# The car

"""
    CarParams(; lf, lr, width, s_max, steer_cmd_min, steer_cmd_max, speed_cmd_max, decel_cmd, scan_rate, scan_range)
    CarParams(p)

What the controller needs to know about the car and its LiDAR. The second
form reads these fields from any object that has them as properties, such as
F1Tenth.jl's `VehicleParams`.

- `lf`, `lr`: distance from the center of gravity to the front and rear axle [m]
- `width`: the car's width [m]
- `s_max`: the steering angle's physical limit [rad]
- `steer_cmd_min`, `steer_cmd_max`: the steering command's limits [rad]
  (`steer_cmd_min < 0`)
- `speed_cmd_max`: the largest speed command [m/s]
- `decel_cmd`: the deceleration the speed command can count on [m/s²]
- `scan_rate`: LiDAR scans per second [Hz]
- `scan_range`: the LiDAR's detection range [m]
"""
Base.@kwdef struct CarParams
    lf::Float64
    lr::Float64
    width::Float64
    s_max::Float64
    steer_cmd_min::Float64
    steer_cmd_max::Float64
    speed_cmd_max::Float64
    decel_cmd::Float64
    scan_rate::Float64
    scan_range::Float64
end

CarParams(p) = CarParams(; (f => getproperty(p, f) for f in fieldnames(CarParams))...)

# ---------------------------------------------------------------------------
# Speed

"""
    SpeedConfig(car::CarParams; sweep, latency, v_max=car.speed_cmd_max, kwargs...)

The limits that set the speed. There is no cruise speed: the car goes as
fast as it can still slow down in time for every curve it sees, and for the
sharpest curve it could meet just past what it sees.
"""
Base.@kwdef struct SpeedConfig
    v_max::Float64            # never faster than this [m/s]
    v_min::Float64 = 1.0      # never slower than this [m/s]
    # Lateral acceleration planned for turns [m/s²]: about half the tires'
    # peak grip (1.0 g at the rear; the front, which slides first, gives
    # 0.77 g at 0.2 rad of slip), leaving the rest for steering corrections
    # and braking. Every racetrack lap finishes from 2 to 6 m/s².
    a_lat::Float64 = 5.0
    a_brake::Float64          # deceleration the car can count on [m/s²]
    r_min::Float64            # the car's tightest turning radius [m]
    # Time from a scan's first beam until the car acts on it, and the speed
    # command changes [s]. The car drives on meanwhile.
    delay::Float64
    margin::Float64 = 0.5     # distance to keep from a wall straight ahead [m]
end

# From the car: its top speed command, the command limiter's deceleration,
# the turning radius at its smaller steering limit, and the delay from a
# scan's first beam until the next command could replace this one: the sweep
# (`sweep` seconds), the latency until the command takes effect, and one scan
# period until the next decision.
SpeedConfig(car::CarParams; sweep, latency, v_max=car.speed_cmd_max, kwargs...) = SpeedConfig(; v_max,
    a_brake=car.decel_cmd, r_min=(car.lf + car.lr) / tan(min(car.steer_cmd_max, -car.steer_cmd_min)),
    delay=sweep + latency + 1 / car.scan_rate, kwargs...)

"""
    speed_command(cfg::SpeedConfig, lane::LaneEstimate, v, yaw_rate) -> v_cmd

The first point of the velocity profile (forward-backward, from
VelocityProfilePlanner.jl) along the path ahead: the fastest speed from which
the car, braking at `a_brake` after driving on for `delay`, reaches every
point no faster than its curve allows (lateral acceleration at most `a_lat`),
can stop `margin` short of a wall straight ahead, and can still take, just
past the end of the path (`lane.sight`), a curve as sharp as it can turn.
Also capped so that the lateral acceleration in the turn the car is making
now (speed × yaw rate) is at most `a_lat`.
"""
function speed_command(cfg::SpeedConfig, lane::LaneEstimate, v, yaw_rate)
    coast = cfg.delay * abs(v)   # driven before the car reacts
    from_braking(d) = max(d - coast, 0.0)
    # points (distance from where braking can start, curvature, speed cap)
    pts = Tuple{Float64,Float64,Float64}[]
    for (j, κ) in enumerate(lane.κ_profile)
        d = (j - 1) * PROFILE_STEP
        d > lane.sight && break
        isnan(κ) || push!(pts, (from_braking(d), κ, Inf))
    end
    push!(pts, (from_braking(lane.sight), 0.0, cornering_speed(1 / cfg.r_min; a_lat=cfg.a_lat)))
    wall = lane.clearance - cfg.margin
    isfinite(wall) && push!(pts, (from_braking(wall), 0.0, cfg.v_min))
    sort!(pts; by=first)
    profile = velocity_profile(getindex.(pts, 1), getindex.(pts, 2); a_lat=cfg.a_lat, a_brake=cfg.a_brake,
        v_cap=getindex.(pts, 3))
    return clamp(min(profile[1], cfg.a_lat / max(abs(yaw_rate), eps())), cfg.v_min, cfg.v_max)
end

# ---------------------------------------------------------------------------
# The controller: what runs on the car, from a scan to a command

"""
    Racer(car, beams, lidar_dist; stanley, perception, smoothing, margin, latency, v_max)

The on-board controller of a car `car` (a [`CarParams`](@ref), or anything
`CarParams` can read) with a LiDAR whose beams point at `beams` [rad],
`lidar_dist` meters ahead of the center of gravity: lane perception, the
path planner (keeping `margin` meters from every return), the Kalman filter,
the Stanley law (which measures the errors at the front axle) and the speed
planner. A scan is published `sweep` seconds after its first beam, and its
command takes effect `latency` after that. `path_x`, `path_y` hold the last
path planned (LiDAR frame).
"""
mutable struct Racer
    perceive::LanePerception
    margin::Float64
    smooth::LaneFilter
    stanley::Stanley
    speed::SpeedConfig
    sweep::Float64
    latency::Float64
    path_x::Vector{Float64}
    path_y::Vector{Float64}
end

function Racer(car::CarParams, beams, lidar_dist; stanley=default_stanley(car),
               perception=default_perception(car), smoothing=default_smoothing(),
               margin=default_margin(car), latency=default_latency(), v_max=car.speed_cmd_max)
    # the front axle is lf ahead of the center of gravity, so lf - lidar_dist
    # ahead of the LiDAR, and lf + lr ahead of the rear axle
    perceive = LanePerception(beams, car.lf - lidar_dist; cfg=perception)
    sweep = (beams[end] - beams[1]) / 2π / car.scan_rate
    return Racer(perceive, margin, LaneFilter(car.lf + car.lr; cfg=smoothing), stanley,
        SpeedConfig(car; sweep, latency, v_max), sweep, latency, Float64[], Float64[])
end

Racer(car, beams, lidar_dist; kwargs...) = Racer(CarParams(car), beams, lidar_dist; kwargs...)

# From a scan's publication to its command taking effect [s]: computing it
# and passing it on. ForzaETH measured their whole race stack's latency on
# NUC5's computer (an Intel i5-10210U) at 7.5 ms at most on average (Baumann
# et al., "ForzaETH Race Stack", 2024, §8.1.1); this controller is lighter
# (about 0.25 ms here), so 7.5 ms is a cautious value. No transport delay
# (Ethernet, driver) has been published.
default_latency() = 0.0075

# The defaults below were tuned on 12 of the 23 racetracks (two LiDAR noise
# seeds each) and checked on the other 11; each sits in the middle of a range
# of values that all finish every lap.

# Yaw damping (Hoffmann et al.) lets the cross-track gain be stiff despite
# the servo lag: without it the car weaves from k = 3 on; with k_yaw = 0.1,
# every k from 2 to 5 finishes. The softening speed hardly matters.
default_stanley(car) = Stanley(k=3.0, k_soft=1.0, k_yaw=0.1, max_steer=car.s_max)

# Use every return the LiDAR guarantees (its detection range, 10 m on white
# walls), with the curvature as far ahead as that. Far along a wall seen at a
# grazing angle, neighboring returns are about r² Δθ / w apart (10² m² ×
# 0.25° / 1.1 m ≈ 0.4 m), so a wall is only cut at gaps over 0.8 m: 0.3 m cut
# far walls short and the car saw less of the lane (6% slower); 0.6 to 1.3 m
# all did about the same, and smaller keeps clear of real openings. The other
# settings changed nothing measurable on the racetracks (stride, smooth,
# spacing and curve_base by under 1%; the one-wall fallbacks never run
# there).
default_perception(car) = PerceptionConfig(car_halfwidth=car.width / 2, r_max=car.scan_range,
    preview=car.scan_range, jump=0.8)

# The path keeps half the car's width plus 0.15 m from every return (it
# seldom comes that close).
default_margin(car) = car.width / 2 + 0.15

# Scan noise as measured against the true lane center (standard deviations
# of the scan's errors on the tuning tracks). Process noise: 5 × the measured
# drift of the true lane from the filter's motion model (0.10 m/s, 0.17 rad/s,
# 0.16 1/m²), because the line followed is re-planned every scan and moves
# more than the lane does; 3× to 8× all work. The outlier gate and the
# curvature profile's averaging changed nothing measurable on the racetracks
# (the speed is limited by how far ahead the lane is seen, not by its
# curvature), so they keep their defaults.
default_smoothing() = FilterConfig(σ_e=0.05, σ_ψ=0.075, σ_κ=0.06, q_e=0.5, q_ψ=0.85, q_κ=0.8)

"""
    control!(r::Racer, ranges, v, yaw_rate, dt) -> (steer, speed, lane)

The steering angle [rad] and speed [m/s] to command for a new scan `ranges`,
given the measured speed and yaw rate and the time `dt` since the last scan,
with the lane estimate they came from.
"""
function control!(r::Racer, ranges, v, yaw_rate, dt)
    p = r.perceive
    detect_lane!(p, ranges)
    c = lane_corridor(p, r.margin)
    r.path_x, r.path_y = c === nothing ? (p.center_x, p.center_y) :
        min_curvature_path(c.x, c.y, c.lower, c.upper; normals=(c.nx, c.ny))
    lane = track!(r.smooth, measure!(p, r.path_x, r.path_y), v, yaw_rate, dt)
    steer = steering_angle(r.stanley, lane.e, lane.ψe, v; yaw_rate, path_yaw_rate=lane.κ * v)
    return steer, speed_command(r.speed, lane, v, yaw_rate), lane
end

end # module
