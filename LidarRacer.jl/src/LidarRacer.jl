"""
    LidarRacer

Races a car around a track using only its LiDAR, without a map or a pose.
The walls are the lane boundaries. On every scan, a [`Racer`](@ref):

- **estimation**: LidarLaneNets.jl's networks estimate the lane relative to
  its center (cross-track and heading error at the front axle, the
  curvature and the free room on each side ahead, how far ahead the lane is
  seen, the free distance along a fan of arcs), small ones on the car at
  every scan and large ones in the cloud when a schedule calls them, and a
  Kalman filter combines them over time.
- **path**: the lane center and the room around it are rebuilt from those
  estimates, and MinimumCurvaturePlanner.jl plans the line through that
  room that bends least: a local raceline, re-planned on every scan.
- **steering**: the car is measured against that line, and
  StanleyController.jl turns the errors into a steering angle.
- **speed**: VelocityProfilePlanner.jl sets it: as fast as the car can still
  slow down in time for what it sees, and for what it may not see yet.

[`drive`](@ref) does the path, steering and speed from any [`LaneState`](@ref)
(the true one in a simulator, say); [`control!`](@ref) adds the networks.
It doesn't depend on a simulator. A car is described by [`CarParams`](@ref).
"""
module LidarRacer

using LidarLaneNets
using MinimumCurvaturePlanner
using PrecompileTools
using StanleyController
using VelocityProfilePlanner

export CarParams, SpeedConfig, speed_command, Planner, Plan, drive, lane_center, Racer, control!

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
    margin::Float64 = 0.5     # distance to keep from a wall ahead [m]
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
    speed_command(cfg::SpeedConfig, κ_profile, sight, free, v, yaw_rate) -> v_cmd

The first point of the velocity profile (forward-backward, from
VelocityProfilePlanner.jl) along the path ahead: the fastest speed from which
the car, braking at `a_brake` after driving on for `delay`, reaches every
point no faster than its curve allows (lateral acceleration at most `a_lat`),
can slow to `v_min` `margin` short of a wall `free` meters ahead, and can
still take, just past the end of the path (`sight` meters ahead), a curve as
sharp as it can turn. `κ_profile[j]` is the path's curvature `(j - 1) *
PROFILE_STEP` meters ahead (NaN where unknown). Also capped so that the
lateral acceleration in the turn the car is making now (speed × yaw rate)
is at most `a_lat`.
"""
function speed_command(cfg::SpeedConfig, κ_profile::AbstractVector{<:Real}, sight::Real, free::Real, v, yaw_rate)
    coast = cfg.delay * abs(v)   # driven before the car reacts
    from_braking(d) = max(d - coast, 0.0)
    # points (distance from where braking can start, curvature, speed cap)
    pts = Tuple{Float64,Float64,Float64}[]
    for (j, κ) in enumerate(κ_profile)
        d = (j - 1) * PROFILE_STEP
        d > sight && break
        isnan(κ) || push!(pts, (from_braking(d), κ, Inf))
    end
    push!(pts, (from_braking(sight), 0.0, cornering_speed(1 / cfg.r_min; a_lat=cfg.a_lat)))
    wall = free - cfg.margin
    isfinite(wall) && push!(pts, (from_braking(wall), 0.0, cfg.v_min))
    sort!(pts; by=first)
    profile = velocity_profile(getindex.(pts, 1), getindex.(pts, 2); a_lat=cfg.a_lat, a_brake=cfg.a_brake,
        v_cap=getindex.(pts, 3))
    return clamp(min(profile[1], cfg.a_lat / max(abs(yaw_rate), eps())), cfg.v_min, cfg.v_max)
end

# ---------------------------------------------------------------------------
# From the estimates to a steering angle and a speed

"""
    Planner(car::CarParams; sweep, latency, stanley, margin, k_σ, v_max)

Everything after the estimates: the path planner (keeping `margin` meters
from the walls), the Stanley law and the speed planner. `sight` and
`clearance` are taken `k_σ` standard deviations short of their estimates.
"""
struct Planner
    margin::Float64
    k_σ::Float64
    stanley::Stanley
    speed::SpeedConfig
end

Planner(car::CarParams; sweep, latency=default_latency(), stanley=default_stanley(car), margin=default_margin(car),
        k_σ=1.0, v_max=car.speed_cmd_max, speed...) =
    Planner(margin, k_σ, stanley, SpeedConfig(car; sweep, latency, v_max, speed...))

"""
    Plan

What [`drive`](@ref) planned, in the front axle's frame (x forward, y
left): the line `x`, `y`; the car against it (`e`, `ψe`, `κ`, as
StanleyController.jl measures them); the line's curvature ahead
(`κ_profile`, every `PROFILE_STEP` meters); the `sight` and free distance
`free` the speed was set for.
"""
struct Plan
    x::Vector{Float64}
    y::Vector{Float64}
    e::Float64
    ψe::Float64
    κ::Float64
    κ_profile::Vector{Float64}
    sight::Float64
    free::Float64
end

"""
    lane_center(lane::LaneState, reach) -> (x, y, h, wl, wr)

The lane center rebuilt from the estimates, in the front axle's frame: its
point beside the car is `e` to the side, it starts in direction `ψe`, and
it bends by `κ_profile`, out to `reach` meters (or the first unknown
point), every `PROFILE_STEP` meters; with its heading `h` and the free room
to each side at each point. One more point half a meter behind lets the car
be measured against it.
"""
function lane_center(lane::LaneState, reach::Real)
    h, x, y = lane.ψe, -lane.e * sin(lane.ψe), lane.e * cos(lane.ψe)
    xs, ys, hs = [x - 0.5cos(h), x], [y - 0.5sin(h), y], [h, h]
    wl, wr = [lane.w_left[1], lane.w_left[1]], [lane.w_right[1], lane.w_right[1]]
    κp = lane.κ_profile
    for j in 2:length(PROFILE_S)
        PROFILE_S[j] > reach && break
        any(isnan, (κp[j], lane.w_left[j], lane.w_right[j])) && break
        hn = h + PROFILE_STEP * (κp[j-1] + κp[j]) / 2
        hm, half = (h + hn) / 2, (hn - h) / 2
        chord = PROFILE_STEP * (abs(half) < 1e-9 ? 1.0 : sin(half) / half)   # of an arc of constant curvature
        x, y, h = x + chord * cos(hm), y + chord * sin(hm), hn
        push!(xs, x); push!(ys, y); push!(hs, h)
        push!(wl, lane.w_left[j]); push!(wr, lane.w_right[j])
    end
    return xs, ys, hs, wl, wr
end

# Curvature of the circle through the path's points at s - base/2, s and
# s + base/2, or NaN when less than half that stretch is on the path.
function _curvature_at(p::Path, s, base)
    lo, hi = max(s - base / 2, 0.0), min(s + base / 2, p.s[end])
    hi - lo < base / 2 && return NaN
    a, b, c = _point_at(p, lo), _point_at(p, (lo + hi) / 2), _point_at(p, hi)
    cross = (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1])
    return 2cross / (hypot(b[1] - a[1], b[2] - a[2]) * hypot(c[1] - b[1], c[2] - b[2]) * hypot(c[1] - a[1], c[2] - a[2]))
end

function _point_at(p::Path, s)
    i = clamp(searchsortedlast(p.s, s), 1, length(p.s) - 1)
    t = (s - p.s[i]) / p.seg_length[i]
    return (p.x[i] + t * (p.x[i+1] - p.x[i]), p.y[i] + t * (p.y[i+1] - p.y[i]))
end

# The free distance along the arc of curvature κ, from the fan of arcs
# (interpolated; past the fan's ends, its end arc).
function _free_along(free, κ)
    k = clamp(κ, first(CLEARANCE_K), last(CLEARANCE_K))
    u = (k - first(CLEARANCE_K)) / step(CLEARANCE_K) + 1
    i = min(floor(Int, u), length(CLEARANCE_K) - 1)
    return free[i] + (u - i) * (free[i+1] - free[i])
end

"""
    drive(pl::Planner, lane::LaneState, v, yaw_rate) -> (steer, speed, plan)

The steering angle [rad] and speed [m/s] for the lane `lane`, given the
measured speed and yaw rate, and the [`Plan`](@ref) they came from.
"""
function drive(pl::Planner, lane::LaneState, v, yaw_rate)
    sight = clamp(lane.sight - pl.k_σ * lane.σ_sight, 0.0, SIGHT_MAX)
    xs, ys, hs, wl, wr = lane_center(lane, max(sight, 1.0))
    # the room each point may move along its left normal, keeping `margin` from the walls
    upper = max.(wl .- pl.margin, 0.0)
    lower = min.(pl.margin .- wr, 0.0)
    px, py = length(xs) >= 3 ?
        min_curvature_path(xs, ys, lower, upper; normals=(-sin.(hs), cos.(hs))) : (xs, ys)
    path = Path(px, py)
    err = tracking_error(path, 0.0, 0.0, 0.0)
    κ = _curvature_at(path, err.s, 1.0)
    κ = isnan(κ) ? lane.κ : κ
    profile = [_curvature_at(path, err.s + d, 1.0) for d in PROFILE_S]
    clear = max.(lane.clearance .- pl.k_σ .* lane.σ_clearance, 0.0)
    turning = abs(v) > 0.5 ? yaw_rate / abs(v) : 0.0
    # clear for as long as either the arc driven or the line is (see the README)
    free = max(_free_along(clear, turning), _free_along(clear, κ))
    free >= CLEARANCE_MAX - 0.5 && (free = Inf)   # nothing within range
    steer = steering_angle(pl.stanley, err.e, err.ψe, v; yaw_rate, path_yaw_rate=κ * v)
    speed = speed_command(pl.speed, profile, sight, free, v, yaw_rate)
    return steer, speed, Plan(px, py, err.e, err.ψe, κ, profile, sight, free)
end

# ---------------------------------------------------------------------------
# The controller: what runs on the car, from a scan to a command

"""
    Racer(car, beams, lidar_dist, nets; schedule, selector, tracking, stanley, margin, k_σ, latency, v_max)

The on-board controller of a car `car` (a [`CarParams`](@ref), or anything
`CarParams` can read) with a LiDAR whose beams point at `beams` [rad],
`lidar_dist` meters ahead of the center of gravity, estimating the lane with
the networks `nets` (as from `LidarLaneNets.load_nets`; without `nets.large`,
the small tier alone). `schedule` (a `LidarLaneNets.CloudSchedule`) decides
which estimates to ask the cloud's large networks for at each scan, given
the last lane state; `selector` and `tracking` tune LidarLaneNets.jl's
selector and Kalman filter; the rest, the [`Planner`](@ref). A scan is
published `sweep` seconds after its first beam, and its command takes effect
`latency` after that. `plan` holds the last [`Plan`](@ref).

When the lane is lost (no cross-track or heading error for too long), the
car brakes to a stop and holds its steering.
"""
mutable struct Racer{E}
    estimator::E
    tracker::LaneTracker
    planner::Planner
    sweep::Float64
    latency::Float64
    t::Float64
    steer::Float64
    plan::Union{Nothing,Plan}
    lane::Union{Nothing,LaneState}
end

function Racer(car::CarParams, beams, lidar_dist, nets; schedule=RoundRobin(), selector=SelectorConfig(),
               tracking=TrackerConfig(),
               stanley=default_stanley(car), margin=default_margin(car), k_σ=1.0, latency=default_latency(),
               v_max=car.speed_cmd_max)
    sweep = (beams[end] - beams[1]) / 2π / car.scan_rate
    planner = Planner(car; sweep, latency, stanley, margin, k_σ, v_max)
    return Racer(TieredEstimator(nets; schedule, cfg=selector), LaneTracker(car.lf + car.lr; cfg=tracking), planner,
        sweep, latency, 0.0, 0.0, nothing, nothing)
end

Racer(car, beams, lidar_dist, nets; kwargs...) = Racer(CarParams(car), beams, lidar_dist, nets; kwargs...)

# From a scan's publication to its command taking effect [s]: computing it
# and passing it on. ForzaETH measured their whole race stack's latency on
# NUC5's computer (an Intel i5-10210U) at 7.5 ms at most on average (Baumann
# et al., "ForzaETH Race Stack", 2024, §8.1.1); the small networks take well
# under 1 ms here, so 7.5 ms is a cautious value. No transport delay
# (Ethernet, driver) has been published.
default_latency() = 0.0075

# Yaw damping (Hoffmann et al.) lets the cross-track gain be stiff despite
# the servo lag: without it the car weaves from k = 3 on; with k_yaw = 0.1,
# every k from 2 to 5 finishes (tuned with the hand-written estimator).
default_stanley(car) = Stanley(k=3.0, k_soft=1.0, k_yaw=0.1, max_steer=car.s_max)

# The path keeps half the car's width plus 0.15 m from the walls.
default_margin(car) = car.width / 2 + 0.15

"""
    control!(r::Racer, ranges, v, yaw_rate, dt) -> (steer, speed, lane)

The steering angle [rad] and speed [m/s] to command for a new scan `ranges`,
given the measured speed and yaw rate and the time `dt` since the last scan,
with the [`LaneState`](@ref) they came from.
"""
function control!(r::Racer, ranges, v, yaw_rate, dt)
    r.t += dt
    meas = observe!(r.estimator, ranges, r.t; lane=r.lane, v, yaw_rate)
    lane = r.lane = track!(r.tracker, meas, v, yaw_rate, r.t)
    if lane.status === :lost
        r.plan = nothing
        return r.steer, 0.0, lane
    end
    r.steer, speed, r.plan = drive(r.planner, lane, v, yaw_rate)
    return r.steer, speed, lane
end

include("precompile.jl")

end # module
