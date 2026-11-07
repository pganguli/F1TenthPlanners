"""
    VelocityProfilePlanner

Velocity profiles along a path by the forward-backward method (the
three-pass algorithm of Kapania, Subosits and Gerdes, "A sequential two-step
algorithm for fast generation of vehicle racing trajectories", ASME Journal
of Dynamic Systems, Measurement, and Control 138(9), 2016; also called a
quasi-steady-state velocity profile), with nothing tied to a particular
vehicle or planner:

1. at each point, the fastest speed the curve allows with lateral
   acceleration at most `a_lat` (and any other speed cap);
2. a backward pass, so the vehicle can brake (at most `a_brake`) from every
   point down to the cap of every point after it;
3. a forward pass, so it can accelerate (at most `a_accel`) up to each point.

Optionally, braking and accelerating share the tires' grip with cornering
(a friction circle of radius `a_grip`). See [`velocity_profile`](@ref) and
[`cornering_speed`](@ref).
"""
module VelocityProfilePlanner

export cornering_speed, velocity_profile

"""
    cornering_speed(κ; a_lat, v_max=Inf, v_min=0.0) -> v

The fastest speed that keeps lateral acceleration `v² |κ|` at or below
`a_lat` [m/s²] on a curve of curvature `κ` [1/m], clamped to
`[v_min, v_max]` (`v_max` on a straight).
"""
function cornering_speed(κ::Real; a_lat::Real, v_max::Real=Inf, v_min::Real=0.0)
    a_lat > 0 || throw(ArgumentError("a_lat must be positive, got $a_lat"))
    v_min <= v_max || throw(ArgumentError("v_min ($v_min) must not exceed v_max ($v_max)"))
    iszero(κ) && return Float64(v_max)
    return clamp(sqrt(a_lat / abs(κ)), v_min, v_max)
end

"""
    velocity_profile(s, κ; a_lat, a_brake, a_accel=Inf, a_grip=Inf, v_max=Inf,
                     v_cap=nothing, v_start=Inf, v_end=Inf) -> v

The speed [m/s] at each point of a path, at arc length `s[i]` [m]
(nondecreasing) and curvature `κ[i]` [1/m]: as fast as possible while

- lateral acceleration `v² |κ|` stays at most `a_lat` [m/s²], and the speed
  at most `v_max` and `v_cap[i]` (if given: other limits, such as a stop);
- braking from point to point needs at most `a_brake` [m/s²], down to
  `v_end` at the last point (what lies past it allows);
- accelerating needs at most `a_accel` [m/s²], from `v_start` at the first
  point (the vehicle's speed now; `Inf` leaves the forward pass out).

With a finite `a_grip` [m/s²], braking and accelerating may only use the
grip cornering leaves: `√(a_grip² - (v² κ)²)` at most (a friction circle).
"""
function velocity_profile(s::AbstractVector{<:Real}, κ::AbstractVector{<:Real}; a_lat::Real, a_brake::Real,
                          a_accel::Real=Inf, a_grip::Real=Inf, v_max::Real=Inf, v_cap=nothing,
                          v_start::Real=Inf, v_end::Real=Inf)
    n = length(s)
    length(κ) == n || throw(DimensionMismatch("s and κ have different lengths"))
    v_cap === nothing || length(v_cap) == n || throw(DimensionMismatch("v_cap must have one entry per point"))
    n >= 1 || throw(ArgumentError("need at least one point"))
    issorted(s) || throw(ArgumentError("s must be nondecreasing"))
    a_brake > 0 || throw(ArgumentError("a_brake must be positive, got $a_brake"))
    a_accel > 0 || throw(ArgumentError("a_accel must be positive, got $a_accel"))
    # longitudinal acceleration available at speed v on curvature κ
    function available(a, v, κ)
        (isinf(a_grip) || iszero(κ)) && return a
        a_y = v^2 * abs(κ)
        return a_y >= a_grip ? 0.0 : min(a, sqrt(a_grip^2 - a_y^2))
    end

    v = [cornering_speed(κ[i]; a_lat, v_max) for i in 1:n]
    v_cap === nothing || (v .= min.(v, v_cap))
    v[n] = min(v[n], v_end)
    for i in n-1:-1:1   # backward: reach each next point's speed by braking
        v[i] = min(v[i], sqrt(v[i+1]^2 + 2available(a_brake, v[i+1], κ[i+1]) * (s[i+1] - s[i])))
    end
    if isfinite(v_start)
        v[1] = min(v[1], v_start)
        for i in 1:n-1  # forward: reach each next point by accelerating
            v[i+1] = min(v[i+1], sqrt(v[i]^2 + 2available(a_accel, v[i], κ[i]) * (s[i+1] - s[i])))
        end
    end
    return v
end

end # module
