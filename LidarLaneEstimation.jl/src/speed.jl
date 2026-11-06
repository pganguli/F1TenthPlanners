# Speed

"""
    SpeedConfig(; kwargs...)

Cruise speed and the limits that lower it.
"""
Base.@kwdef struct SpeedConfig
    cruise::Float64 = 5.0     # the fixed speed on straights [m/s]
    v_min::Float64 = 1.0      # never slower than this [m/s]
    a_lat::Float64 = 4.0      # lateral acceleration allowed in turns [m/s²]
    a_brake::Float64 = 4.0    # deceleration assumed for the free distance ahead [m/s²]
    margin::Float64 = 0.5     # distance to keep from a wall straight ahead [m]
end

"""
    target_speed(cfg::SpeedConfig, lane::LaneEstimate) -> v

The cruise speed, lowered so that the sharpest curve ahead keeps the lateral
acceleration at most `a_lat`, and so that the vehicle can stop within the
free distance straight ahead.
"""
function target_speed(cfg::SpeedConfig, lane::LaneEstimate)
    v = curvature_speed(lane.κ_ahead; v_max=cfg.cruise, a_lat=cfg.a_lat, v_min=cfg.v_min)
    v_clear = sqrt(cfg.v_min^2 + 2cfg.a_brake * max(lane.clearance - cfg.margin, 0.0))
    return clamp(min(v, v_clear), cfg.v_min, cfg.cruise)
end
