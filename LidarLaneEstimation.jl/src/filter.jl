# Smoothing over time: an unscented Kalman filter
#
# Each scan's estimate is noisy (a few degrees of heading error). A
# controller acting on it directly makes the vehicle twitch, more so at
# speed, since the sideways jolt grows with v². Plain averaging over scans
# would delay turns. Instead a Kalman filter tracks the lane state
# x = (e, ψe, κ) with two steps per scan:
#
#   predict  how x changes until this scan, from the vehicle's speed v and
#            yaw rate r, which are measured and far less noisy than the scan:
#              de/dt  = v⋅sin(ψe) − d⋅r⋅cos(ψe)
#            (d: the reference point's distance ahead of the rear axle,
#            which moves sideways at d⋅r when the rear wheels don't slip)
#              dψe/dt = κ⋅v − r
#              dκ/dt  = 0                        (changes count as noise)
#   correct  move part of the way toward the scan. The filter works out how
#            far from how uncertain the prediction is compared with the scan.
#
# The prediction is nonlinear (the sines and cosines), so it uses the
# unscented Kalman filter (Wan and van der Merwe, 2000) from KalmanFilters.jl:
# a few "sigma points" spread around the estimate are pushed through the
# model, and their spread after it gives the new uncertainty.
#
# Turning into a corner shows up in r first, so it is predicted, not delayed.
# A scan far from the prediction (measured in standard deviations) is
# treated as an outlier and ignored. If that keeps happening, the lane really
# changed (for example, a wall came into view) and the filter restarts from
# the scan.
#
# The curvature profile ahead is averaged over scans too. Each point of it
# stays where it is on the ground while the vehicle drives past.

"""
    FilterConfig(; kwargs...)

Noise levels for the Kalman filter. Distances in meters, angles in radians.
"""
Base.@kwdef struct FilterConfig
    # Noise of one scan's estimate (standard deviations). Larger trusts the
    # scans less: smoother, but slower to correct the prediction.
    σ_e::Float64 = 0.03      # [m]
    σ_ψ::Float64 = 0.06      # [rad]
    σ_κ::Float64 = 0.03      # [1/m]
    # What the prediction can't explain, as rates. Larger follows the scans
    # more closely, with less smoothing.
    q_e::Float64 = 0.3       # sideways speed from wheel slip [m/s]
    q_ψ::Float64 = 0.5       # heading rate from slip [rad/s]
    q_κ::Float64 = 0.5       # change of the lane's curvature per meter driven [1/m²]
    # A scan more than this many standard deviations from the prediction is
    # an outlier; after max_outliers in a row the filter restarts from the scan.
    gate::Float64 = 4.0
    max_outliers::Int = 3
    # Weight of each new scan in the curvature profile ahead. Smaller removes
    # more noise (which reads as false curves on straights) but registers a
    # gentle turn later. A point that changes by more than gate_κ [1/m] (a sharp turn
    # coming into view) is taken as it is.
    curve_gain::Float64 = 0.25
    gate_κ::Float64 = 0.15
end

"""
    LaneFilter(ref_offset; cfg=FilterConfig())

Kalman filter over the lane estimates. `ref_offset` is how far the reference
point (where [`LanePerception`](@ref) measures the errors) is ahead of the
rear axle [m]: the wheelbase for the front axle, 0 for the rear axle. Feed
it every scan's estimate with [`track!`](@ref).
"""
mutable struct LaneFilter
    cfg::FilterConfig
    ref_offset::Float64           # reference point, meters ahead of the rear axle
    ready::Bool
    x::Vector{Float64}            # e, ψe, κ (ψe not wrapped, so the sigma points stay continuous)
    P::Matrix{Float64}            # their covariance
    outliers::Int                 # outliers in a row
    profile::Vector{Float64}      # signed curvature ahead, as in LaneEstimate
end

LaneFilter(ref_offset::Real; cfg::FilterConfig=FilterConfig()) = LaneFilter(cfg, ref_offset, false, zeros(3), zeros(3, 3), 0, Float64[])

_scan_noise(c::FilterConfig) = Diagonal([c.σ_e, c.σ_ψ, c.σ_κ] .^ 2)

function _restart!(f::LaneFilter, lane::LaneEstimate)
    f.x = [lane.e, lane.ψe, lane.κ]
    f.P = Matrix(_scan_noise(f.cfg))
    f.profile = copy(lane.κ_profile)
    f.outliers = 0
    f.ready = true
    return f
end

"""
    track!(f::LaneFilter, lane::LaneEstimate, v, yaw_rate, dt) -> LaneEstimate

`lane` combined with the earlier scans. `v` and `yaw_rate` are the
vehicle's speed [m/s] and yaw rate [rad/s] (counterclockwise), and `dt` the
time since the last scan [s].
"""
function track!(f::LaneFilter, lane::LaneEstimate, v, yaw_rate, dt)
    c = f.cfg
    if !f.ready
        lane.walls === :none || _restart!(f, lane)
        return lane
    end
    # predict
    d = f.ref_offset
    motion((e, ψe, κ)) = [e + (v * sin(ψe) - d * yaw_rate * cos(ψe)) * dt, ψe + (κ * v - yaw_rate) * dt, κ]
    Q = Diagonal([c.q_e * dt, c.q_ψ * dt, c.q_κ * abs(v) * dt] .^ 2)
    tu = time_update(f.x, f.P, motion, Q)
    x, P = get_state(tu), get_covariance(tu)
    profile = _shift(f.profile, v * dt)   # the vehicle moved forward along the lane
    if lane.walls !== :none               # correct (with no lane in view, keep the prediction)
        # the scan measures x directly; its heading is unwrapped to be near the prediction
        z = [lane.e, x[2] + _wrap(lane.ψe - x[2]), lane.κ]
        mu = measurement_update(x, P, z, Matrix(1.0I, 3, 3), _scan_noise(c))
        if calc_nis(mu) > c.gate^2        # squared distance in standard deviations
            f.outliers += 1
            if f.outliers >= c.max_outliers
                _restart!(f, lane)
                return lane
            end
        else
            f.outliers = 0
            x, P = get_state(mu), get_covariance(mu)
        end
        for j in eachindex(profile)
            z = lane.κ_profile[j]
            isnan(z) && continue                         # not in this scan: keep the old value
            profile[j] = isnan(profile[j]) || abs(z - profile[j]) > c.gate_κ ? z :
                         profile[j] + c.curve_gain * (z - profile[j])
        end
    end
    f.x, f.P, f.profile = x, P, profile
    ψe = _wrap(x[2])
    return LaneEstimate(x[1], ψe, x[3], _sharpest(profile, x[3]), lane.width, lane.walls, profile, lane.sight)
end

# A profile sampled every PROFILE_STEP meters, moved d meters toward the vehicle
# (linear interpolation; NaN where nothing was known).
function _shift(profile, d)
    n = length(profile)
    out = fill(NaN, n)
    for j in 1:n
        u = 1 + (j - 1) + d / PROFILE_STEP   # position in the old profile
        i = floor(Int, u)
        i > n && break
        a, b = profile[i], profile[min(i + 1, n)]
        out[j] = isnan(a) ? b : isnan(b) ? a : a + (u - i) * (b - a)
    end
    return out
end
