# The estimates that come from no network, shared by evaluate.jl, fusion.jl
# and LidarRacer.jl's race scripts:
#
# - :hand, the hand-written estimator (LidarLaneEstimation.jl), every
#   estimate, as LidarRacer used it on `main`;
# - :geometric, widths, sight and clearance measured in the scan
#   (LidarLaneNets.ScanGeometry), along the lane center the car's networks
#   give.
#
# Each needs LidarLaneNets, LidarLaneEstimation and StanleyController loaded.

import LidarLaneEstimation as LLE
using LidarLaneNets
using StanleyController: Path, tracking_error

# NUC5 and its LiDAR, as in F1Tenth.jl: 1081 beams over 270°, the front axle
# 0.013 m behind the LiDAR, the car 0.228 m wide.
const BEAMS = range(-3π / 4, 3π / 4; length=1081)
const X_REF = 0.130 - 0.143
const HALFWIDTH = 0.228 / 2

const SOURCES = (:hand, :geometric)
source_estimates(s::Symbol) = s === :hand ? ESTIMATES : s === :geometric ? GEOMETRIC_ESTIMATES :
                              throw(ArgumentError("unknown source $s"))

# The hand-written estimator as LidarRacer used it, with its curvature
# profile reaching as far as the networks'.
hand_config() = LLE.PerceptionConfig(car_halfwidth=HALFWIDTH, r_max=10.0, preview=PROFILE_LENGTH, jump=0.8)
hand_perception(beams=BEAMS, x_ref=X_REF) = LLE.LanePerception(beams, x_ref; cfg=hand_config())

"""
    hand_estimate!(p, ranges) -> Dict{Symbol,Vector{Float64}}

The hand-written estimator's estimates for one scan (it keeps a running
lane width, so give it the scans in order): every estimate, or only
`clearance` when it finds no lane. Profiles are NaN where it gives nothing.
"""
function hand_estimate!(p::LLE.LanePerception, ranges)
    np = length(PROFILE_S)
    out = Dict{Symbol,Vector{Float64}}()
    est = LLE.estimate_lane!(p, ranges)
    # (clearance measures the scan last given to estimate_lane!)
    out[:clearance] = [min(LLE.clearance(p, κ), CLEARANCE_MAX) for κ in CLEARANCE_K]
    est.walls === :none && return out
    out[:e], out[:ψe] = [est.e], [est.ψe]
    out[:κ_profile] = copy(est.κ_profile)
    out[:sight] = [min(est.sight, SIGHT_MAX)]
    # the room to each side, at the points beside and ahead of the front axle
    widths = fill(NaN, 2np)
    m = 0.01
    c = LLE.lane_corridor(p, m; step=0.2)
    if c !== nothing
        s = vcat(0.0, cumsum(hypot.(diff(c.x), diff(c.y))))
        s_ref = tracking_error(Path(c.x, c.y), p.x_ref, 0.0, 0.0).s
        for (k, d) in enumerate(PROFILE_S)
            u = s_ref + d
            (0 <= u <= s[end]) || continue
            i = clamp(searchsortedlast(s, u), 1, length(s) - 1)
            f = (u - s[i]) / (s[i+1] - s[i])
            widths[k] = m + (1 - f) * c.upper[i] + f * c.upper[i+1]
            widths[np+k] = m - ((1 - f) * c.lower[i] + f * c.lower[i+1])
        end
    end
    out[:widths] = widths
    return out
end

# A matrix per estimate, NaN where the source gives nothing.
_blank(ests, N) = Dict(est => fill(NaN, output_size(est), N) for est in ests)

"""
    hand_estimates(X) -> (Dict of (n_out, N), share of scans without a lane)

The hand-written estimator's estimates for every scan in `X` (in order).
"""
function hand_estimates(X)
    p = hand_perception()
    N = size(X, 2)
    out = _blank(ESTIMATES, N)
    none = 0
    for j in 1:N
        r = hand_estimate!(p, Float64.(view(X, :, j)))
        haskey(r, :e) || (none += 1)
        for (est, v) in r
            out[est][:, j] .= v
        end
    end
    return out, none / N
end

"""
    geometric_estimates(X, e, ψe, κ_profile) -> Dict of (n_out, N)

Widths, sight and clearance measured in every scan of `X`, along the lane
center given by `e`, `ψe` (`(1, N)`) and `κ_profile` (`(NP, N)`): the car
networks' results, say.
"""
function geometric_estimates(X, e, ψe, κ_profile)
    g = ScanGeometry(BEAMS, X_REF; hw=HALFWIDTH)
    N = size(X, 2)
    out = _blank(GEOMETRIC_ESTIMATES, N)
    Threads.@threads for j in 1:N
        ranges = view(X, :, j)
        meas = [Measurement(:e, 0.0, :small, [e[1, j]], [0.0]), Measurement(:ψe, 0.0, :small, [ψe[1, j]], [0.0]),
            Measurement(:κ_profile, 0.0, :small, κ_profile[:, j], zeros(size(κ_profile, 1)))]
        for (est, v) in g(ranges, meas)
            out[est][:, j] .= v
        end
    end
    return out
end
