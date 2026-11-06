"""
    LidarLaneEstimation

Finds the lane between two walls in a 2D LiDAR scan and where the vehicle
is in it, for any lane-keeping or path-tracking controller:

- the lane's center line ahead, as points in the vehicle frame;
- the cross-track error `e`, heading error `ψe` and curvature `κ` at a
  reference point you choose (for example the front axle for the Stanley
  law, the rear axle for pure pursuit);
- the sharpest curvature ahead and the free distance straight ahead, for
  choosing a speed.

[`estimate_lane!`](@ref) works on one scan ([`LanePerception`](@ref)), and
[`track!`](@ref) combines the scans over time with an unscented Kalman filter
([`LaneFilter`](@ref)), which removes most of the scan noise.

Everything is in the vehicle frame: x forward, y to the left, angles
counterclockwise.
"""
module LidarLaneEstimation

using KalmanFilters
using LinearAlgebra
using PrecompileTools

export PerceptionConfig, LanePerception, LaneEstimate, estimate_lane!
export FilterConfig, LaneFilter, track!

include("geometry.jl")
include("perception.jl")
include("filter.jl")
include("precompile.jl")

end # module
