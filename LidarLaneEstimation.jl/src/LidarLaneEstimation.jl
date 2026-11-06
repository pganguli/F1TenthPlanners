"""
    LidarLaneEstimation

Finds the lane between two walls in a 2D LiDAR scan and tells a path-tracking
controller where the vehicle is in it: the cross-track error `e`, heading
error `ψe` and curvature `κ` at the front axle (the inputs of the Stanley
law; see StanleyController.jl), plus the curvature ahead and the free
distance straight ahead, for choosing a speed.

- [`estimate_lane!`](@ref) works on one scan ([`LanePerception`](@ref)).
- [`track!`](@ref) combines the scans over time with an unscented Kalman
  filter ([`LaneFilter`](@ref)), which removes most of the scan noise.
- [`target_speed`](@ref) picks a speed from the result ([`SpeedConfig`](@ref)).

Everything is in the vehicle frame: x forward, y to the left, angles
counterclockwise. Nothing depends on a particular vehicle or simulator.
"""
module LidarLaneEstimation

using KalmanFilters
using LinearAlgebra
using PrecompileTools
using StanleyController

export PerceptionConfig, LanePerception, LaneEstimate, estimate_lane!
export FilterConfig, LaneFilter, track!
export SpeedConfig, target_speed

include("perception.jl")
include("filter.jl")
include("speed.jl")
include("precompile.jl")

end # module
