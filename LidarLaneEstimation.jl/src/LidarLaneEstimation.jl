"""
    LidarLaneEstimation

Finds the lane between two walls in a 2D LiDAR scan and where the vehicle
is in it, for any lane-keeping or path-tracking controller:

- the lane's center line ahead, as points in the vehicle frame;
- the free corridor around it, for a path planner;
- the cross-track error `e`, heading error `ψe` and curvature `κ` at a
  reference point you choose (for example the front axle for the Stanley
  law, the rear axle for pure pursuit), against the center or a planned
  path;
- the curvature ahead, how far ahead the line is seen, and the free
  distance ahead along any arc, for choosing a speed.

Per scan ([`LanePerception`](@ref)): [`detect_lane!`](@ref) finds the lane,
[`lane_corridor`](@ref) gives the room around its center,
[`measure!`](@ref) measures the vehicle against a line, and
[`clearance`](@ref) gives the free distance ahead along an arc;
[`estimate_lane!`](@ref) does `detect_lane!` then `measure!` against the
center.
[`track!`](@ref) combines the scans over time with an unscented Kalman filter
([`LaneFilter`](@ref)), which removes most of the scan noise.

Everything is in the vehicle frame: x forward, y to the left, angles
counterclockwise.
"""
module LidarLaneEstimation

using KalmanFilters
using LinearAlgebra
using PrecompileTools

export PerceptionConfig, LanePerception, LaneEstimate, estimate_lane!, detect_lane!, measure!, lane_corridor, clearance, PROFILE_STEP
export FilterConfig, LaneFilter, track!

include("geometry.jl")
include("perception.jl")
include("corridor.jl")
include("filter.jl")
include("precompile.jl")

end # module
