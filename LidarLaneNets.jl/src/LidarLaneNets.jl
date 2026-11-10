"""
    LidarLaneNets

Lane estimation from a 2D LiDAR scan with neural networks: for each
estimate (see [`ESTIMATES`](@ref)), a small network that runs on the car and
a large one that runs in the cloud.

- [`EstimatorNet`](@ref), [`predict`](@ref), [`load_nets`](@ref): the
  networks.
- [`TieredEstimator`](@ref), [`observe!`](@ref): runs the small networks
  on every scan, calls the cloud's large ones (with a simulated network
  delay) for the estimates a [`CloudSchedule`](@ref) picks, and checks
  which results to use.
- [`LaneTracker`](@ref), [`track!`](@ref): combines the results over time
  into a [`LaneState`](@ref), with each result counted at the time of its
  scan.

Everything is relative to the lane center, at the front axle, in the car's
frame: x forward, y to the left, angles counterclockwise.
"""
module LidarLaneNets

using JLD2
using LinearAlgebra
using Lux
using Random
using Statistics

export PROFILE_STEP, PROFILE_LENGTH, PROFILE_S, CLEARANCE_K, RANGE_MAX, SIGHT_MAX, CLEARANCE_MAX, WIDTH_MAX
export ESTIMATES, output_size, LaneLabels, lane_labels, w_left, w_right
export TIERS, EstimatorNet, build_model, large_from_small, scan_features, scan_features!, input_size, predict
export save_net, load_net, load_nets, weights_file
export Measurement, SelectorConfig, TieredEstimator, observe!, set_schedule!
export CloudSchedule, cloud_calls, NoCalls, AllCalls, FixedCalls, RoundRobin
export TrackerConfig, LaneTracker, LaneState, track!

include("estimates.jl")
include("nets.jl")
include("tiers.jl")
include("tracker.jl")

end # module
