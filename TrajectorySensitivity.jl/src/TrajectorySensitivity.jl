"""
    TrajectorySensitivity

How much each lane estimate's errors and delays move an F1TENTH car's
trajectory, through the whole pipeline (LidarLaneNets.jl's selector and
Kalman filter, LidarRacer.jl's planners and controller), with the networks
replaced by an oracle: the true estimates, plus errors drawn the way the
networks err, delivered with the delay of the car's or the cloud's
networks. One estimate is changed at a time; the others stay true.

- [`NetworkErrors`](@ref), [`ErrorDraws`](@ref), [`load_errors`](@ref):
  how a network errs, as a model to draw from;
- [`Oracle`](@ref), [`OracleNet`](@ref): stand-ins for the networks;
- [`Perturbation`](@ref), [`reference`](@ref), [`accuracy`](@ref),
  [`latency`](@ref), [`oracle_racer`](@ref): the experiments, and the
  controller each one runs;
- [`TrajectoryLog`](@ref), [`compare`](@ref), [`tube`](@ref): the
  trajectory, against the reference;
- [`TrackDesign`](@ref), [`build_track`](@ref): a track built from
  segments, as a map and centerline;
- [`Zonotope`](@ref), [`discretize`](@ref), [`bias_set`](@ref),
  [`disturbance_set`](@ref): the closed loop linearized about one
  situation, and the sets a bias or a late estimate lets it reach.
"""
module TrajectorySensitivity

using LinearAlgebra
using Random
using SpecialFunctions: erf, erfc, erfcinv
using TOML
using LidarLaneNets
using LidarRacer
using StanleyController: Path, tracking_error

export SCORES, from_score, to_score, GaussProcess, correlation, NetworkErrors, ErrorDraws, draw!, load_errors, typical_σ,
    situation, err_factor, σ_factor
export Oracle, OracleNet, set_truth!
export Perturbation, reference, accuracy, latency, describe, oracle_racer
export TrajectoryLog, log!, distance_driven, along, Comparison, compare, tube, by_segment
export Piece, straight, bends, corner, spiral, TrackDesign, BuiltTrack, build_track, track_problems, track_map,
    write_track, read_segments
export Zonotope, box, linear_map, minkowski_sum, interval_hull, widths, jacobian, discretize, stable, bias_set,
    disturbance_set

include("errors.jl")
include("oracle.jl")
include("experiments.jl")
include("trajectory.jl")
include("track.jl")
include("linear.jl")

end # module
