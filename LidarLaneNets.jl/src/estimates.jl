# What the networks estimate, and how each estimate is laid out as numbers.
#
# Every estimate comes from one scan alone and describes the lane relative to
# its center line, at the front axle, in the car's frame (x forward, y left,
# angles counterclockwise). A network's output is a vector; `ESTIMATES` names
# them and `output_size` gives each length.

"""
    PROFILE_STEP, PROFILE_LENGTH, PROFILE_S

`κ_profile`, `w_left` and `w_right` are sampled every `PROFILE_STEP` meters
along the lane center, from the point beside the front axle to
`PROFILE_LENGTH` meters ahead: entry `j` is `PROFILE_S[j]` meters ahead.
"""
const PROFILE_STEP = 0.25
const PROFILE_LENGTH = 8.0
const PROFILE_S = 0.0:PROFILE_STEP:PROFILE_LENGTH

"""
    CLEARANCE_K

The curvatures [1/m] of the fan of arcs along which `clearance` is measured
(`> 0` turning left). They span the car's tightest turn (about 1.17 1/m for
NUC5) on both sides.
"""
const CLEARANCE_K = range(-1.2, 1.2; length=13)

# Ranges past this are treated as "nothing here" [m] (the LiDAR's detection
# range), and caps on the estimates that can be unbounded [m].
const RANGE_MAX = 10.0
const SIGHT_MAX = 10.0
const CLEARANCE_MAX = 10.0
const WIDTH_MAX = 5.0

"""
    ESTIMATES

The estimates, each made by its own networks:

- `:e`: cross-track error [m], the sideways distance from the lane center to
  the front axle; `> 0` when the center is to the left.
- `:ψe`: heading error [rad], the lane's direction minus the car's heading.
- `:κ_profile`: the lane center's curvature [1/m] at `PROFILE_S`; the first
  entry is the curvature beside the car.
- `:widths`: the free room [m] from the lane center to each side at
  `PROFILE_S`: `w_left` then `w_right`.
- `:sight`: how far ahead along the lane center it is seen [m], at most
  `SIGHT_MAX`.
- `:clearance`: the free distance [m] ahead along each arc of `CLEARANCE_K`
  that leaves the front axle straight ahead, at most `CLEARANCE_MAX`.
"""
const ESTIMATES = (:e, :ψe, :κ_profile, :widths, :sight, :clearance)

const _NP = length(PROFILE_S)

# How many numbers each estimate is, and its name in file names (without Unicode).
const _SIZES = (e=1, ψe=1, κ_profile=_NP, widths=2_NP, sight=1, clearance=length(CLEARANCE_K))
const _FILE_NAMES = (e="e", ψe="psi_e", κ_profile="kappa_profile", widths="widths", sight="sight",
    clearance="clearance")

"""
    UNITS

The unit of each estimate, as `(e="m", ψe="rad", …)`.
"""
const UNITS = (e="m", ψe="rad", κ_profile="1/m", widths="m", sight="m", clearance="m")

_check(est::Symbol) = est in ESTIMATES ? est : throw(ArgumentError("unknown estimate $est"))

"""
    output_size(estimate) -> n

How many numbers an estimate is.
"""
output_size(est::Symbol) = _SIZES[_check(est)]

"""
    file_name(estimate) -> String

The estimate's name in file names and in `fusion.toml` (no Unicode): `"psi_e"` for `:ψe`, say.
"""
file_name(est::Symbol) = _FILE_NAMES[_check(est)]

"""
    LaneLabels

One scan's estimates (or their true values): a vector per estimate in
`ESTIMATES`, keyed by name.
"""
const LaneLabels = NamedTuple{ESTIMATES,NTuple{6,Vector{Float64}}}

"""
    lane_labels(; e, ψe, κ_profile, w_left, w_right, sight, clearance) -> LaneLabels

Pack the estimates into their vectors.
"""
lane_labels(; e, ψe, κ_profile, w_left, w_right, sight, clearance) =
    LaneLabels(([Float64(e)], [Float64(ψe)], collect(Float64, κ_profile),
        vcat(collect(Float64, w_left), collect(Float64, w_right)), [Float64(sight)], collect(Float64, clearance)))

w_left(widths::AbstractVector) = widths[1:_NP]
w_right(widths::AbstractVector) = widths[_NP+1:end]
