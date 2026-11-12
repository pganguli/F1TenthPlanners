# Stand-ins for the networks: the true estimates, with a network's errors or
# none, in the shape LidarLaneNets.jl's selector takes networks in.

"""
    Oracle()

What the networks' stand-ins know: the true estimates of the scan being
processed and its time. The simulation sets them with [`set_truth!`](@ref)
before the controller sees the scan.
"""
mutable struct Oracle
    labels::Union{Nothing,LaneLabels}
    t::Float64
end

Oracle() = Oracle(nothing, 0.0)

"""
    set_truth!(oracle, labels, t) -> oracle

The true estimates (a `LaneLabels`) of the scan taken at time `t` [s].
"""
set_truth!(o::Oracle, labels::LaneLabels, t::Real) = (o.labels = labels; o.t = Float64(t); o)

"""
    OracleNet(oracle, estimate; draws=nothing, gain=1.0, σ_min, silent=false)

Stands in for the network of one estimate, wherever LidarLaneNets.jl takes
a network (`predict(net, ranges)`, which ignores the ranges): the
estimate's true value for the oracle's scan, and

- with `draws` (an [`ErrorDraws`](@ref)): plus the network's error times
  `gain`, with its σ times `gain` (at least `σ_min`): what a network `gain`
  times as wrong would give;
- without: σ = `σ_min` (a vector, per entry), a network that is right and
  sure of it;
- `silent`: no result (σ = Inf, which the selector drops as unsure), as
  when that network does not run.
"""
mutable struct OracleNet{D}
    oracle::Oracle
    estimate::Symbol
    draws::D
    gain::Float64
    σ_min::Vector{Float64}
    silent::Bool
    last_t::Float64
end

OracleNet(o::Oracle, est::Symbol; draws=nothing, gain::Real=1.0, σ_min::AbstractVector{<:Real}, silent::Bool=false) =
    OracleNet(o, est, draws, Float64(gain), collect(Float64, σ_min), silent, -Inf)

function LidarLaneNets.predict(n::OracleNet, ::AbstractVector{<:Real})
    o = n.oracle
    o.labels === nothing && throw(ArgumentError("the oracle has no scan: call set_truth! first"))
    truth = o.labels[n.estimate]
    n.silent && return fill(NaN, length(truth)), fill(Inf, length(truth))
    n.draws === nothing && return copy(truth), copy(n.σ_min)
    dt = o.t - n.last_t
    n.last_t = o.t
    err, σ = draw!(n.draws, dt, situation(n.estimate, o.labels))
    return truth .+ n.gain .* err, max.(n.gain .* σ, n.σ_min)
end
