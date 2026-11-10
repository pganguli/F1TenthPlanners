# The two tiers, when to call the cloud, and which results to use.
#
# Every scan, the small networks run on the car and give every estimate. A
# schedule (see `CloudSchedule`) picks which estimates, if any, to also ask
# the cloud's large networks for; each call is a cost. Their results come
# back `remote_latency` seconds later (or never, with probability
# `remote_drop`) and replace the small networks' results for that scan in the
# tracker.
#
# Each result is checked before it is used:
# - unsure: its mean standard deviation is above `max_σ`; a small result
#   that is unsure is dropped, so the tracker only predicts that estimate.
# - late: a large result older than `max_age` is dropped.
# - disagreeing (off by default): a large result too far from the small one
#   for the same scan (more than `agree` combined standard deviations, on the
#   median entry) is dropped. The tracker uses such disagreement to learn the
#   small network's bias, so it is kept unless `agree` is set.

"""
    Measurement

One estimate from one scan: which (`estimate`), from when (`scan_time`, the
scan's time [s]), from which tier, and its value and standard deviation
(vectors, as in [`ESTIMATES`](@ref)).
"""
struct Measurement
    estimate::Symbol
    scan_time::Float64
    tier::Symbol
    value::Vector{Float64}
    σ::Vector{Float64}
end

# ---------------------------------------------------------------------------
# When to call the cloud

"""
    CloudSchedule

Decides, scan by scan, which estimates to ask the cloud for. A schedule is
any subtype with a method

    cloud_calls(schedule, ctx) -> collection of estimate names

where `ctx` is `(; t, lane, v, yaw_rate)`: the scan's time [s], the last
[`LaneState`](@ref) (`nothing` before the first) and the car's speed [m/s]
and yaw rate [rad/s]. Given: [`NoCalls`](@ref), [`AllCalls`](@ref),
[`FixedCalls`](@ref), [`RoundRobin`](@ref). An online, sensitivity-based
schedule plugs in the same way.
"""
abstract type CloudSchedule end

"""
    cloud_calls(schedule, ctx) -> collection of estimate names

The estimates to ask the cloud for at this scan (see [`CloudSchedule`](@ref)).
"""
function cloud_calls end

"""
    NoCalls()

Never call the cloud: the small networks alone.
"""
struct NoCalls <: CloudSchedule end
cloud_calls(::NoCalls, ctx) = ()

"""
    AllCalls()

Ask the cloud for every estimate at every scan.
"""
struct AllCalls <: CloudSchedule end
cloud_calls(::AllCalls, ctx) = ESTIMATES

"""
    FixedCalls(estimates...)

Ask the cloud for the same estimates at every scan.
"""
struct FixedCalls <: CloudSchedule
    estimates::Vector{Symbol}
end
FixedCalls(ests::Symbol...) = FixedCalls(collect(ests))
cloud_calls(s::FixedCalls, ctx) = s.estimates

"""
    RoundRobin(estimates=ESTIMATES; per_scan=1)

Ask the cloud for `per_scan` estimates per scan, taking `estimates` in turn.
"""
mutable struct RoundRobin <: CloudSchedule
    estimates::Vector{Symbol}
    per_scan::Int
    next::Int
end
RoundRobin(ests=ESTIMATES; per_scan::Integer=1) = RoundRobin(collect(Symbol, ests), per_scan, 1)

function cloud_calls(s::RoundRobin, ctx)
    n = length(s.estimates)
    n == 0 && return Symbol[]
    out = [s.estimates[mod1(s.next + k, n)] for k in 0:min(s.per_scan, n)-1]
    s.next = mod1(s.next + s.per_scan, n)
    return out
end

# ---------------------------------------------------------------------------
# The selector

"""
    SelectorConfig(; kwargs...)

Which results the selector uses. Times in seconds.
"""
Base.@kwdef struct SelectorConfig
    # From a scan to the cloud's results arriving: the uplink, the inference
    # and queueing, the downlink. One scan period at 40 Hz.
    remote_latency::Float64 = 0.025
    remote_drop::Float64 = 0.0           # chance a call's results never arrive
    max_age::Float64 = 0.1               # cloud results older than this are not used
    # An estimate whose mean standard deviation is above this is "unsure"
    # (units as the estimate's).
    max_σ::NamedTuple = (e=0.3, ψe=0.3, κ_profile=0.5, widths=1.0, sight=3.0, clearance=3.0)
    agree::Float64 = Inf
    # Whether the small networks' results are used at all. Without them, an
    # estimate is only predicted between cloud results: a car running on the
    # cloud alone (with `AllCalls`).
    use_small::Bool = true
end

"""
    TieredEstimator(nets; schedule=RoundRobin(), cfg=SelectorConfig(), rng=Random.default_rng())

Both tiers of networks (as from [`load_nets`](@ref); without `nets.large`,
the cloud is never called) and the selector. Feed it every scan with
[`observe!`](@ref). `calls` counts the cloud calls made for each estimate.
"""
mutable struct TieredEstimator{N}
    nets::N
    schedule::CloudSchedule
    cfg::SelectorConfig
    rng::AbstractRNG
    pending::Vector{Tuple{Float64,Measurement}}   # (arrival time, result), in arrival order
    recent_small::Dict{Tuple{Symbol,Float64},Measurement}   # small results by (estimate, scan time)
    calls::Dict{Symbol,Int}
end

TieredEstimator(nets; schedule::CloudSchedule=RoundRobin(), cfg::SelectorConfig=SelectorConfig(),
                rng::AbstractRNG=Random.default_rng()) =
    TieredEstimator(nets, schedule, cfg, rng, Tuple{Float64,Measurement}[], Dict{Tuple{Symbol,Float64},Measurement}(),
        Dict(est => 0 for est in ESTIMATES))

"""
    set_schedule!(te, schedule) -> te

Change when the cloud is called, from the next scan on.
"""
set_schedule!(te::TieredEstimator, s::CloudSchedule) = (te.schedule = s; te)

_has_large(te::TieredEstimator) = hasproperty(te.nets, :large)

_unsure(cfg::SelectorConfig, m::Measurement) = sum(m.σ) / length(m.σ) > cfg.max_σ[m.estimate]

function _agrees(cfg::SelectorConfig, large::Measurement, small::Measurement)
    z = [abs(large.value[i] - small.value[i]) / sqrt(large.σ[i]^2 + small.σ[i]^2) for i in eachindex(large.value)]
    return median(z) <= cfg.agree
end

"""
    observe!(te, ranges, t; lane=nothing, v=0.0, yaw_rate=0.0) -> Vector{Measurement}

Run the small networks on the scan `ranges` taken at time `t` [s], call the
cloud for what the schedule asks (given the last lane state `lane` and the
car's speed and yaw rate), and return the measurements to use now: the
small networks' for this scan, and any cloud results that have arrived by
`t`.
"""
function observe!(te::TieredEstimator, ranges::AbstractVector{<:Real}, t::Real; lane=nothing, v=0.0, yaw_rate=0.0)
    cfg, out = te.cfg, Measurement[]
    t = Float64(t)
    for est in ESTIMATES
        μ, σ = predict(te.nets.small[est], ranges)
        m = Measurement(est, t, :small, μ, σ)
        te.recent_small[(est, t)] = m
        cfg.use_small && !_unsure(cfg, m) && push!(out, m)
    end
    # forget small results too old for any cloud result to be compared with
    filter!(kv -> t - kv.first[2] <= cfg.max_age + 1e-9, te.recent_small)

    if _has_large(te)
        calls = cloud_calls(te.schedule, (; t, lane, v, yaw_rate))
        arrives = rand(te.rng) >= cfg.remote_drop
        for est in calls
            te.calls[est] += 1
            arrives || continue
            μ, σ = predict(te.nets.large[est], ranges)
            push!(te.pending, (t + cfg.remote_latency, Measurement(est, t, :large, μ, σ)))
        end
        while !isempty(te.pending) && te.pending[1][1] <= t + 1e-9
            _, m = popfirst!(te.pending)
            t - m.scan_time <= cfg.max_age + 1e-9 || continue
            _unsure(cfg, m) && continue
            small = get(te.recent_small, (m.estimate, m.scan_time), nothing)
            (!cfg.use_small || small === nothing || _agrees(cfg, m, small)) || continue
            push!(out, m)
        end
    end
    return out
end
