# The experiments: one estimate changed at a time, and the controller each
# one runs.

"""
    Perturbation

One experiment. `kind` is

- `:none`: every estimate true and on time (the reference);
- `:accuracy`: `estimate` has the errors of the network set `network`,
  times `gain`, drawn from `seed`, and comes from the car at every scan,
  without delay;
- `:latency`: `estimate` is true, but comes only from the cloud, `delay`
  seconds after its scan, as LidarLaneNets.jl's selector handles late
  results (dropped past its `max_age`);
- `:cloud`: `estimate` with the car network's errors from the car at every
  scan, and with the cloud network's errors (from the same recorded scans)
  from the cloud, `delay` seconds after its scan: calling the cloud for it;
- `:network`: `estimate` from the real network of the set `network`, from
  the car at every scan (no oracle for it), `seed` the LiDAR noise's.

Errors come from `source`: `:replay` (the networks' recorded errors,
[`ReplayDraws`](@ref)) or `:parametric` (the model, [`ErrorDraws`](@ref)).
The other estimates are true and come from the car at every scan.
"""
struct Perturbation
    kind::Symbol
    estimate::Union{Nothing,Symbol}
    gain::Float64
    network::Symbol
    delay::Float64
    seed::Int
    source::Symbol
end

"""
    reference() -> Perturbation

Every estimate true and on time.
"""
reference() = Perturbation(:none, nothing, 0.0, :small_weak, 0.0, 0, :none)

"""
    accuracy(estimate, gain; network=:small_weak, seed=1, source=:replay) -> Perturbation

`estimate` with the errors of `network`, times `gain`.
"""
accuracy(est::Symbol, gain::Real; network::Symbol=:small_weak, seed::Integer=1, source::Symbol=:replay) =
    Perturbation(:accuracy, _check_est(est), Float64(gain), network, 0.0, seed, _check_source(source))

"""
    latency(estimate, delay) -> Perturbation

`estimate` true, but only from the cloud, `delay` seconds after its scan.
"""
latency(est::Symbol, delay::Real) = Perturbation(:latency, _check_est(est), 0.0, :large, Float64(delay), 0, :none)

"""
    cloud_call(estimate, delay; seed=1) -> Perturbation

`estimate` from the car's network and from the cloud's, `delay` seconds
late, both with their recorded errors.
"""
cloud_call(est::Symbol, delay::Real; seed::Integer=1) =
    Perturbation(:cloud, _check_est(est), 1.0, :large, Float64(delay), seed, :replay)

"""
    real_network(estimate; network=:small_weak, seed=1) -> Perturbation

`estimate` from the real network of `network`, on the car, the LiDAR's
noise drawn from `seed`.
"""
real_network(est::Symbol; network::Symbol=:small_weak, seed::Integer=1) =
    Perturbation(:network, _check_est(est), 1.0, network, 0.0, seed, :network)

_check_est(est) = est in ESTIMATES ? est : throw(ArgumentError("unknown estimate $est"))
_check_source(s) = s in (:replay, :parametric) ? s : throw(ArgumentError("unknown error source $s"))

"""
    describe(p::Perturbation) -> String

A short name for the experiment, such as `"accuracy ψe ×2 (small_weak, seed 3)"`.
"""
function describe(p::Perturbation)
    p.kind === :none && return "reference"
    p.kind === :accuracy && return "accuracy $(p.estimate) ×$(p.gain) ($(p.network), $(p.source), seed $(p.seed))"
    p.kind === :cloud && return "cloud $(p.estimate) $(round(1000p.delay; digits=1)) ms (seed $(p.seed))"
    p.kind === :network && return "network $(p.estimate) ($(p.network), seed $(p.seed))"
    return "latency $(p.estimate) $(round(1000p.delay; digits=1)) ms"
end

"""
    oracle_racer(car, beams, p::Perturbation, models; library=nothing, nets=nothing, car_set=:small_weak,
                 cloud_set=:large, sure=0.01, selector=SelectorConfig(), tracking=TrackerConfig(), racer...)
        -> (racer, oracle)

The controller (a LidarRacer `Racer`, everything as on the car) for the
experiment `p`, its networks replaced by [`OracleNet`](@ref)s of one
[`Oracle`](@ref). `models` holds the error models of each network set
(`models[set]`, as from [`load_errors`](@ref)): `car_set` the car's,
`cloud_set` the cloud's, and `p.network`; `library` the recorded errors
(an [`ErrorLibrary`](@ref), for `:replay` and `:cloud`); `nets` the real
networks (`nets[set]`, as from LidarLaneNets' `load_nets`, for `:network`).

What each network tells the Kalman filter is what a network making those
errors would: its own σ per scan and, in the filter's noise parameters,
the persisting error that train/fusion.jl would measure for it (both
scaled by the gain). A true estimate comes with `sure` times its network's
typical σ (and bias): a network that is right and knows it. `selector`,
`tracking` and `racer` (`stanley`, `margin`, `k_σ`, `latency`, `v_max`)
replace the controller's defaults, except what the experiment sets: the
filter's noise parameters, the cloud schedule, and the cloud's delay.
"""
function oracle_racer(car, beams, p::Perturbation, models; library=nothing, nets=nothing, car_set::Symbol=:small_weak,
                      cloud_set::Symbol=:large, sure::Real=0.01, selector::SelectorConfig=SelectorConfig(),
                      tracking::TrackerConfig=TrackerConfig(), racer...)
    oracle = Oracle()
    car_m, cloud_m = models[car_set], models[cloud_set]
    changed(est) = p.kind !== :none && est === p.estimate
    erring = p.kind in (:accuracy, :cloud, :network)   # the changed estimate has a network's errors
    # the changed estimate's errors: recorded (shared by the car's and the cloud's stand-ins) or modeled
    draws = !(p.kind in (:accuracy, :cloud)) ? nothing :
            p.source === :replay ? ReplayDraws(_need(library, "library"), p.estimate, Xoshiro(p.seed)) :
            ErrorDraws(models[p.network][p.estimate], Xoshiro(p.seed))
    role(set) = set === cloud_set ? :cloud : :car
    small = NamedTuple{ESTIMATES}(Tuple(begin
        if changed(est) && p.kind === :network
            _need(nets, "nets")[p.network][est]
        elseif changed(est) && p.kind === :accuracy
            OracleNet(oracle, est; draws, role=role(p.network), gain=p.gain, σ_min=sure .* typical_σ(models[p.network][est]))
        elseif changed(est) && p.kind === :cloud
            OracleNet(oracle, est; draws, role=:car, σ_min=sure .* typical_σ(car_m[est]))
        else
            OracleNet(oracle, est; σ_min=sure .* typical_σ(car_m[est]), silent=changed(est) && p.kind === :latency)
        end
    end for est in ESTIMATES))
    large = NamedTuple{ESTIMATES}(Tuple(begin
        changed(est) && p.kind === :cloud ?
            OracleNet(oracle, est; draws, role=:cloud, σ_min=sure .* typical_σ(cloud_m[est])) :
            OracleNet(oracle, est; σ_min=sure .* typical_σ(cloud_m[est]))
    end for est in ESTIMATES))
    noise = NamedTuple{ESTIMATES}(Tuple(begin
        acc = changed(est) && erring
        set = acc && p.kind !== :cloud ? p.network : car_set
        f = models[set][est].filter
        g = acc ? max(p.kind === :accuracy ? p.gain : 1.0, sure) : sure
        EstimateNoise(persist=f.persist, τ=f.τ, σ_bias=g * f.σ_bias, ν_small=f.ν_small, ν_large=cloud_m[est].filter.ν_large)
    end for est in ESTIMATES))
    schedule = p.kind in (:latency, :cloud) ? FixedCalls([p.estimate]) : NoCalls()
    sel = p.kind in (:latency, :cloud) ? _with(selector; remote_latency=p.delay) : selector
    racer_ = Racer(car, beams, (small=small, large=large); schedule, selector=sel, tracking=_with(tracking; noise),
        rng=Xoshiro(p.seed), racer...)
    return racer_, oracle
end

"""
    JointAccuracy

Every estimate with the errors of the network set `network`, each times its
own gain (`gains`, by estimate; 0: true), all from the car at every scan,
without delay, and all from the same recorded scans (so they err together
as the networks do). `seed` draws the replay. For variance-based
sensitivity (scripts/interactions.jl): how the estimates' errors add up and
interact.
"""
struct JointAccuracy
    gains::NamedTuple{ESTIMATES,NTuple{6,Float64}}
    network::Symbol
    seed::Int
end

"""
    joint_accuracy(gains; network=:small_weak, seed=1) -> JointAccuracy

`gains`: six gains in the order of `ESTIMATES`, or a NamedTuple by estimate.
"""
joint_accuracy(gains; network::Symbol=:small_weak, seed::Integer=1) =
    JointAccuracy(NamedTuple{ESTIMATES}(Tuple(Float64.(values(gains)))), network, seed)

describe(p::JointAccuracy) = "joint accuracy " * join(("$e ×$(round(g; digits=3))" for (e, g) in pairs(p.gains)), ", ") *
                             " ($(p.network), seed $(p.seed))"

function oracle_racer(car, beams, p::JointAccuracy, models; library=nothing, car_set::Symbol=:small_weak,
                      cloud_set::Symbol=:large, sure::Real=0.01, selector::SelectorConfig=SelectorConfig(),
                      tracking::TrackerConfig=TrackerConfig(), nets=nothing, racer...)
    oracle = Oracle()
    m, cloud_m = models[p.network], models[cloud_set]
    # one replay for all six: their errors come from the same recorded scans
    draws = ReplayDraws(_need(library, "library"), first(ESTIMATES), Xoshiro(p.seed))
    role = p.network === cloud_set ? :cloud : :car
    small = NamedTuple{ESTIMATES}(Tuple(
        OracleNet(oracle, est; draws, role, gain=p.gains[est], σ_min=sure .* typical_σ(m[est])) for est in ESTIMATES))
    large = NamedTuple{ESTIMATES}(Tuple(
        OracleNet(oracle, est; σ_min=sure .* typical_σ(cloud_m[est])) for est in ESTIMATES))
    noise = NamedTuple{ESTIMATES}(Tuple(begin
        f = m[est].filter
        EstimateNoise(persist=f.persist, τ=f.τ, σ_bias=max(p.gains[est], sure) * f.σ_bias, ν_small=f.ν_small,
            ν_large=cloud_m[est].filter.ν_large)
    end for est in ESTIMATES))
    racer_ = Racer(car, beams, (small=small, large=large); schedule=NoCalls(), selector, tracking=_with(tracking; noise),
        rng=Xoshiro(p.seed), racer...)
    return racer_, oracle
end

_need(x, what) = x === nothing ? throw(ArgumentError("this experiment needs `$what`")) : x

# A copy of a keyword-constructed struct with some fields replaced.
_with(x::T; kw...) where {T} = T(; (f => get(kw, f, getfield(x, f)) for f in fieldnames(T))...)
