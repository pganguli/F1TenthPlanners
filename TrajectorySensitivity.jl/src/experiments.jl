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
  results (dropped past its `max_age`).

The other estimates are true and come from the car at every scan.
"""
struct Perturbation
    kind::Symbol
    estimate::Union{Nothing,Symbol}
    gain::Float64
    network::Symbol
    delay::Float64
    seed::Int
end

"""
    reference() -> Perturbation

Every estimate true and on time.
"""
reference() = Perturbation(:none, nothing, 0.0, :small_weak, 0.0, 0)

"""
    accuracy(estimate, gain; network=:small_weak, seed=1) -> Perturbation

`estimate` with the errors of `network`, times `gain`.
"""
accuracy(est::Symbol, gain::Real; network::Symbol=:small_weak, seed::Integer=1) =
    Perturbation(:accuracy, _check_est(est), Float64(gain), network, 0.0, seed)

"""
    latency(estimate, delay) -> Perturbation

`estimate` true, but only from the cloud, `delay` seconds after its scan.
"""
latency(est::Symbol, delay::Real) = Perturbation(:latency, _check_est(est), 0.0, :large, Float64(delay), 0)

_check_est(est) = est in ESTIMATES ? est : throw(ArgumentError("unknown estimate $est"))

"""
    describe(p::Perturbation) -> String

A short name for the experiment, such as `"accuracy ψe ×2 (small_weak, seed 3)"`.
"""
function describe(p::Perturbation)
    p.kind === :none && return "reference"
    p.kind === :accuracy && return "accuracy $(p.estimate) ×$(p.gain) ($(p.network), seed $(p.seed))"
    return "latency $(p.estimate) $(round(1000p.delay; digits=1)) ms"
end

"""
    oracle_racer(car, beams, p::Perturbation, models; car_set=:small_weak, cloud_set=:large,
                 sure=0.01, selector=SelectorConfig(), tracking=TrackerConfig(), racer...) -> (racer, oracle)

The controller (a LidarRacer `Racer`, everything as on the car) for the
experiment `p`, its networks replaced by [`OracleNet`](@ref)s of one
[`Oracle`](@ref). `models` holds the error models of each network set
(`models[set]`, as from [`load_errors`](@ref)): `car_set` the car's,
`cloud_set` the cloud's, and `p.network`.

What each network tells the Kalman filter is what a network making those
errors would: its own σ per scan and, in the filter's noise parameters,
the persisting error that train/fusion.jl would measure for it (both
scaled by the gain). A true estimate comes with `sure` times its network's
typical σ (and bias): a network that is right and knows it. `selector`,
`tracking` and `racer` (`stanley`, `margin`, `k_σ`, `latency`, `v_max`)
replace the controller's defaults, except what the experiment sets: the
filter's noise parameters, the cloud schedule, and the cloud's delay.
"""
function oracle_racer(car, beams, p::Perturbation, models; car_set::Symbol=:small_weak, cloud_set::Symbol=:large,
                      sure::Real=0.01, selector::SelectorConfig=SelectorConfig(), tracking::TrackerConfig=TrackerConfig(),
                      racer...)
    oracle = Oracle()
    car_m, cloud_m = models[car_set], models[cloud_set]
    changed(est) = p.kind !== :none && est === p.estimate
    small = NamedTuple{ESTIMATES}(Tuple(begin
        if changed(est) && p.kind === :accuracy
            m = models[p.network][est]
            OracleNet(oracle, est; draws=ErrorDraws(m, Xoshiro(p.seed)), gain=p.gain, σ_min=sure .* typical_σ(m))
        else
            OracleNet(oracle, est; σ_min=sure .* typical_σ(car_m[est]), silent=changed(est) && p.kind === :latency)
        end
    end for est in ESTIMATES))
    large = NamedTuple{ESTIMATES}(Tuple(OracleNet(oracle, est; σ_min=sure .* typical_σ(cloud_m[est]))
                                        for est in ESTIMATES))
    noise = NamedTuple{ESTIMATES}(Tuple(begin
        acc = changed(est) && p.kind === :accuracy
        f = (acc ? models[p.network][est] : car_m[est]).filter
        EstimateNoise(persist=f.persist, τ=f.τ, σ_bias=(acc ? max(p.gain, sure) : sure) * f.σ_bias,
            ν_small=f.ν_small, ν_large=cloud_m[est].filter.ν_large)
    end for est in ESTIMATES))
    schedule = p.kind === :latency ? FixedCalls([p.estimate]) : NoCalls()
    sel = p.kind === :latency ? _with(selector; remote_latency=p.delay) : selector
    racer_ = Racer(car, beams, (small=small, large=large); schedule, selector=sel, tracking=_with(tracking; noise),
        rng=Xoshiro(p.seed), racer...)
    return racer_, oracle
end

# A copy of a keyword-constructed struct with some fields replaced.
_with(x::T; kw...) where {T} = T(; (f => get(kw, f, getfield(x, f)) for f in fieldnames(T))...)
