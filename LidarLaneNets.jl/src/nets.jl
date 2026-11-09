# The networks: one per estimate and tier, each a 1D convolutional network
# over the scan that outputs the estimate's mean and standard deviation.

"""
    TIERS

`:small` runs on the car: every 4th beam, three thin convolutions (each
output sees about 29° of the scan). `:large` runs in the cloud: every beam,
six wider convolutions (each output sees about 63°) and a wider head.
"""
const TIERS = (:small, :large)

beam_stride(tier::Symbol) = tier === :small ? 4 : tier === :large ? 1 : throw(ArgumentError("unknown tier $tier"))

input_size(tier::Symbol, num_beams::Integer) = cld(num_beams, beam_stride(tier))

"""
    scan_features(ranges, tier) -> Vector{Float32}

A scan as a network's input: the beams the tier uses, each range clipped to
`RANGE_MAX` (anything out of range or invalid reads as `RANGE_MAX`) and
divided by it.
"""
function scan_features(ranges::AbstractVector{<:Real}, tier::Symbol)
    s = beam_stride(tier)
    return Float32[_feature(ranges[i]) for i in 1:s:length(ranges)]
end

"""
    scan_features!(X, ranges, tier, j) -> X

Write a scan's features into column `j` of `X`.
"""
function scan_features!(X::AbstractMatrix{Float32}, ranges::AbstractVector{<:Real}, tier::Symbol, j::Integer)
    s = beam_stride(tier)
    for (k, i) in enumerate(1:s:length(ranges))
        X[k, j] = _feature(ranges[i])
    end
    return X
end

_feature(r::Real) = Float32(0 < r <= RANGE_MAX ? r / RANGE_MAX : 1.0)

# A convolution that halves the length (odd kernel, "same" padding, stride 2).
_halve(k, ch, act=relu) = Conv((k,), ch, act; stride=2, pad=(k - 1) ÷ 2)

"""
    build_model(tier, n_in, n_out) -> Lux model

The network for one estimate: input `(n_in, 1, batch)`, output
`(2n_out, batch)`: the normalized mean, then the log standard deviation.
"""
function build_model(tier::Symbol, n_in::Integer, n_out::Integer)
    if tier === :small
        L = cld(cld(cld(n_in, 2), 2), 2)
        return Chain(_halve(5, 1 => 8), _halve(5, 8 => 16), _halve(5, 16 => 16), FlattenLayer(),
            Dense(16L => 64, relu), Dense(64 => 2n_out))
    elseif tier === :large
        L = n_in
        for _ in 1:6
            L = cld(L, 2)
        end
        return Chain(_halve(5, 1 => 16), _halve(5, 16 => 32), _halve(5, 32 => 32), _halve(5, 32 => 64),
            _halve(5, 64 => 64), _halve(5, 64 => 128), FlattenLayer(), Dense(128L => 256, relu),
            Dense(256 => 128, relu), Dense(128 => 2n_out))
    end
    throw(ArgumentError("unknown tier $tier"))
end

# log σ is clamped to this range (normalized units) so the likelihood stays finite.
const LOGσ_MIN, LOGσ_MAX = -6.0f0, 3.0f0

"""
    EstimatorNet

A trained network for one estimate and tier: the model, its parameters and
state, and the mean and standard deviation the targets were normalized by.
"""
struct EstimatorNet{M,P,S}
    estimate::Symbol
    tier::Symbol
    num_beams::Int
    model::M
    ps::P
    st::S
    y_mean::Vector{Float32}
    y_std::Vector{Float32}
end

"""
    EstimatorNet(estimate, tier, num_beams; rng, y_mean, y_std)

An untrained network, with fresh parameters drawn from `rng`.
"""
function EstimatorNet(estimate::Symbol, tier::Symbol, num_beams::Integer; rng::AbstractRNG=Random.default_rng(),
                      y_mean=zeros(Float32, output_size(estimate)), y_std=ones(Float32, output_size(estimate)))
    model = build_model(tier, input_size(tier, num_beams), output_size(estimate))
    ps, st = Lux.setup(rng, model)
    return EstimatorNet(estimate, tier, Int(num_beams), model, ps, st, Float32.(y_mean), Float32.(y_std))
end

"""
    predict(net, X) -> (μ, σ)

Means and standard deviations, each `(n_out, batch)`, in the estimate's
units, for features `X` (`(n_in, batch)`, as from [`scan_features!`](@ref)).
"""
function predict(net::EstimatorNet, X::AbstractMatrix{Float32})
    n = output_size(net.estimate)
    y, _ = net.model(reshape(X, size(X, 1), 1, size(X, 2)), net.ps, Lux.testmode(net.st))
    μ = y[1:n, :] .* net.y_std .+ net.y_mean
    σ = exp.(clamp.(y[n+1:2n, :], LOGσ_MIN, LOGσ_MAX)) .* net.y_std
    return μ, σ
end

"""
    predict(net, ranges) -> (μ, σ)

The estimate from one scan, as vectors.
"""
function predict(net::EstimatorNet, ranges::AbstractVector{<:Real})
    μ, σ = predict(net, reshape(scan_features(ranges, net.tier), :, 1))
    return Float64.(vec(μ)), Float64.(vec(σ))
end

# File names avoid Unicode.
const _FILE_NAMES = Dict(:e => "e", :ψe => "psi_e", :κ_profile => "kappa_profile", :widths => "widths",
    :sight => "sight", :clearance => "clearance")

weights_file(dir, estimate::Symbol, tier::Symbol) = joinpath(dir, "$(tier)_$(_FILE_NAMES[estimate]).jld2")

"""
    save_net(path, net)

Save a network's parameters, state and normalization (on the CPU).
"""
function save_net(path::AbstractString, net::EstimatorNet)
    cpu = cpu_device()
    jldsave(path; estimate=String(net.estimate), tier=String(net.tier), num_beams=net.num_beams,
        ps=cpu(net.ps), st=cpu(net.st), y_mean=net.y_mean, y_std=net.y_std)
    return path
end

"""
    load_net(path) -> EstimatorNet

A network saved by [`save_net`](@ref), on the CPU.
"""
function load_net(path::AbstractString)
    d = load(path)
    est, tier = Symbol(d["estimate"]), Symbol(d["tier"])
    model = build_model(tier, input_size(tier, d["num_beams"]), output_size(est))
    return EstimatorNet(est, tier, d["num_beams"], model, d["ps"], d["st"], d["y_mean"], d["y_std"])
end

"""
    load_nets(dir; tiers=TIERS) -> NamedTuple

Every network in `dir`, as `(small=(e=…, ψe=…, …), large=(…))`.
"""
function load_nets(dir::AbstractString; tiers=TIERS)
    load_tier(t) = NamedTuple{ESTIMATES}(Tuple(load_net(weights_file(dir, est, t)) for est in ESTIMATES))
    return NamedTuple{Tuple(tiers)}(Tuple(load_tier(t) for t in tiers))
end
