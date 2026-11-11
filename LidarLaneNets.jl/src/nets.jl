# The networks: one per estimate and tier, each a 1D convolutional network
# over the scan that outputs the estimate's mean and standard deviation.

"""
    TIERS

`:small` runs on the car: three thin convolutions over every few beams
(shapes in [`SMALL_STRONG_SPEC`](@ref); the trained sets in
[`NETWORKS`](@ref)). `:large` runs in the cloud: the small
network plus a correction from every beam (six wider convolutions, each
output seeing about 63°, and a wider head). The correction starts at zero,
so a large network starts out as its small one and is trained from there:
it is never less accurate on the validation tracks.
"""
const TIERS = (:small, :large)

"""
    SMALL_STRONG_SPEC, SMALL_WEAK_SPEC

The shape of a small network (and of a large one's small part): which beams
it takes (`stride`: every `stride`-th), its three convolutions' channels and
its dense layer's width. `SMALL_WEAK_SPEC` has half the beams and half the
channels of `SMALL_STRONG_SPEC`.
"""
const SMALL_STRONG_SPEC = (stride=4, channels=(8, 16, 16), dense=64)
const SMALL_WEAK_SPEC = (stride=8, channels=(4, 8, 8), dense=32)

"""
    NETWORK_SETS, NETWORKS

The trained network sets, named as their weight files are
(`weights/<name>_<estimate>.jld2`), with each set's tier and the shape of
its small networks (`spec`):

- `:small_weak`: small networks of `SMALL_WEAK_SPEC`, for the car (the edge);
- `:small_strong`: small networks of `SMALL_STRONG_SPEC`, for comparison;
- `:large`: large networks for the cloud; each is a `:small_strong` network
  (its `base`) plus a correction.

`NETWORKS` is their names. [`load_nets`](@ref) pairs `:small_weak` with
`:large` by default.
"""
const NETWORK_SETS = (
    small_weak=(tier=:small, spec=SMALL_WEAK_SPEC),
    small_strong=(tier=:small, spec=SMALL_STRONG_SPEC),
    large=(tier=:large, spec=SMALL_STRONG_SPEC, base=:small_strong),
)
const NETWORKS = keys(NETWORK_SETS)

_set(name::Symbol) = haskey(NETWORK_SETS, name) ? NETWORK_SETS[name] : throw(ArgumentError("unknown network set $name"))
network_tier(name::Symbol) = _set(name).tier
network_spec(name::Symbol) = _set(name).spec

beam_stride(tier::Symbol; spec=SMALL_STRONG_SPEC) =
    tier === :small ? spec.stride : tier === :large ? 1 : throw(ArgumentError("unknown tier $tier"))

input_size(tier::Symbol, num_beams::Integer; spec=SMALL_STRONG_SPEC) = cld(num_beams, beam_stride(tier; spec))

"""
    scan_features(ranges, stride) -> Vector{Float32}
    scan_features(scans, stride) -> Matrix{Float32}

A scan as a network's input: every `stride`-th beam (see [`input_stride`](@ref)),
each range clipped to `RANGE_MAX` (anything out of range or invalid reads as
`RANGE_MAX`) and divided by it. Given a matrix of scans (one per column),
the features of each, in parallel.
"""
scan_features(ranges::AbstractVector{<:Real}, stride::Integer) =
    Float32[_feature(ranges[i]) for i in 1:stride:length(ranges)]

function scan_features(scans::AbstractMatrix{<:Real}, stride::Integer)
    F = Matrix{Float32}(undef, cld(size(scans, 1), stride), size(scans, 2))
    Threads.@threads for j in axes(scans, 2)
        scan_features!(F, view(scans, :, j), stride, j)
    end
    return F
end

"""
    scan_features!(X, ranges, stride, j) -> X

Write a scan's features into column `j` of `X`.
"""
function scan_features!(X::AbstractMatrix{Float32}, ranges::AbstractVector{<:Real}, stride::Integer, j::Integer)
    for (k, i) in enumerate(1:stride:length(ranges))
        X[k, j] = _feature(ranges[i])
    end
    return X
end

_feature(r::Real) = Float32(0 < r <= RANGE_MAX ? r / RANGE_MAX : 1.0)

# A convolution that halves the length (odd kernel, "same" padding, stride 2).
_halve(k, ch, act=relu) = Conv((k,), ch, act; stride=2, pad=(k - 1) ÷ 2)

function _small_model(n_in, n_out, spec)
    L = cld(cld(cld(n_in, 2), 2), 2)
    c1, c2, c3 = spec.channels
    return Chain(_halve(5, 1 => c1), _halve(5, c1 => c2), _halve(5, c2 => c3), FlattenLayer(),
        Dense(c3 * L => spec.dense, relu), Dense(spec.dense => 2n_out))
end

# The large network's correction to the small one, from every beam; its last
# layer starts at zero, so the correction does too.
function _correction(n_in, n_out)
    L = n_in
    for _ in 1:6
        L = cld(L, 2)
    end
    return Chain(_halve(5, 1 => 16), _halve(5, 16 => 32), _halve(5, 32 => 32), _halve(5, 32 => 64),
        _halve(5, 64 => 64), _halve(5, 64 => 128), FlattenLayer(), Dense(128L => 256, relu),
        Dense(256 => 128, relu), Dense(128 => 2n_out; init_weight=zeros32, init_bias=zeros32))
end

# Every k-th beam of a batch of scans.
struct _EveryKth <: Function
    k::Int
end
(f::_EveryKth)(x) = x[1:f.k:end, :, :]

"""
    build_model(tier, n_in, n_out; spec=SMALL_STRONG_SPEC) -> Lux model

The network for one estimate: input `(n_in, 1, batch)`, output
`(2n_out, batch)`: the normalized mean, then the log standard deviation.
`spec` shapes the small network (see [`SMALL_STRONG_SPEC`](@ref)). The large one is
the small one (on every `spec.stride`-th of its beams) plus a correction:
`layer_1.layer_2` of its parameters is a small network's.
"""
function build_model(tier::Symbol, n_in::Integer, n_out::Integer; spec=SMALL_STRONG_SPEC)
    tier === :small && return _small_model(n_in, n_out, spec)
    tier === :large && return Parallel(+,
        Chain(WrappedFunction(_EveryKth(spec.stride)), _small_model(cld(n_in, spec.stride), n_out, spec)),
        _correction(n_in, n_out))
    throw(ArgumentError("unknown tier $tier"))
end

# log σ is clamped to this range (normalized units) so the likelihood stays finite.
const LOGσ_MIN, LOGσ_MAX = -6.0f0, 3.0f0

"""
    EstimatorNet

A trained network for one estimate and tier: the shape of its small
network (`spec`, see [`SMALL_STRONG_SPEC`](@ref)), the model, its parameters and
state, and the mean and standard deviation the targets were normalized by.
"""
struct EstimatorNet{M,P,S}
    estimate::Symbol
    tier::Symbol
    num_beams::Int
    spec::NamedTuple{(:stride, :channels, :dense),Tuple{Int,NTuple{3,Int},Int}}
    model::M
    ps::P
    st::S
    y_mean::Vector{Float32}
    y_std::Vector{Float32}
end

"""
    EstimatorNet(estimate, tier, num_beams; spec=SMALL_STRONG_SPEC, rng, y_mean, y_std)

An untrained network, with fresh parameters drawn from `rng`.
"""
function EstimatorNet(estimate::Symbol, tier::Symbol, num_beams::Integer; spec=SMALL_STRONG_SPEC,
                      rng::AbstractRNG=Random.default_rng(), y_mean=zeros(Float32, output_size(estimate)),
                      y_std=ones(Float32, output_size(estimate)))
    model = build_model(tier, input_size(tier, num_beams; spec), output_size(estimate); spec)
    ps, st = Lux.setup(rng, model)
    return EstimatorNet(estimate, tier, Int(num_beams), _spec(spec), model, ps, st, Float32.(y_mean),
        Float32.(y_std))
end

_spec(s) = (stride=Int(s.stride), channels=NTuple{3,Int}(s.channels), dense=Int(s.dense))

"""
    input_stride(net) -> k

The network takes every `k`-th beam: its small network's stride, or 1 for a
large network.
"""
input_stride(net::EstimatorNet) = beam_stride(net.tier; spec=net.spec)

input_size(net::EstimatorNet) = cld(net.num_beams, input_stride(net))

"""
    large_from_small(small::EstimatorNet; rng) -> EstimatorNet

A large network that is exactly `small` (its small part has `small`'s
parameters and normalization; its correction is zero), to be trained from.
"""
function large_from_small(small::EstimatorNet; rng::AbstractRNG=Random.default_rng())
    small.tier === :small || throw(ArgumentError("need a small network"))
    net = EstimatorNet(small.estimate, :large, small.num_beams; spec=small.spec, rng, y_mean=small.y_mean,
        y_std=small.y_std)
    ps = merge(net.ps, (; layer_1=merge(net.ps.layer_1, (; layer_2=small.ps))))
    st = merge(net.st, (; layer_1=merge(net.st.layer_1, (; layer_2=small.st))))
    return EstimatorNet(net.estimate, :large, net.num_beams, net.spec, net.model, ps, st, net.y_mean, net.y_std)
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
    μ, σ = predict(net, reshape(scan_features(ranges, input_stride(net)), :, 1))
    return Float64.(vec(μ)), Float64.(vec(σ))
end

"""
    weights_file(dir, estimate, name) -> path

Where the network for `estimate` of the set `name` (see [`NETWORKS`](@ref)) is saved.
"""
weights_file(dir, estimate::Symbol, name::Symbol) = joinpath(dir, "$(name)_$(file_name(estimate)).jld2")

"""
    save_net(path, net)

Save a network's shape, parameters, state and normalization (on the CPU).
"""
function save_net(path::AbstractString, net::EstimatorNet)
    cpu = cpu_device()
    jldsave(path; estimate=String(net.estimate), tier=String(net.tier), num_beams=net.num_beams,
        spec_stride=net.spec.stride, spec_channels=collect(net.spec.channels), spec_dense=net.spec.dense,
        ps=cpu(net.ps), st=cpu(net.st), y_mean=net.y_mean, y_std=net.y_std)
    return path
end

"""
    load_net(path) -> EstimatorNet

A network saved by [`save_net`](@ref), on the CPU. (Files without a shape
are `SMALL_STRONG_SPEC`.)
"""
function load_net(path::AbstractString)
    d = load(path)
    est, tier = Symbol(d["estimate"]), Symbol(d["tier"])
    spec = haskey(d, "spec_stride") ?
           _spec((stride=d["spec_stride"], channels=Tuple(d["spec_channels"]), dense=d["spec_dense"])) : SMALL_STRONG_SPEC
    model = build_model(tier, input_size(tier, d["num_beams"]; spec), output_size(est); spec)
    return EstimatorNet(est, tier, d["num_beams"], spec, model, d["ps"], d["st"], d["y_mean"], d["y_std"])
end

"""
    load_nets(dir; small=:small_weak, large=:large) -> NamedTuple

The networks in `dir` for the car (`small`) and the cloud (`large`; `nothing`
for none), as `(small=(e=…, ψe=…, …), large=(…))`: by default the weak small
set on the car and the large set in the cloud (see [`NETWORKS`](@ref)).
"""
function load_nets(dir::AbstractString; small::Symbol=:small_weak, large::Union{Symbol,Nothing}=:large)
    load_set(name) = NamedTuple{ESTIMATES}(Tuple(load_net(weights_file(dir, est, name)) for est in ESTIMATES))
    network_tier(small) === :small || throw(ArgumentError("$small is not a small network set"))
    large === nothing && return (small=load_set(small),)
    network_tier(large) === :large || throw(ArgumentError("$large is not a large network set"))
    return (small=load_set(small), large=load_set(large))
end
