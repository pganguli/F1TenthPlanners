# Kinds of situation, and the networks' recorded errors replayed by them.

"""
    SITUATIONS

The kinds of situation the results are grouped by, from how much the lane
bends beside the car and 1 to 3 m ahead (see [`situation_class`](@ref)):
straight, or a gentle, medium or tight bend, on its entry, in its middle or
on its exit.
"""
const SITUATIONS = ["straight", "gentle entry", "gentle middle", "gentle exit", "medium entry", "medium middle",
    "medium exit", "tight entry", "tight middle", "tight exit"]

"""
    situation_class(labels) -> Int

The kind of situation (an index into [`SITUATIONS`](@ref)) of a scan, from
its true estimates: the bend is the larger |κ| beside the car or 1 to 3 m
ahead (straight below 0.05 1/m, gentle below 0.2, medium below 0.5, tight
above; the racetracks' center lines bend more than 0.07, 0.19 and 0.66
1/m a quarter, a tenth and a hundredth of the way); entry when the lane
bends more ahead than beside the car (by 0.05 1/m), exit when less.
"""
function situation_class(l)
    κp = l.κ_profile
    here = abs(κp[1])
    ahead = maximum(abs(κp[j]) for j in eachindex(PROFILE_S) if 1.0 <= PROFILE_S[j] <= 3.0)
    peak = max(here, ahead)
    peak < 0.05 && return 1
    bend = peak < 0.2 ? 0 : peak < 0.5 ? 1 : 2
    phase = ahead > here + 0.05 ? 1 : here > ahead + 0.05 ? 3 : 2
    return 1 + 3bend + phase
end

"""
    situation_features(labels) -> Vector{Float64}

What the replay matches a scan by: the true lane the network looks at.
`e`, `ψe`, the curvature at 0, 1, 2, 3 and 5 m ahead, the room to the left
and to the right beside the car and 2 m ahead, and how far ahead the lane is
seen.
"""
function situation_features(l)
    κ, w = l.κ_profile, l.widths
    at(s) = round(Int, s / PROFILE_STEP) + 1
    np = length(PROFILE_S)
    return [l.e[1], l.ψe[1], (κ[at(s)] for s in FEATURE_S)..., w[at(0.0)], w[at(2.0)], w[np+at(0.0)], w[np+at(2.0)],
        l.sight[1]]
end

const FEATURE_S = (0.0, 1.0, 2.0, 3.0, 5.0)
const _N_FEATURES = 2 + length(FEATURE_S) + 5

# The features of the mirror image (left and right swapped).
function _mirror_features(f::AbstractVector)
    nκ = length(FEATURE_S)
    return [-f[1]; -f[2]; -f[3:2+nκ]; f[5+nκ:6+nκ]; f[3+nκ:4+nκ]; f[end]]
end

# An estimate's error (`sign=true`) or σ (`sign=false`) for the mirror image.
function _mirror(est::Symbol, v::AbstractVector; sign::Bool)
    est === :widths && return [v[length(PROFILE_S)+1:end]; v[1:length(PROFILE_S)]]
    est === :clearance && return reverse(v)                     # CLEARANCE_K is symmetric about 0
    est in (:e, :ψe, :κ_profile) && sign && return -v
    return v
end

"""
    MeanError

An estimate's mean error, per entry, as a function of the entry's true
value: binned (`N_BINS` equal bins between the entry's least and greatest
true value), and interpolated between the bins' centers. Fitted over the
library's scans and their mirror images, since the replay uses both.
"""
struct MeanError
    lo::Vector{Float64}
    width::Vector{Float64}
    mean::Matrix{Float64}                  # bins × entries
end

const N_BINS = 24
const MIN_BIN = 20                         # a bin with fewer scans takes its nearest fuller bin's mean

function (m::MeanError)(j::Integer, x::Real)
    isfinite(x) || return 0.0
    u = (x - m.lo[j]) / m.width[j] + 0.5   # in bin centers: 1 at the first one's
    u <= 1 && return m.mean[1, j]
    u >= N_BINS && return m.mean[N_BINS, j]
    i = floor(Int, u)
    return m.mean[i, j] + (u - i) * (m.mean[i+1, j] - m.mean[i, j])
end

"""
    ErrorLibrary

The car's and the cloud's networks' errors (estimate − truth) and their own
σ, recorded scan after scan on the same drives (so their correlation with
each other, over time and across entries stays as it was), with each scan's
true estimates and lane ([`situation_features`](@ref)), its track, and whether the next
scan follows it in the same drive. Written by scripts/error_library.jl;
[`load_library`](@ref). `without(lib, track)` leaves a track's scans out.
"""
struct ErrorLibrary
    sets::NTuple{2,Symbol}                 # the car's and the cloud's network sets
    tracks::Vector{String}
    track::Vector{Int}                     # each scan's, an index into `tracks`
    follows::BitVector                     # scan k + 1 follows scan k in the same drive
    features::Matrix{Float64}              # situation features × scans, scaled by their spread
    mirrored::Matrix{Float64}              # the same, of the mirror images
    scale::Vector{Float64}
    usable::BitVector                      # the scans a replay may use
    truth::Dict{Symbol,Matrix{Float32}}    # estimate => its true value, entries × scans
    err::Dict{Tuple{Symbol,Symbol},Matrix{Float32}}   # (estimate, :car or :cloud) => entries × scans
    σ::Dict{Tuple{Symbol,Symbol},Matrix{Float32}}
    mean_err::Dict{Tuple{Symbol,Symbol},MeanError}    # per entry: the mean error by the true value
    cache::Dict{String,ErrorLibrary}       # `without` each track, made once
    lock::ReentrantLock
end

function ErrorLibrary(sets, tracks, track, follows, features::AbstractMatrix, truth, err, σ; min_bin::Integer=MIN_BIN)
    scale = [max(sqrt(sum(abs2, r .- sum(r) / length(r)) / max(length(r) - 1, 1)), 1e-6) for r in eachrow(features)]
    F = features ./ scale
    M = reduce(hcat, (_mirror_features(c) for c in eachcol(features))) ./ scale
    usable = trues(length(track))
    return ErrorLibrary(Tuple(sets), collect(String, tracks), collect(Int, track), BitVector(follows), F, M, scale,
        usable, truth, err, σ, _mean_errors(truth, err, usable; min_bin), Dict{String,ErrorLibrary}(), ReentrantLock())
end

function _mean_errors(truth, err, usable; min_bin::Integer=MIN_BIN)
    out = Dict{Tuple{Symbol,Symbol},MeanError}()
    for ((est, role), E) in err
        X = truth[est]
        n = size(E, 1)
        lo, hi = fill(Inf, n), fill(-Inf, n)
        both(f) = for k in eachindex(usable)
            usable[k] || continue
            x, e = Float64.(X[:, k]), Float64.(E[:, k])
            f(x, e)
            f(_mirror(est, x; sign=true), _mirror(est, e; sign=true))
        end
        both() do x, _
            for j in 1:n
                isfinite(x[j]) && (lo[j] = min(lo[j], x[j]); hi[j] = max(hi[j], x[j]))
            end
        end
        width = [isfinite(hi[j] - lo[j]) ? max(hi[j] - lo[j], 1e-9) / N_BINS : 1.0 for j in 1:n]
        lo = [isfinite(l) ? l : 0.0 for l in lo]
        sums, counts = zeros(N_BINS, n), zeros(Int, N_BINS, n)
        both() do x, e
            for j in 1:n
                (isfinite(x[j]) && isfinite(e[j])) || continue
                i = clamp(floor(Int, (x[j] - lo[j]) / width[j]) + 1, 1, N_BINS)
                sums[i, j] += e[j]; counts[i, j] += 1
            end
        end
        mean = zeros(N_BINS, n)
        for j in 1:n
            full = findall(>=(min_bin), @view counts[:, j])
            isempty(full) && continue
            for i in 1:N_BINS
                f = full[argmin(abs.(full .- i))]
                mean[i, j] = sums[f, j] / counts[f, j]
            end
        end
        out[(est, role)] = MeanError(lo, width, mean)
    end
    return out
end

"""
    without(lib, track) -> ErrorLibrary

The library without the scans recorded on `track` (by name; the same
library if it has none), so that a run on a track never replays that
track's own errors.
"""
function without(lib::ErrorLibrary, name::AbstractString)
    k = findfirst(==(name), lib.tracks)
    k === nothing && return lib
    lock(lib.lock) do
        get!(lib.cache, name) do
            usable = lib.usable .& (lib.track .!= k)
            ErrorLibrary(lib.sets, lib.tracks, lib.track, lib.follows, lib.features, lib.mirrored, lib.scale, usable,
                lib.truth, lib.err, lib.σ, _mean_errors(lib.truth, lib.err, usable), Dict{String,ErrorLibrary}(),
                ReentrantLock())
        end
    end
end

"""
    load_library(path) -> ErrorLibrary

An error library saved by scripts/error_library.jl.
"""
function load_library(path::AbstractString)
    d = JLD2.load(path)
    err = Dict((Symbol(e), Symbol(r)) => v for ((e, r), v) in d["err"])
    σ = Dict((Symbol(e), Symbol(r)) => v for ((e, r), v) in d["sigma"])
    truth = Dict(Symbol(e) => v for (e, v) in d["truth"])
    return ErrorLibrary(Symbol.(d["sets"]), d["tracks"], d["track"], d["follows"], d["features"], truth, err, σ)
end

"""
    ReplayDraws(library, estimate, rng; block=40, near=8, keep=1.0)

Replays a library's recorded errors for `estimate`, by situation: each scan
gets the errors of a recorded scan whose true lane
([`situation_features`](@ref), each scaled by its spread) is like the car's,
mirrored when the recorded lane bends the other way. It goes on through the
recorded drive, scan after scan, while the recorded lane stays within
`keep` of the car's (and for at most `block` scans, 1 s at 40 Hz); then it
starts again at one of the `near` recorded scans nearest the car's lane.
The car's and the cloud's errors for a scan come from the same recorded
scan.

A recorded error is carried over to the car's lane: where an entry's true
value differs from the recorded scan's, the error changes by how much the
network's mean error ([`MeanError`](@ref), over the whole library) differs
between the two values. A network that misses a wide-open side's width, or
pulls a tight bend's curvature toward a gentler one, does so by the car's
own lane.
"""
mutable struct ReplayDraws{R<:AbstractRNG}
    lib::ErrorLibrary
    estimate::Symbol
    rng::R
    block::Int
    near::Int
    keep::Float64
    idx::Int
    flip::Bool
    left::Int
    t::Float64
end

ReplayDraws(lib::ErrorLibrary, est::Symbol, rng::AbstractRNG; block::Integer=40, near::Integer=8, keep::Real=1.0) =
    ReplayDraws(lib, est, rng, block, near, Float64(keep), 0, false, 0, NaN)

_dist2(F, j, f) = sum(abs2(F[i, j] - f[i]) for i in eachindex(f))

function _advance!(d::ReplayDraws, f::AbstractVector)
    lib = d.lib
    F(flip) = flip ? lib.mirrored : lib.features
    j = d.idx + 1
    if d.idx > 0 && d.left > 0 && lib.follows[d.idx] && lib.usable[j] && _dist2(F(d.flip), j, f) <= d.keep^2
        d.idx, d.left = j, d.left - 1
        return d
    end
    # the `near` nearest recorded scans, either way round
    best = fill((Inf, 0, false), d.near)
    for flip in (false, true), j in eachindex(lib.usable)
        lib.usable[j] || continue
        δ = _dist2(F(flip), j, f)
        if δ < best[end][1]
            best[end] = (δ, j, flip)
            sort!(best; by=first)
        end
    end
    filter!(b -> b[2] > 0, best)
    isempty(best) && throw(ArgumentError("the error library has no usable scans"))
    _, d.idx, d.flip = rand(d.rng, best)
    d.left = d.block - 1
    return d
end

"""
    oracle_errors!(draws, role, t, dt, estimate, labels) -> (error, σ)

The error and σ a stand-in network in `role` (`:car` or `:cloud`) adds for
the scan at time `t` (`dt` after the last), whose true estimates are
`labels`.
"""
function oracle_errors!(d::ReplayDraws, role::Symbol, t, dt, est, labels)
    if t != d.t
        _advance!(d, situation_features(labels) ./ d.lib.scale)
        d.t = t
    end
    lib = d.lib
    e = Float64.(lib.err[(est, role)][:, d.idx])
    σ = Float64.(lib.σ[(est, role)][:, d.idx])
    x = Float64.(lib.truth[est][:, d.idx])
    if d.flip
        e, σ, x = _mirror(est, e; sign=true), _mirror(est, σ; sign=false), _mirror(est, x; sign=true)
    end
    m, now = lib.mean_err[(est, role)], labels[est]
    for j in eachindex(e)
        e[j] += m(j, now[j]) - m(j, x[j])
    end
    return e, σ
end

oracle_errors!(d::ErrorDraws, role::Symbol, t, dt, est, labels) = draw!(d, dt, situation(est, labels))
