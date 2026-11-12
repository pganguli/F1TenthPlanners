# The car's trajectory, measured along the track's centerline, and against a
# reference trajectory.

"""
    TrajectoryLog(center::Path)

The car's trajectory, measured against the track's closed `center` line:
for each sample ([`log!`](@ref)), the time `t`, the distance driven along
the centerline `s` (counted on past the start line, lap after lap), the
lateral offset `d` of the car from the centerline (`> 0` to its left), the
heading error `ψ` (the centerline's direction minus the car's), the speed
`v` and the gap from the body to the nearest wall; the pose `x`, `y`, `θ`.
`crashed` is set by the simulation.
"""
mutable struct TrajectoryLog
    center::Path
    length::Float64
    hint::Int
    laps::Int
    last_s::Float64
    t::Vector{Float64}
    s::Vector{Float64}
    d::Vector{Float64}
    ψ::Vector{Float64}
    v::Vector{Float64}
    gap::Vector{Float64}
    x::Vector{Float64}
    y::Vector{Float64}
    θ::Vector{Float64}
    crashed::Bool
end

TrajectoryLog(center::Path) = TrajectoryLog(center, center.s[end] + center.seg_length[end], 0, 0, NaN,
    Float64[], Float64[], Float64[], Float64[], Float64[], Float64[], Float64[], Float64[], Float64[], false)

"""
    log!(log, t, x, y, θ, v, gap) -> log

Add a sample: the car's reference point `(x, y)`, heading `θ`, speed `v`
and gap to the walls at time `t`.
"""
function log!(lg::TrajectoryLog, t, x, y, θ, v, gap)
    err = tracking_error(lg.center, x, y, θ; hint=lg.hint, window=lg.hint == 0 ? 0 : 40)
    lg.hint = err.segment
    s = err.s
    # crossing the start line, either way (a first sample just behind it counts from −length)
    if !isfinite(lg.last_s)
        s > lg.length / 2 && (lg.laps = -1)
    else
        s - lg.last_s < -lg.length / 2 && (lg.laps += 1)
        s - lg.last_s > lg.length / 2 && (lg.laps -= 1)
    end
    lg.last_s = s
    push!(lg.t, t); push!(lg.s, s + lg.laps * lg.length)
    # tracking_error's e is the path's offset from the point; the car's from the path is its negative
    push!(lg.d, -err.e); push!(lg.ψ, err.ψe); push!(lg.v, v); push!(lg.gap, gap)
    push!(lg.x, x); push!(lg.y, y); push!(lg.θ, θ)
    return lg
end

"""
    distance_driven(log) -> m

How far along the centerline the car got (from its first sample).
"""
distance_driven(lg::TrajectoryLog) = isempty(lg.s) ? 0.0 : maximum(lg.s) - lg.s[1]

# Linear interpolation of y(x) at xq, x increasing; NaN outside.
function _interp(x, y, xq)
    out = fill(NaN, length(xq))
    isempty(x) && return out
    if length(x) == 1   # the car never got anywhere
        out[xq.==x[1]] .= y[1]
        return out
    end
    for (k, q) in enumerate(xq)
        (q < x[1] || q > x[end]) && continue
        i = clamp(searchsortedlast(x, q), 1, length(x) - 1)
        f = x[i+1] > x[i] ? (q - x[i]) / (x[i+1] - x[i]) : 0.0
        out[k] = (1 - f) * y[i] + f * y[i+1]
    end
    return out
end

"""
    along(log, grid) -> NamedTuple

The log at the distances `grid` along the centerline (NaN where the car
did not get to): offset `d`, heading error `ψ`, speed `v`, gap and the time
`t` it got there. The car's first pass through each distance counts.
"""
function along(lg::TrajectoryLog, grid)
    # the first pass: keep the samples that set a new farthest distance
    keep = Int[]
    far = -Inf
    for (i, s) in enumerate(lg.s)
        s > far && (push!(keep, i); far = s)
    end
    s = lg.s[keep]
    f(v) = _interp(s, v[keep], grid)
    return (s=collect(Float64, grid), d=f(lg.d), ψ=f(lg.ψ), v=f(lg.v), gap=f(lg.gap), t=f(lg.t))
end

"""
    Comparison

A run against the reference, on the grid `s` along the centerline (where
both got to): the differences in lateral offset `Δd` [m], heading `Δψ`
[rad], speed `Δv` [m/s] and arrival time `Δt` [s]; and over the whole run,
`max_Δd`, `rms_Δd`, `max_Δψ`, `max_Δv`, `Δlap` (the lap time's change; NaN
unless both finished the distance), `min_gap` [m], whether it `crashed`
and where (`crash_s`).
"""
struct Comparison
    s::Vector{Float64}
    Δd::Vector{Float64}
    Δψ::Vector{Float64}
    Δv::Vector{Float64}
    Δt::Vector{Float64}
    max_Δd::Float64
    rms_Δd::Float64
    max_Δψ::Float64
    max_Δv::Float64
    Δlap::Float64
    min_gap::Float64
    crashed::Bool
    crash_s::Float64
end

_nanmax(x) = (y = filter(isfinite, x); isempty(y) ? NaN : maximum(y))
_nanrms(x) = (y = filter(isfinite, x); isempty(y) ? NaN : sqrt(sum(abs2, y) / length(y)))

"""
    compare(run, ref; distance=ref's, step=0.25) -> Comparison

The trajectory `run` (a [`TrajectoryLog`](@ref)) against the reference
`ref`, every `step` meters along the centerline over the first `distance`
meters from the start.
"""
function compare(run::TrajectoryLog, ref::TrajectoryLog; distance::Real=distance_driven(ref), step::Real=0.25)
    s0 = isempty(ref.s) ? 0.0 : ref.s[1]
    grid = s0:step:s0+distance
    a, b = along(run, grid), along(ref, grid)
    Δd, Δψ, Δv, Δt = a.d .- b.d, rem2pi.(a.ψ .- b.ψ, RoundNearest), a.v .- b.v, a.t .- b.t
    last_t(x) = (i = findlast(isfinite, x); i === nothing ? NaN : x[i])
    finished = isfinite(a.t[end]) && isfinite(b.t[end])
    crash_s = run.crashed && !isempty(run.s) ? maximum(run.s) - s0 : NaN
    return Comparison(collect(grid) .- s0, Δd, Δψ, Δv, Δt, _nanmax(abs.(Δd)), _nanrms(Δd), _nanmax(abs.(Δψ)),
        _nanmax(abs.(Δv)), finished ? a.t[end] - b.t[end] : NaN, isempty(run.gap) ? NaN : minimum(run.gap),
        run.crashed, crash_s)
end

"""
    tube(runs; distance, step=0.25) -> NamedTuple

The envelope of the trajectories `runs` (each a [`TrajectoryLog`](@ref),
from the same start): along the centerline `s` (from the start), the least
and greatest lateral offset (`lo`, `hi`) and speed (`v_lo`, `v_hi`) over
the runs that got there, and how many did (`n`). Its `width` is `hi − lo`.
"""
function tube(runs::AbstractVector{TrajectoryLog}; distance::Real, step::Real=0.25)
    s0 = runs[1].s[1]
    grid = s0:step:s0+distance
    A = [along(r, grid) for r in runs]
    n = length(grid)
    lo, hi, vlo, vhi, cnt = fill(NaN, n), fill(NaN, n), fill(NaN, n), fill(NaN, n), zeros(Int, n)
    for k in 1:n
        ds = [a.d[k] for a in A if isfinite(a.d[k])]
        vs = [a.v[k] for a in A if isfinite(a.v[k])]
        cnt[k] = length(ds)
        isempty(ds) && continue
        lo[k], hi[k] = extrema(ds)
        vlo[k], vhi[k] = extrema(vs)
    end
    return (s=collect(grid) .- s0, lo, hi, width=hi .- lo, v_lo=vlo, v_hi=vhi, n=cnt)
end

"""
    by_segment(s, x, segments; f=_nanmax) -> Dict{String,Float64}

`f` of the values `x` (along the centerline at `s`, from the start) over
each kind of segment: `segments` is a list of `(s_start, s_end, label)`
along the centerline; NaNs are skipped.
"""
function by_segment(s, x, segments; f=_nanmax)
    out = Dict{String,Vector{Float64}}()
    len = isempty(segments) ? Inf : maximum(seg[2] for seg in segments)
    for (sk, xk) in zip(s, x)
        isfinite(xk) || continue
        u = mod(sk, len)
        i = findfirst(seg -> seg[1] <= u < seg[2], segments)
        i === nothing && continue
        push!(get!(out, segments[i][3], Float64[]), xk)
    end
    return Dict(k => f(v) for (k, v) in out)
end
