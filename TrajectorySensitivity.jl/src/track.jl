# A closed track built from pieces of known curvature and width, written as
# a ROS map (an image and its yaml) with its centerline, as the
# f1tenth_racetracks maps are: thin walls on free ground.

"""
    Piece(label, length, κ_start, κ_end, width)

A stretch of the track's centerline whose curvature [1/m] (`> 0` turning
left) changes linearly from `κ_start` to `κ_end` over `length` meters, the
lane `width` meters wide. A `NaN` length is free: found by
[`build_track`](@ref) to close the loop (two free straights for the
position, one free arc for the heading).
"""
struct Piece
    label::String
    length::Float64
    κ0::Float64
    κ1::Float64
    width::Float64
end

"""
    straight(length; width, label="straight") -> [Piece]

A straight; `length = NaN` leaves it free.
"""
straight(len::Real; width::Real, label::AbstractString="straight") = [Piece(label, len, 0.0, 0.0, width)]

"""
    bends(κs, angles; ramp=3.0, width, label) -> [Piece]

Arcs one after another (one: a corner; two of opposite sign: an S-bend),
each of curvature `κs[i]` turning by `angles[i]` [rad] (with ramps it
counts half of), the curvature changing linearly over `ramp` meters into
the first, between them, and out of the last (clothoids). `angles[i] =
NaN` leaves that arc free.
"""
function bends(κs, angles; ramp::Real=3.0, width::Real, label::AbstractString)
    out = Piece[]
    prev = 0.0
    for (κ, θ) in zip(κs, angles)
        push!(out, Piece(label, ramp, prev, κ, width))
        len = isnan(θ) ? NaN : abs(θ) / abs(κ) - ramp
        isnan(len) || len >= 0 || throw(ArgumentError("an arc of curvature $κ turning $θ rad is too short for its ramps"))
        push!(out, Piece(label, len, κ, κ, width))
        prev = κ
    end
    push!(out, Piece(label, ramp, prev, 0.0, width))
    return out
end

"""
    corner(κ, angle; ramp=3.0, width, label) -> [Piece]

One corner: [`bends`](@ref) with a single arc.
"""
corner(κ::Real, θ::Real; kw...) = bends([κ], [θ]; kw...)

"""
    spiral(κ_start, κ_end, length; ramp=3.0, width, label) -> [Piece]

A corner whose curvature changes from `κ_start` to `κ_end` over `length`
meters (tightening, say), entered and left over `ramp` meters.
"""
spiral(κa::Real, κb::Real, len::Real; ramp::Real=3.0, width::Real, label::AbstractString) =
    [Piece(label, ramp, 0.0, κa, width), Piece(label, len, κa, κb, width), Piece(label, ramp, κb, 0.0, width)]

"""
    TrackDesign(pieces; turn=2π)

A closed track: its `pieces` in driving order, turning by `turn` in all
(2π: counterclockwise). Exactly one arc and two straights (not parallel)
must be free (`NaN` length).
"""
struct TrackDesign
    pieces::Vector{Piece}
    turn::Float64
end

TrackDesign(pieces::AbstractVector{Piece}; turn::Real=2π) = TrackDesign(collect(pieces), Float64(turn))

"""
    BuiltTrack

A track's centerline every `step` meters: `x`, `y`, `heading`, curvature
`κ`, lane `width`, arc length `s`; its `length`; its `pieces` (with every
length found) and `segments`, `(s_start, s_end, label)` with consecutive
pieces of one label merged.
"""
struct BuiltTrack
    x::Vector{Float64}
    y::Vector{Float64}
    heading::Vector{Float64}
    κ::Vector{Float64}
    width::Vector{Float64}
    s::Vector{Float64}
    length::Float64
    pieces::Vector{Piece}
    segments::Vector{Tuple{Float64,Float64,String}}
end

_angle(p::Piece) = (p.κ0 + p.κ1) / 2 * p.length

# Integrate the pieces from (0, 0) heading 0: points every `step` (and the end).
function _integrate(pieces, step)
    x, y, h, κs, ws, ss = [0.0], [0.0], [0.0], [pieces[1].κ0], [pieces[1].width], [0.0]
    for p in pieces
        p.length > 0 || continue
        n = max(1, ceil(Int, p.length / step))
        ds = p.length / n
        for i in 1:n
            κa = p.κ0 + (p.κ1 - p.κ0) * (i - 1) / n
            κb = p.κ0 + (p.κ1 - p.κ0) * i / n
            hm = h[end] + ds * (3κa + κb) / 8   # heading at the step's middle
            push!(x, x[end] + ds * cos(hm)); push!(y, y[end] + ds * sin(hm))
            push!(h, h[end] + ds * (κa + κb) / 2); push!(κs, κb); push!(ws, p.width); push!(ss, ss[end] + ds)
        end
    end
    return x, y, h, κs, ws, ss
end

"""
    build_track(design::TrackDesign; step=0.05, blend=5.0) -> BuiltTrack

The centerline of `design`, its free lengths found so that it closes: the
free arc for the heading, the two free straights for the position. Where
the width changes, it blends over `blend` meters.
"""
function build_track(d::TrackDesign; step::Real=0.05, blend::Real=5.0)
    pieces = copy(d.pieces)
    free = findall(p -> isnan(p.length), pieces)
    arcs = filter(i -> pieces[i].κ0 != 0, free)
    lines = filter(i -> pieces[i].κ0 == 0 && pieces[i].κ1 == 0, free)
    (length(arcs) == 1 && length(lines) == 2 && length(free) == 3) ||
        throw(ArgumentError("a design needs exactly one free arc and two free straights"))
    # the heading: the free arc turns what the others don't
    a = only(arcs)
    rest = d.turn - sum(_angle(p) for (i, p) in enumerate(pieces) if !(i in free))
    len = rest / pieces[a].κ0
    len > 0 || throw(ArgumentError("the free arc would have to turn $(rest) rad the other way"))
    pieces[a] = Piece(pieces[a].label, len, pieces[a].κ0, pieces[a].κ1, pieces[a].width)
    # the position: linear in the two straights' lengths
    zeroed = [i in lines ? Piece(p.label, 0.0, 0.0, 0.0, p.width) : p for (i, p) in enumerate(pieces)]
    x, y, _, _, _, _ = _integrate(zeroed, step)
    head(i) = sum((_angle(p) for p in zeroed[1:i-1]); init=0.0)
    A = [cos(head(lines[1])) cos(head(lines[2])); sin(head(lines[1])) sin(head(lines[2]))]
    abs(det(A)) > 1e-3 || throw(ArgumentError("the free straights are parallel"))
    L = A \ [-x[end], -y[end]]
    all(>(0), L) || throw(ArgumentError("the loop does not close with straights of positive length: $(L)"))
    for (k, i) in enumerate(lines)
        pieces[i] = Piece(pieces[i].label, L[k], 0.0, 0.0, pieces[i].width)
    end
    x, y, h, κ, w, s = _integrate(pieces, step)
    # the last point is the first again
    total = s[end]
    pop!(x); pop!(y); pop!(h); pop!(κ); pop!(w); pop!(s)
    # blend the widths (a circular moving average)
    n, k = length(w), max(1, round(Int, blend / step / 2))
    wb = [sum(w[mod1(i + j, n)] for j in -k:k) / (2k + 1) for i in 1:n]
    # segments: runs of one label
    segs = Tuple{Float64,Float64,String}[]
    s0 = 0.0
    for (i, p) in enumerate(pieces)
        p.length > 0 || continue
        if !isempty(segs) && segs[end][3] == p.label
            segs[end] = (segs[end][1], s0 + p.length, p.label)
        else
            push!(segs, (s0, s0 + p.length, p.label))
        end
        s0 += p.length
    end
    return BuiltTrack(x, y, h, κ, wb, s, total, pieces, segs)
end

"""
    track_problems(t::BuiltTrack; wall=0.1, gap=0.5) -> Vector{String}

What would make the track unusable: an inner wall that folds over (the
curvature times half the width near 1), or two parts of the track closer
than their half widths, the walls (`wall` meters thick) and `gap`.
"""
function track_problems(t::BuiltTrack; wall::Real=0.1, gap::Real=0.5)
    out = String[]
    fold = maximum(abs.(t.κ) .* t.width ./ 2)
    fold >= 0.9 && push!(out, "an inner wall comes within $(round(1 - fold; digits=2)) of folding over (|κ| w / 2 = $(round(fold; digits=2)))")
    stride = max(1, round(Int, 0.5 / (t.s[2] - t.s[1])))
    idx = 1:stride:length(t.x)
    for (a, i) in enumerate(idx), j in idx[a+1:end]
        along = abs(t.s[j] - t.s[i])
        min(along, t.length - along) < 3 * max(t.width[i], t.width[j]) && continue
        need = (t.width[i] + t.width[j]) / 2 + 2wall + gap
        dist = hypot(t.x[i] - t.x[j], t.y[i] - t.y[j])
        if dist < need
            push!(out, "s = $(round(t.s[i]; digits=1)) and $(round(t.s[j]; digits=1)) m are $(round(dist; digits=2)) m apart (need $(round(need; digits=2)))")
            length(out) > 10 && return out
        end
    end
    return out
end

"""
    track_map(t::BuiltTrack; resolution=0.05, wall=0.1, border=3.0) -> (image, origin)

The track as a map image (gray values, row 1 at the top, as a picture):
walls `wall` meters thick from half the width on each side of the
centerline outward, everything else free; `border` meters of free ground around.
`origin` is the map's bottom left corner `(x, y, 0)`, as in its yaml.
"""
function track_map(t::BuiltTrack; resolution::Real=0.05, wall::Real=0.1, border::Real=3.0)
    n = length(t.x)
    nx(i) = -sin(t.heading[i])
    ny(i) = cos(t.heading[i])
    walls = [(side, i) for side in (1, -1) for i in 1:n]
    r = max(wall / 2, 0.75resolution)   # at least a pixel across, whatever the direction
    off(i) = t.width[i] / 2 + r          # the wall's middle, its inner face at half the width
    wx(side, i) = t.x[i] + side * off(i) * nx(i)
    wy(side, i) = t.y[i] + side * off(i) * ny(i)
    xs = [wx(sd, i) for (sd, i) in walls]
    ys = [wy(sd, i) for (sd, i) in walls]
    x0, y0 = minimum(xs) - border, minimum(ys) - border
    W = ceil(Int, (maximum(xs) + border - x0) / resolution)
    H = ceil(Int, (maximum(ys) + border - y0) / resolution)
    img = fill(0xff, H, W)
    rc = ceil(Int, r / resolution)
    paint(px, py) = begin
        c, rb = floor(Int, (px - x0) / resolution), floor(Int, (py - y0) / resolution)   # 0-based, from the bottom
        for dc in -rc:rc, dr in -rc:rc
            cc, rr = c + dc, rb + dr
            (0 <= cc < W && 0 <= rr < H) || continue
            hypot((cc + 0.5) * resolution + x0 - px, (rr + 0.5) * resolution + y0 - py) <= r + 1e-9 || continue
            img[H-rr, cc+1] = 0x00
        end
    end
    for side in (1, -1), i in 1:n
        j = mod1(i + 1, n)
        ax, ay, bx, by = wx(side, i), wy(side, i), wx(side, j), wy(side, j)
        k = max(1, ceil(Int, hypot(bx - ax, by - ay) / (resolution / 3)))
        for q in 0:k-1
            paint(ax + (bx - ax) * q / k, ay + (by - ay) * q / k)
        end
    end
    return img, (x0, y0, 0.0)
end

"""
    write_track(dir, name, t::BuiltTrack; resolution=0.05, wall=0.1, border=3.0, centerline_step=0.2) -> yaml path

Write the track as the f1tenth_racetracks maps are: `<name>_map.pgm` (8-bit
gray) and `<name>_map.yaml`, `<name>_centerline.csv` (x, y and the room to
the right and left, every `centerline_step` meters, starting at the start
line), and `<name>_segments.csv` (where each kind of segment starts and
ends along the centerline). Returns the yaml's path without its extension,
as F1Tenth.jl's `F110Env(map=…)` takes it (with `map_ext=".pgm"`).
"""
function write_track(dir::AbstractString, name::AbstractString, t::BuiltTrack; resolution::Real=0.05, wall::Real=0.1,
                     border::Real=3.0, centerline_step::Real=0.2)
    mkpath(dir)
    img, (x0, y0, _) = track_map(t; resolution, wall, border)
    base = joinpath(dir, name * "_map")
    open(base * ".pgm", "w") do io
        H, W = size(img)
        write(io, "P5\n$W $H\n255\n")
        for r in 1:H
            write(io, img[r, :])
        end
    end
    write(base * ".yaml", """
        image: $(name)_map.pgm
        resolution: $(resolution)
        origin: [$(x0), $(y0), 0.000000]
        negate: 0
        occupied_thresh: 0.45
        free_thresh: 0.196
        """)
    stride = max(1, round(Int, centerline_step / (t.s[2] - t.s[1])))
    open(joinpath(dir, name * "_centerline.csv"), "w") do io
        println(io, "# x_m, y_m, w_tr_right_m, w_tr_left_m")
        for i in 1:stride:length(t.x)
            println(io, t.x[i], ", ", t.y[i], ", ", t.width[i] / 2, ", ", t.width[i] / 2)
        end
    end
    open(joinpath(dir, name * "_segments.csv"), "w") do io
        println(io, "# s_start_m, s_end_m, label")
        for (a, b, l) in t.segments
            println(io, a, ", ", b, ", ", l)
        end
    end
    return base
end

"""
    read_segments(path) -> Vector{Tuple{Float64,Float64,String}}

The segments written by [`write_track`](@ref).
"""
function read_segments(path::AbstractString)
    out = Tuple{Float64,Float64,String}[]
    for l in eachline(path)
        (startswith(l, "#") || isempty(strip(l))) && continue
        a, b, c = strip.(split(l, ","; limit=3))
        push!(out, (parse(Float64, a), parse(Float64, b), String(c)))
    end
    return out
end
