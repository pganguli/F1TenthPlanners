# The representative track: one closed track with the situations of all 23
# f1tenth_racetracks tracks, for the sensitivity runs. Builds it from the
# design below, measures it and the real tracks the same way, and writes
#
# - track/Representative_map.pgm, .yaml, _centerline.csv, _segments.csv;
# - track/Representative.png, a picture of it;
# - results/track.md, its geometry against the real tracks'.
#
#     julia --project=. track.jl

const TRAIN = joinpath(@__DIR__, "..", "..", "LidarLaneNets.jl", "train")
include(joinpath(TRAIN, "truth.jl"))   # Track, center_curvature, free_run, ...

using FileIO
using Printf
using TrajectorySensitivity

const NAME = "Representative"
const DIR = joinpath(@__DIR__, "track")

# Lane widths [m]: the real tracks' are constant along each track, from 1.47
# (Montreal) to 2.62 (BrandsHatch), median 2.26 (results/track.md).
const W_NARROW, W_MID, W_WIDE = 1.5, 2.2, 2.6

# Curvatures [1/m] and turns [rad] from the real tracks' corners (turning
# more than 0.3 rad): peak curvature 0.15 / 0.39 / 1.03 at the 10th / 50th /
# 90th percentiles, turn 1.26 / 2.46 at the 50th / 90th, hairpins (more than
# 2 rad) peaking at 0.2 to 1.7. Straights: 11 m median, 38 m at the 90th
# percentile, 108 m at most. An inner wall folds over where the curvature
# times half the width reaches 1, so the tightest corners are in the
# narrow lane.
function design()
    p = Piece[]
    append!(p, straight(NaN; width=W_MID, label="long straight"))                       # start line; free
    append!(p, corner(0.75, 2.62; width=W_MID, label="hairpin"))                         # after the long straight: hard braking
    append!(p, straight(11.0; width=W_MID))
    append!(p, corner(-0.39, 1.26; width=W_MID, label="medium corner"))                  # the median corner, to the right
    append!(p, straight(8.0; width=W_WIDE))
    append!(p, corner(0.15, 0.9; ramp=4.0, width=W_WIDE, label="fast sweeper"))
    append!(p, straight(12.0; width=W_WIDE))
    append!(p, bends([-0.6, 0.6], [0.7, 0.7]; ramp=1.0, width=W_NARROW, label="chicane"))
    append!(p, straight(6.0; width=W_NARROW))
    append!(p, corner(1.0, 2.46; ramp=2.0, width=W_NARROW, label="tight hairpin"))      # the 90th percentile turn
    append!(p, straight(NaN; width=W_MID))                                               # free
    append!(p, spiral(-0.15, -0.6, 3.0; ramp=2.0, width=W_MID, label="tightening corner"))
    append!(p, straight(10.0; width=W_MID))
    append!(p, bends([0.3, 0.15, 0.35], [1.0, 0.4, NaN]; ramp=2.0, width=W_MID, label="double apex"))  # free: closes the turn
    return TrackDesign(p)
end

# --- measuring a track the same way, real or built

function geometry(tr::Track)
    ss = 0.0:0.25:tr.length-0.25
    κ = [center_curvature(tr, s) for s in ss]
    w = map(ss) do s
        px, py = center_point(tr, s)
        h = center_heading(tr, s)
        free_run(tr.map, px, py, -sin(h), cos(h), WIDTH_MAX) + free_run(tr.map, px, py, sin(h), -cos(h), WIDTH_MAX)
    end
    straights, corners = Float64[], NamedTuple[]
    run = 0
    for k in κ
        abs(k) < 0.03 ? (run += 1) : (run > 0 && push!(straights, 0.25run); run = 0)
    end
    i, n = 1, length(κ)
    while i <= n
        if abs(κ[i]) > 0.08
            j, sg = i, sign(κ[i])
            while j < n && abs(κ[j+1]) > 0.08 && sign(κ[j+1]) == sg
                j += 1
            end
            push!(corners, (peak=maximum(abs, κ[i:j]), angle=abs(sum(κ[i:j])) * 0.25, len=0.25(j - i + 1)))
            i = j + 1
        else
            i += 1
        end
    end
    return (; κ, w, straights=filter(>(2), straights), corners=filter(c -> c.angle > 0.3, corners), length=tr.length)
end

pct(x, ps) = join((@sprintf("%.2f", quantile(x, p)) for p in ps), " / ")

function main()
    t = build_track(design())
    probs = track_problems(t)
    isempty(probs) || error("the track is unusable:\n" * join(probs, "\n"))
    base = write_track(DIR, NAME, t)
    m = F1Tenth.load_map(base * ".yaml", ".pgm")
    # a picture, at most 2000 pixels across
    img = FileIO.load(base * ".pgm")
    k = max(1, cld(maximum(size(img)), 2000))
    FileIO.save(joinpath(DIR, NAME * ".png"), img[1:k:end, 1:k:end])

    rows = String[]
    real = [geometry(Track(n)) for n in sort(collect(keys(F1Tenth.RACETRACKS)))]
    center = Path(t.x[1:4:end], t.y[1:4:end]; closed=true)
    rep = geometry(Track(NAME, m, center, t.length))
    for (label, g) in (("all 23 racetracks", (κ=reduce(vcat, (r.κ for r in real)), w=reduce(vcat, (r.w for r in real)),
            straights=reduce(vcat, (r.straights for r in real)), corners=reduce(vcat, (r.corners for r in real)),
            length=sum(r.length for r in real))), (NAME, rep))
        pk, an = [c.peak for c in g.corners], [c.angle for c in g.corners]
        push!(rows, @sprintf("| %s | %.0f | %s | %.2f | %s | %d: %s | %s | %s |", label, g.length,
            pct(abs.(g.κ), (0.5, 0.75, 0.9, 0.99)), maximum(abs, g.κ), pct(g.w, (0.01, 0.5, 0.99)),
            length(g.straights), pct(g.straights, (0.5, 0.9, 1.0)), pct(pk, (0.1, 0.5, 0.9, 1.0)), pct(an, (0.5, 0.9, 1.0))))
    end
    segs = join(("| $(l) | $(@sprintf("%.1f", a)) | $(@sprintf("%.1f", b - a)) |" for (a, b, l) in t.segments), "\n")
    mkpath(joinpath(@__DIR__, "results"))
    write(joinpath(@__DIR__, "results", "track.md"), """
        # The representative track

        ![The representative track](../track/$(NAME).png)

        One closed track, $(round(t.length; digits=1)) m long, built from pieces of known curvature and width
        (TrajectorySensitivity.jl's `TrackDesign`; the design is in [`track.jl`](../track.jl)) so that it has
        every kind of situation the 23 f1tenth_racetracks tracks have, each at least once, and nothing they
        don't: a long straight into a hairpin, the median corner, a fast sweeper, a chicane, a tight hairpin, a
        tightening corner and a double apex; the lane as narrow, as wide and as usual as on the real tracks,
        changing from one to the other (the real tracks each keep one width). Left and right turns. The
        same thin walls on free ground as the real maps, 0.05 m per pixel.

        Measured the same way on both (every 0.25 m of the centerline: curvature through points 1 m apart,
        lane width from the map; straights: runs with |κ| < 0.03 1/m longer than 2 m; corners: runs with
        |κ| > 0.08 1/m of one sign turning more than 0.3 rad). The representative track covers the real
        tracks' range; it has a larger share of corners, since each situation is there once and the real
        tracks are mostly straight.

        | Track | Length [m] | \\|κ\\| 50 / 75 / 90 / 99% [1/m] | max \\|κ\\| | width 1 / 50 / 99% [m] | straights: n: length 50 / 90 / 100% [m] | corner peak \\|κ\\| 10 / 50 / 90 / 100% | corner turn 50 / 90 / 100% [rad] |
        |---|---|---|---|---|---|---|---|
        $(join(rows, "\n"))

        Its segments, in driving order from the start line (the situations the results are grouped by):

        | Segment | Starts at [m] | Length [m] |
        |---|---|---|
        $(segs)
        """)
    println(join(rows, "\n"))
    println(segs)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
