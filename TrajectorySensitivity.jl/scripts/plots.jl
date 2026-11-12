# The plots of the sensitivity runs (results/runs.jld2, from sensitivity.jl),
# into results/*.png:
#
# - accuracy.png: per estimate, how far the trajectory moves (max |Δd|)
#   against the gain on the car network's errors;
# - latency.png: per estimate, max |Δd| against the cloud's delay;
# - along_track.png: per estimate, where on the track the trajectories
#   spread (the tube's width, car network's errors).
#
#     julia --project=. plots.jl

using CairoMakie
using JLD2
using LidarLaneNets: ESTIMATES
using Statistics
using TrajectorySensitivity
using TrajectorySensitivity: _nanmax

const OUT = joinpath(@__DIR__, "results")
const TRACK_DIR = joinpath(@__DIR__, "track")

# The reference palette's first two categorical slots and its text and grid inks (light mode).
const BLUE, ORANGE = colorant"#2a78d6", colorant"#eb6834"
const INK, INK2, GRID = colorant"#0b0b0b", colorant"#52514e", colorant"#e4e3df"
const LABELS = (e="e (cross-track)", ψe="ψe (heading)", κ_profile="κ_profile (curvature ahead)",
    widths="widths (room each side)", sight="sight", clearance="clearance (arc fan)")

theme() = Theme(fontsize=13, textcolor=INK,
    Axis=(xgridcolor=GRID, ygridcolor=GRID, leftspinecolor=INK2, bottomspinecolor=INK2, xtickcolor=INK2,
        ytickcolor=INK2, topspinevisible=false, rightspinevisible=false, titlefont=:regular, titlesize=13))

pct(x, p) = (y = filter(isfinite, x); isempty(y) ? NaN : quantile(y, p))
# a floor for log axes: nothing moved means "less than this"
const FLOOR = 1e-3

function accuracy_plot(ps, comps)
    gains = sort(unique(p.gain for p in ps if p.kind === :accuracy && p.network === :small_weak))
    fig = Figure(size=(960, 620))
    ests = [e for e in ESTIMATES if any(p -> p.kind === :accuracy && p.estimate === e, ps)]
    axes = Axis[]
    for (k, est) in enumerate(ests)
        ax = Axis(fig[cld(k, 3), mod1(k, 3)]; title=LABELS[est], xscale=log2, yscale=log10,
            xticks=(gains, [string(g) for g in gains]), xlabel=k > 3 ? "gain on the car network's errors" : "",
            ylabel=mod1(k, 3) == 1 ? "max |Δd| [m]" : "")
        push!(axes, ax)
        med, lo, hi, crash = Float64[], Float64[], Float64[], Int[]
        for g in gains
            idx = findall(p -> p.kind === :accuracy && p.estimate === est && p.network === :small_weak && p.gain == g, ps)
            v = [max(comps[i].max_Δd, FLOOR) for i in idx]
            push!(med, pct(v, 0.5)); push!(lo, pct(v, 0.1)); push!(hi, pct(v, 0.9))
            push!(crash, count(i -> comps[i].crashed, idx))
        end
        band!(ax, gains, lo, hi; color=(BLUE, 0.18))
        lines!(ax, gains, med; color=BLUE, linewidth=2)
        scatter!(ax, gains, med; color=BLUE, markersize=8, strokecolor=:white, strokewidth=1.5)
        for (g, m, c) in zip(gains, med, crash)
            c > 0 && text!(ax, g, m; text="$c crash", align=(:center, :bottom), offset=(0, 8), fontsize=11, color=INK2)
        end
        idx = findall(p -> p.kind === :accuracy && p.estimate === est && p.network === :large, ps)
        if !isempty(idx)
            v = [max(comps[i].max_Δd, FLOOR) for i in idx]
            scatter!(ax, [1.0], [pct(v, 0.5)]; color=ORANGE, marker=:diamond, markersize=11, strokecolor=:white,
                strokewidth=1.5)
        end
    end
    linkyaxes!(axes...)
    Legend(fig[3, 1:3], [[PolyElement(color=(BLUE, 0.18)), LineElement(color=BLUE, linewidth=2)],
            MarkerElement(color=ORANGE, marker=:diamond, markersize=11)],
        ["car network's errors × gain: median over the error seeds (band: 10th to 90th percentile)",
            "cloud network's errors (gain 1), median"]; orientation=:horizontal, framevisible=false, tellwidth=false)
    Label(fig[0, 1:3], "How far the trajectory moves with each estimate's errors (the others true)"; fontsize=15,
        halign=:left, tellwidth=false)
    save(joinpath(OUT, "accuracy.png"), fig; px_per_unit=2)
end

function latency_plot(ps, comps, logs)
    delays = sort(unique(p.delay for p in ps if p.kind === :latency))
    ests = [e for e in ESTIMATES if any(p -> p.kind === :latency && p.estimate === e, ps)]
    fig = Figure(size=(960, 600))
    axes = Axis[]
    for (k, est) in enumerate(ests)
        ax = Axis(fig[cld(k, 3), mod1(k, 3)]; title=LABELS[est], yscale=log10, xticks=1000 .* delays,
            xlabel=k > 3 ? "cloud delay [ms]" : "", ylabel=mod1(k, 3) == 1 ? "max |Δd| [m]" : "")
        push!(axes, ax)
        vlines!(ax, [100.0]; color=INK2, linestyle=:dash, linewidth=1)
        xs, ys, notes = Float64[], Float64[], Tuple{Float64,String}[]
        for d in delays
            i = findfirst(p -> p.kind === :latency && p.estimate === est && p.delay == d, ps)
            i === nothing && continue
            c, lg = comps[i], logs[i]
            if distance_driven(lg) < 1.0
                push!(notes, (1000d, "no start"))
            elseif c.crashed
                push!(notes, (1000d, "crash"))
                push!(xs, 1000d); push!(ys, max(c.max_Δd, FLOOR))
            elseif !isfinite(c.Δlap)
                push!(notes, (1000d, "too slow"))
                push!(xs, 1000d); push!(ys, max(c.max_Δd, FLOOR))
            else
                push!(xs, 1000d); push!(ys, max(c.max_Δd, FLOOR))
            end
        end
        lines!(ax, xs, ys; color=BLUE, linewidth=2)
        scatter!(ax, xs, ys; color=BLUE, markersize=8, strokecolor=:white, strokewidth=1.5)
        for (x, s) in notes
            text!(ax, x, FLOOR; text=s, rotation=π / 2, align=(:left, :center), offset=(0, 4), fontsize=11, color=INK2)
        end
    end
    linkyaxes!(axes...)
    Label(fig[3, 1:3], "dashed: the selector's max_age (100 ms); later results are dropped. " *
                       "\"too slow\": did not finish the laps in time."; fontsize=12, color=INK2, tellwidth=false)
    Label(fig[0, 1:3], "How far the trajectory moves when each estimate comes only from the cloud, late (the others true)";
        fontsize=15, halign=:left, tellwidth=false)
    save(joinpath(OUT, "latency.png"), fig; px_per_unit=2)
end

function along_plot(tubes, segments, len)
    ests = [e for e in ESTIMATES if haskey(tubes, string((e, :small_weak, 1.0)))]
    fig = Figure(size=(960, 130 * length(ests) + 160))
    corners = [(a, b, l) for (a, b, l) in segments if l != "straight" && l != "long straight"]
    letters = [string(Char('A' + i - 1)) for i in eachindex(corners)]
    # a strip naming the corners, both laps
    strip = Axis(fig[1, 1]; height=22)
    hidedecorations!(strip); hidespines!(strip)
    for lap in 0:1, ((a, b, _), c) in zip(corners, letters)
        vspan!(strip, a + lap * len, b + lap * len; color=(INK2, 0.08))
        text!(strip, (a + b) / 2 + lap * len, 0.5; text=c, align=(:center, :center), fontsize=11, color=INK2)
    end
    ylims!(strip, 0, 1)
    axes = Axis[]
    for (k, est) in enumerate(ests)
        tb = tubes[string((est, :small_weak, 1.0))]
        ax = Axis(fig[k+1, 1]; title=LABELS[est], titlealign=:left,
            xlabel=k == length(ests) ? "distance along the centerline from the start [m] (2 laps)" : "",
            xticklabelsvisible=k == length(ests))
        push!(axes, ax)
        for lap in 0:1, (a, b, _) in corners
            vspan!(ax, a + lap * len, b + lap * len; color=(INK2, 0.08))
        end
        w = replace(tb.width, NaN => 0.0)
        band!(ax, tb.s, zeros(length(tb.s)), w; color=(BLUE, 0.35))
        lines!(ax, tb.s, w; color=BLUE, linewidth=1.5)
    end
    linkaxes!(axes...)
    linkxaxes!(strip, axes...)
    key = join(("$c $(l)" for ((_, _, l), c) in zip(corners, letters)), ", ")
    Label(fig[length(ests)+2, 1], "Width [m] of the band all runs' lateral positions lie in, with the car network's " *
                                  "errors (gain 1). Shaded: corners ($key)."; fontsize=12, color=INK2,
        tellwidth=false, word_wrap=true)
    Label(fig[0, 1], "Where on the track each estimate's errors spread the trajectories"; fontsize=15, halign=:left,
        tellwidth=false)
    save(joinpath(OUT, "along_track.png"), fig; px_per_unit=2)
end

function main()
    d = load(joinpath(OUT, "runs.jld2"))
    ps, logs, ref = d["perturbations"], d["logs"], d["reference"]
    dist = distance_driven(ref) - 1.0   # each run's laps end a little apart
    comps = [compare(lg, ref; distance=dist) for lg in logs]
    segments = read_segments(joinpath(TRACK_DIR, "Representative_segments.csv"))
    len = maximum(seg[2] for seg in segments)
    with_theme(theme()) do
        any(p -> p.kind === :accuracy, ps) && accuracy_plot(ps, comps)
        any(p -> p.kind === :latency, ps) && latency_plot(ps, comps, logs)
        isfile(joinpath(OUT, "tubes.jld2")) && along_plot(load(joinpath(OUT, "tubes.jld2"), "tubes"), segments, len)
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
