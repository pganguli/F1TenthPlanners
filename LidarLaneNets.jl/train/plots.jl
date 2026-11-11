# Two plots of the networks, into results/ as PDF, from the tables that
# evaluate.jl and errors.jl write there (run those first):
#
# - accuracy_vs_macs.pdf: the test RMS error of every network, relative to
#   the car's network (small_weak) for the same estimate, against its MACs
#   per scan;
# - error_distributions.pdf: the Student t fitted to every network's errors,
#   one panel per estimate, as the chance of an error larger than x (both
#   axes logarithmic: a more accurate network sits to the left; heavier tails
#   fall off less steeply, with slope −ν).
#
# Color is the network set (the same in both); marker shape is the estimate.
#
#     julia --project=. plots.jl

using CairoMakie
using LidarLaneNets

include("tdist.jl")   # t_tail

const RESULTS_DIR = joinpath(@__DIR__, "results")

# Categorical slots 1-3 of the reference palette (validated for all pairs);
# ink and grid colors for text and axes.
const SET_COLORS = Dict(:small_weak => colorant"#2a78d6", :small_strong => colorant"#eb6834", :large => colorant"#1baf7a")
const SET_LABELS = Dict(:small_weak => "small_weak (car)", :small_strong => "small_strong", :large => "large (cloud)")
const INK, INK2, GRID, SURFACE = colorant"#0b0b0b", colorant"#52514e", colorant"#e4e3df", colorant"#fcfcfb"
const MARKERS = Dict(:e => :circle, :ψe => :rect, :κ_profile => :utriangle, :widths => :diamond, :sight => :dtriangle,
    :clearance => :pentagon)

# The table rows of a results file, as vectors of cells (the header and rule left out).
function table_rows(file)
    rows = Vector{String}[]
    for l in eachline(joinpath(RESULTS_DIR, file))
        startswith(l, "| `") || continue
        push!(rows, strip.(split(strip(l, '|'), '|')))
    end
    return rows
end

estimate_of(cell) = Symbol(strip(cell, '`'))
number(cell) = parse(Float64, replace(cell, "," => ""))

function accuracy_vs_macs()
    raw = Dict{Tuple{Symbol,Symbol},Tuple{Float64,Float64}}()   # (estimate, set) => (MACs, test RMS error)
    for r in table_rows("networks.md")
        r[2] == "hand-written" && continue
        raw[(estimate_of(r[1]), Symbol(r[2]))] = (number(r[5]), number(r[8]))
    end
    # the error relative to the car's network for the same estimate; each
    # estimate nudged sideways (by up to ±20% in MACs), so markers at the same
    # spot stay apart
    nudge(est) = 1.08^(findfirst(==(est), ESTIMATES) - 3.5)
    pts = Dict(k => (v[1] * nudge(k[1]), v[2] / raw[(k[1], :small_weak)][2]) for (k, v) in raw)
    fig = Figure(size=(760, 470), backgroundcolor=SURFACE, fontsize=13)
    ax = Axis(fig[1, 1]; xscale=log10, xlabel="MACs per scan (one network)",
        ylabel="Test RMS error, relative to small_weak", ytickformat=ys -> ["$(round(Int, 100y))%" for y in ys],
        title="Error against cost, every network (held-out tracks)", titlealign=:left, titlecolor=INK, xlabelcolor=INK2,
        ylabelcolor=INK2, xgridcolor=GRID, ygridcolor=GRID, backgroundcolor=SURFACE,
        xticklabelcolor=INK2, yticklabelcolor=INK2, leftspinecolor=GRID, bottomspinecolor=GRID,
        rightspinevisible=false, topspinevisible=false,
        xticks=([1e4, 1e5, 1e6, 1e7], ["10 k", "100 k", "1 M", "10 M"]))
    for est in ESTIMATES   # each estimate's three networks, joined
        xy = sort([pts[(est, s)] for s in NETWORKS])
        lines!(ax, first.(xy), last.(xy); color=GRID, linewidth=1.5)
    end
    for s in NETWORKS, est in ESTIMATES
        x, y = pts[(est, s)]
        scatter!(ax, [x], [y]; color=SET_COLORS[s], marker=MARKERS[est], markersize=13, strokecolor=SURFACE,
            strokewidth=1.5)
    end
    # the sets, labeled over their clusters
    for s in NETWORKS
        x = exp(sum(log(pts[(est, s)][1]) for est in ESTIMATES) / length(ESTIMATES))
        text!(ax, x, 1.035; text=SET_LABELS[s], align=(:center, :bottom), color=INK, fontsize=12)
    end
    ylims!(ax, 0.54, 1.08)
    xlims!(ax, 8e3, 1.2e7)
    Label(fig[2, 1:2], "small_weak is 100% by definition. Estimates are nudged sideways within a set so markers don't overlap.";
        halign=:left, color=INK2, fontsize=11, tellwidth=false)
    sets = [MarkerElement(color=SET_COLORS[s], marker=:circle, markersize=11) for s in NETWORKS]
    ests = [MarkerElement(color=INK2, marker=MARKERS[e], markersize=11) for e in ESTIMATES]
    Legend(fig[1, 2], [sets, ests], [[SET_LABELS[s] for s in NETWORKS], [string(e) for e in ESTIMATES]],
        ["Network set", "Estimate"]; framevisible=false, labelcolor=INK, titlecolor=INK, gridshalign=:left)
    save(joinpath(RESULTS_DIR, "accuracy_vs_macs.pdf"), fig)
    return fig
end

# The half-width holding probability p of a Student t (by bisection).
function t_halfwidth(s, ν, p)
    lo, hi = 0.0, s
    while t_tail(hi, s, ν) > 1 - p
        hi *= 2
    end
    for _ in 1:60
        mid = (lo + hi) / 2
        t_tail(mid, s, ν) > 1 - p ? (lo = mid) : (hi = mid)
    end
    return hi
end

function error_distributions()
    fits = Dict{Tuple{Symbol,Symbol},NTuple{3,Float64}}()   # (estimate, set) => (μ, s, ν)
    for r in table_rows("errors.md")
        fits[(estimate_of(r[1]), Symbol(r[2]))] = (number(r[3]), number(r[4]), number(r[5]))
    end
    fig = Figure(size=(900, 600), backgroundcolor=SURFACE, fontsize=12)
    Label(fig[0, 1:3], "The networks' errors: chance of an error larger than x (Student t fitted to each)";
        halign=:left, color=INK, fontsize=14)
    levels = [0.5, 0.1, 0.01]   # where each curve carries its marker
    for (k, est) in enumerate(ESTIMATES)
        ax = Axis(fig[(k - 1) ÷ 3 + 1, (k - 1) % 3 + 1]; xscale=log10, yscale=log10, title=string(est),
            titlealign=:left, titlecolor=INK, xlabel="|error| [$(UNITS[est])]",
            ylabel=k % 3 == 1 ? "P(|error| > x)" : "", xlabelcolor=INK2, ylabelcolor=INK2,
            xgridcolor=GRID, ygridcolor=GRID, backgroundcolor=SURFACE, xticklabelcolor=INK2,
            yticklabelcolor=INK2, leftspinecolor=GRID, bottomspinecolor=GRID, rightspinevisible=false,
            topspinevisible=false, yticks=([1e-3, 1e-2, 1e-1, 1], ["0.1%", "1%", "10%", "100%"]))
        # from where 90% of errors are larger, to where 0.1% are, across the three fits
        lo = minimum(t_halfwidth(fits[(est, s)][2], fits[(est, s)][3], 0.10) for s in NETWORKS)
        hi = maximum(t_halfwidth(fits[(est, s)][2], fits[(est, s)][3], 0.999) for s in NETWORKS)
        xs = exp.(range(log(lo), log(hi), length=400))
        for s in NETWORKS
            _, sc, ν = fits[(est, s)]
            lines!(ax, xs, t_tail.(xs, sc, ν); color=SET_COLORS[s], linewidth=2)
            mx = [t_halfwidth(sc, ν, 1 - p) for p in levels]
            scatter!(ax, mx, levels; color=SET_COLORS[s], marker=MARKERS[est], markersize=9, strokecolor=SURFACE,
                strokewidth=1)
        end
        ylims!(ax, 1e-3, 1)
        xlims!(ax, lo, hi)
        ν_text = join((string(round(fits[(est, s)][3]; sigdigits=2)) for s in NETWORKS), " / ")
        text!(ax, 0.03, 0.03; text="ν " * ν_text, space=:relative, align=(:left, :bottom), color=INK2, fontsize=11)
    end
    Legend(fig[3, 1:3], [LineElement(color=SET_COLORS[s], linewidth=2) for s in NETWORKS],
        [SET_LABELS[s] for s in NETWORKS]; orientation=:horizontal, framevisible=false, labelcolor=INK,
        tellheight=true)
    Label(fig[4, 1:3], "Further left: smaller errors. Shallower at the bottom: heavier tails (slope −ν). Markers at " *
                       "50%, 10% and 1%. ν: small_weak / small_strong / large.";
        halign=:left, color=INK2, fontsize=11, tellwidth=false)
    save(joinpath(RESULTS_DIR, "error_distributions.pdf"), fig)
    return fig
end

if abspath(PROGRAM_FILE) == @__FILE__
    accuracy_vs_macs()
    error_distributions()
end
