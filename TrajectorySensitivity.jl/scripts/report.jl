# Sums up the campaigns (sensitivity.jl, cloud.jl, calibrate.jl) with 95%
# confidence intervals, into results/sensitivity.md, cloud.md and
# calibration.md, and their plots.
#
#     julia --project=. report.jl
#
# Per experiment and kind of situation, over its runs (tracks × seeds):
# crashes per km (exact Poisson interval), the RMS lateral deviation from the
# track's reference (mean over runs), the least gap to a wall (mean over
# runs), the share of the time within 0.2 m of a wall and of the scans with
# time to collision under 0.5 s; and the tube (the band a group's runs lie
# in: its greatest width, mean over tracks). Intervals: percentile bootstrap
# over runs (2000 resamples).

using CairoMakie
using Printf
using Random
using SpecialFunctions: gamma_inc_inv
using Statistics
using TrajectorySensitivity: SITUATIONS, sobol_indices
using LidarLaneNets: ESTIMATES

const OUT = joinpath(@__DIR__, "results")
const BOOT = 2000

# --- reading

function read_rows(name)
    path = joinpath(OUT, name * ".csv")
    isfile(path) || return nothing
    lines = readlines(path)
    head = Symbol.(split(lines[1], ","))
    num(x) = isempty(x) ? NaN : something(tryparse(Float64, x), NaN)
    rows = map(lines[2:end]) do l
        v = split(l, ",")
        NamedTuple{Tuple(head)}(Tuple(h in (:course, :kind, :estimate, :network, :source, :class) ? String(x) :
                                      h === :crashed ? x == "true" : num(x) for (h, x) in zip(head, v)))
    end
    # a course whose reference (every estimate true) crashes says nothing about any estimate
    bad = sort(unique(r.course for r in rows if r.kind == "none" && r.crashed))
    DROPPED[name] = bad
    return filter(r -> !(r.course in bad), rows)
end

const DROPPED = Dict{String,Vector{String}}()

dropped_note(name) = isempty(get(DROPPED, name, String[])) ? "" :
    "Left out: $(join(DROPPED[name], ", ")), where the car crashes even with every estimate true and on time."

# an experiment: everything but the course, the seed and the class
key(r) = (r.kind, r.estimate, r.network, r.gain, r.delay_ms, r.source)
runid(r) = (r.course, r.seed)

# --- statistics

function boot_ci(f, units; rng=Xoshiro(1))
    isempty(units) && return (NaN, NaN, NaN)
    est = f(units)
    bs = [f(units[rand(rng, eachindex(units), length(units))]) for _ in 1:BOOT]
    bs = filter(isfinite, bs)
    isempty(bs) && return (est, NaN, NaN)
    return (est, quantile(bs, 0.025), quantile(bs, 0.975))
end

# Crashes per km over `km` km: the rate and its exact 95% interval.
function poisson_ci(k, km)
    km <= 0 && return (NaN, NaN, NaN)
    chi2(p, ν) = 2gamma_inc_inv(ν / 2, p, 1 - p)
    lo = k == 0 ? 0.0 : chi2(0.025, 2k) / 2
    hi = chi2(0.975, 2k + 2) / 2
    return (k / km, lo / km, hi / km)
end

"""
    summary(rows) -> NamedTuple

The measures over `rows` (the run rows of one experiment, any classes),
per run first (a run's rows summed), then over runs.
"""
function summary(rows)
    runs = Dict{Any,Vector{Any}}()
    for r in rows
        isnan(r.distance_m) && continue   # a tube row
        push!(get!(runs, runid(r), Any[]), r)
    end
    units = map(collect(values(runs))) do rs
        (dist=sum(r.distance_m for r in rs), time=sum(r.time_s for r in rs), scans=sum(r.scans for r in rs),
            gap=minimum(r.min_gap_m for r in rs), close=sum(r.close_s for r in rs), lowttc=sum(r.low_ttc_scans for r in rs),
            dd2=sum(r.sum_dd2 for r in rs), ndd=sum(r.n_dd for r in rs), crashes=count(r -> r.crashed, rs))
    end
    km = sum((u.dist for u in units); init=0.0) / 1000
    crashes = sum((u.crashes for u in units); init=0)
    rms(us) = mean(sqrt(u.dd2 / u.ndd) for u in us if u.ndd > 0)
    gap(us) = mean(u.gap for u in us if isfinite(u.gap))
    close(us) = sum(u.close for u in us) / max(sum(u.time for u in us), eps())
    lowttc(us) = sum(u.lowttc for u in us) / max(sum(u.scans for u in us), 1)
    return (n=length(units), km, crashes, crash=poisson_ci(crashes, km), rms=boot_ci(rms, units), gap=boot_ci(gap, units),
        close=boot_ci(close, units), lowttc=boot_ci(lowttc, units), units)
end

function tube_summary(rows)
    w = [r.tube_max_m for r in rows if !isnan(r.tube_max_m)]
    return boot_ci(mean, w)
end

# --- formatting

g(x; d=2) = isfinite(x) ? string(round(x; sigdigits=d)) : "–"
ci(t; d=2, scale=1.0) = isfinite(t[1]) ? "$(g(scale * t[1]; d)) ($(g(scale * t[2]; d))–$(g(scale * t[3]; d)))" : "–"

select(rows, f) = filter(f, rows)
in_class(rows, c) = filter(r -> r.class == c, rows)

# =====================================================================
# calibration

function calibration_report(rows)
    lines = String[]
    sources = ("network" => "real network", "replay" => "oracle, recorded errors", "parametric" => "oracle, error model")
    for net in ("small_weak", "large")
        push!(lines, """

            ### $(net == "small_weak" ? "The car's networks (small_weak)" : "The cloud's networks (large), on the car without delay")

            | Estimate | Errors from | Runs | Crashes per km | RMS deviation [m] | Least gap [m] | Scans with TTC < 0.5 s [%] |
            |---|---|---|---|---|---|---|""")
        for est in ESTIMATES, (src, label) in sources
            rs = select(rows, r -> r.estimate == String(est) && r.network == net && r.source == src && r.kind != "none")
            isempty(rs) && continue
            s = summary(rs)
            push!(lines, "| `$est` | $label | $(s.n) | $(ci(s.crash)) | $(ci(s.rms)) | $(ci(s.gap)) | $(ci(s.lowttc; scale=100)) |")
        end
    end
    tracks = unique(r.course for r in rows)
    write(joinpath(OUT, "calibration.md"), """
        # The oracle against the real networks

        Each estimate from the real network on the car (the others true), against the oracle with that
        network's recorded errors (replayed by the true lane, never from the run's own track: what
        sensitivity.jl and cloud.jl use) and with its modeled errors (error_model.jl), on the tracks the
        networks never trained on and the representative track ($(join(tracks, ", "))), 1 lap each (2 on the
        representative track), $(Int(maximum(r.seed for r in rows))) seeds (the real networks: the LiDAR's noise;
        the oracle: its draws). From [`calibrate.jl`](../calibrate.jl), summed up by [`report.jl`](../report.jl);
        95% intervals in brackets. The oracle is right where its intervals overlap the real network's.
        $(dropped_note("calibration"))
        $(join(lines, "\n"))
        """)
end

# =====================================================================
# sensitivity

function sensitivity_report(rows)
    lines = String[]
    gains = sort(unique(r.gain for r in rows if r.kind == "accuracy" && r.network == "small_weak"))
    delays = sort(unique(r.delay_ms for r in rows if r.kind == "latency"))
    # the 12 numbers: slopes, bootstrapped over runs (accuracy) or courses (latency)
    push!(lines, """
        ## Summary: the sensitivities

        Accuracy: how much the RMS deviation from the reference grows per unit of gain on the car network's
        errors (least squares over gains 0.5 to 2, through no deviation at gain 0; gain 1 is the car network
        itself). Latency: how much it grows per 25 ms (one scan) the estimate comes late from the cloud
        (least squares over 0 to 100 ms, from its value on time). Over every track and seed; 95% intervals
        (bootstrap over runs, or over tracks for latency, which has one run per track).

        | Estimate | Accuracy [m per gain] | Latency [m per 25 ms] |
        |---|---|---|""")
    rng = Xoshiro(2)
    for est in ESTIMATES
        acc = select(rows, r -> r.kind == "accuracy" && r.estimate == String(est) && r.network == "small_weak" && r.gain <= 2)
        per_run = Dict{Any,Vector{Tuple{Float64,Float64}}}()
        for k in unique(key.(acc)), (id, rs) in group_runs(filter(r -> key(r) == k, acc))
            push!(get!(per_run, id, Tuple{Float64,Float64}[]), (k[4], run_rms(rs)))
        end
        units = [v for v in values(per_run) if length(v) >= 2]
        slope(us) = (pts = reduce(vcat, us); sum(x * y for (x, y) in pts) / sum(x^2 for (x, _) in pts))
        a = boot_ci(slope, units; rng)
        lat = select(rows, r -> r.kind == "latency" && r.estimate == String(est) && r.delay_ms <= 100)
        per_course = Dict{String,Vector{Tuple{Float64,Float64}}}()
        for k in unique(key.(lat)), (id, rs) in group_runs(filter(r -> key(r) == k, lat))
            push!(get!(per_course, id[1], Tuple{Float64,Float64}[]), (k[5] / 25, run_rms(rs)))
        end
        lunits = collect(values(per_course))
        lslope(us) = (pts = reduce(vcat, us); x0 = mean(y for (x, y) in pts if x == 0);
                      sum(x * (y - x0) for (x, y) in pts) / sum(x^2 for (x, _) in pts))
        l = boot_ci(lslope, lunits; rng)
        push!(lines, "| `$est` | $(ci(a; d=2)) | $(ci(l; d=2)) |")
    end

    # accuracy, overall
    push!(lines, """

        ## Accuracy, over every situation

        | Estimate | Errors | Gain | Crashes per km | RMS deviation [m] | Least gap [m] | Near a wall [% of time] | TTC < 0.5 s [% of scans] | Tube, greatest width [m] |
        |---|---|---|---|---|---|---|---|---|""")
    for est in ESTIMATES, (net, gs) in (("small_weak", gains), ("large", [1.0])), k in gs
        rs = select(rows, r -> r.kind == "accuracy" && r.estimate == String(est) && r.network == net && r.gain == k)
        isempty(rs) && continue
        s = summary(rs)
        push!(lines, "| `$est` | $net | $(g(k)) | $(ci(s.crash)) | $(ci(s.rms)) | $(ci(s.gap)) | $(ci(s.close; scale=100)) | $(ci(s.lowttc; scale=100)) | $(ci(tube_summary(rs))) |")
    end

    # accuracy by situation, gain 1
    push!(lines, """

        ## Accuracy, by situation (the car network's errors, gain 1)

        RMS deviation from the reference [m], and crashes per km where any: per estimate (rows) and kind of
        situation (columns).

        | Estimate | $(join(SITUATIONS, " | ")) |
        |---|$(repeat("---|", length(SITUATIONS)))""")
    for est in ESTIMATES
        rs = select(rows, r -> r.kind == "accuracy" && r.estimate == String(est) && r.network == "small_weak" && r.gain == 1)
        cells = map(SITUATIONS) do c
            cr = in_class(rs, c)
            isempty(cr) && return "–"
            s = summary(cr)
            ci(s.rms) * (s.crashes > 0 ? "; **$(g(s.crash[1])) crashes/km**" : "")
        end
        push!(lines, "| `$est` | $(join(cells, " | ")) |")
    end

    # latency
    push!(lines, """

        ## Latency, over every situation

        The estimate true but only from the cloud, late; one run per track (it is deterministic). Past 100 ms
        the selector drops every result, and the estimate never comes at all: without `e` or `ψe` the car
        never starts (0 km driven); without the others it drives on the filter's start values.

        | Estimate | Delay [ms] | Driven [km] | Crashes per km | RMS deviation [m] | Least gap [m] | TTC < 0.5 s [% of scans] | Tracks without a crash |
        |---|---|---|---|---|---|---|---|""")
    for est in ESTIMATES, d in delays
        rs = select(rows, r -> r.kind == "latency" && r.estimate == String(est) && r.delay_ms == d)
        isempty(rs) && continue
        s = summary(rs)
        fin = count(u -> u.crashes == 0, s.units)
        push!(lines, "| `$est` | $(g(d; d=3)) | $(g(s.km)) | $(ci(s.crash)) | $(ci(s.rms)) | $(ci(s.gap)) | $(ci(s.lowttc; scale=100)) | $fin / $(s.n) |")
    end
    courses = unique(r.course for r in rows)
    write(joinpath(OUT, "sensitivity.md"), """
        # Trajectory and safety sensitivity to each estimate

        One estimate changed at a time, the others true and on time, on $(length(courses)) tracks (the racetracks and
        the representative one), $(Int(maximum(r.seed for r in rows))) seeds, through the whole pipeline (Kalman filter,
        selector, planners). Errors are the networks' own, recorded and replayed by the true lane (see
        [calibration.md](calibration.md) for how they compare with the real networks in the loop). From
        [`sensitivity.jl`](../sensitivity.jl), summed up by [`report.jl`](../report.jl); 95% intervals in brackets.
        RMS deviation: of the car's lateral position from the reference run's (every estimate true and on
        time), at the same distance along the center line. $(dropped_note("sensitivity"))

        $(join(lines, "\n"))
        """)
end

group_runs(rs) = (d = Dict{Any,Vector{Any}}(); foreach(r -> isnan(r.distance_m) || push!(get!(d, runid(r), Any[]), r), rs); d)
run_rms(rs) = (n = sum(r.n_dd for r in rs); n > 0 ? sqrt(sum(r.sum_dd2 for r in rs) / n) : NaN)

# =====================================================================
# cloud

function cloud_report(rows)
    delays = sort(unique(r.delay_ms for r in rows if r.kind == "cloud"))
    lines = String[]
    benefit = Dict{Tuple{Symbol,String},Float64}()
    for est in ESTIMATES
        alone = select(rows, r -> r.kind == "accuracy" && r.estimate == String(est))
        push!(lines, """

            ### `$est`

            Per kind of situation: crashes per km and the least gap to a wall [m], the car's network alone and
            with the cloud's at each delay; and what a call at $(g(delays[min(2, end)]; d=3)) ms changes the least gap by
            (positive: farther from the walls).

            | Situation | Car alone: crashes/km | least gap | $(join(("Cloud $(g(d; d=3)) ms: crashes/km | least gap" for d in delays), " | ")) | Gap change, cloud $(g(delays[min(2, end)]; d=3)) ms |
            |---|---|---|$(repeat("---|---|", length(delays)))---|""")
        for c in vcat(SITUATIONS, "all")
            ac = c == "all" ? alone : in_class(alone, c)
            isempty(ac) && continue
            sa = summary(ac)
            cells = String[ci(sa.crash), ci(sa.gap)]
            diff = (NaN, NaN, NaN)
            for d in delays
                cr = select(rows, r -> r.kind == "cloud" && r.estimate == String(est) && r.delay_ms == d && (c == "all" || r.class == c))
                sc = summary(cr)
                push!(cells, ci(sc.crash), ci(sc.gap))
                if d == delays[min(2, end)]
                    # the difference in mean least gap, bootstrapped over both sets of runs
                    rng = Xoshiro(3)
                    ga = [u.gap for u in sa.units if isfinite(u.gap)]
                    gc = [u.gap for u in sc.units if isfinite(u.gap)]
                    if !isempty(ga) && !isempty(gc)
                        bs = [mean(gc[rand(rng, eachindex(gc), length(gc))]) - mean(ga[rand(rng, eachindex(ga), length(ga))]) for _ in 1:BOOT]
                        diff = (mean(gc) - mean(ga), quantile(bs, 0.025), quantile(bs, 0.975))
                        c != "all" && (benefit[(est, c)] = diff[1])
                    end
                end
            end
            push!(lines, "| $(c == "all" ? "**all**" : c) | $(join(cells, " | ")) | $(ci(diff)) |")
        end
    end
    write(joinpath(OUT, "cloud.md"), """
        # What calling the cloud for one estimate buys

        Each estimate from the car's network alone (its recorded errors), against the car's and the cloud's
        together (the cloud's errors from the same recorded scans, the result arriving a delay after its scan,
        fused by the Kalman filter as on the car); the other estimates true and on time. On $(length(unique(r.course for r in rows)))
        tracks, $(Int(maximum(r.seed for r in rows))) seeds. From [`cloud.jl`](../cloud.jl), summed up by
        [`report.jl`](../report.jl); 95% intervals in brackets. $(dropped_note("cloud")) A schedule would call
        the cloud for an estimate where this buys the most:

        ![How much farther from the walls a cloud call keeps the car](cloud_benefit.png)
        $(join(lines, "\n"))
        """)
    return benefit, delays[min(2, end)]
end

function benefit_plot(benefit, delay)
    isempty(benefit) && return
    M = [get(benefit, (est, c), NaN) for est in ESTIMATES, c in SITUATIONS]
    lim = maximum(abs, filter(isfinite, M); init=0.01)
    cmap = cgrad([colorant"#e34948", colorant"#f0efec", colorant"#2a78d6"])
    fig = Figure(size=(980, 420))
    ax = Axis(fig[1, 1]; xticks=(1:length(SITUATIONS), SITUATIONS), yticks=(1:length(ESTIMATES), collect(String.(ESTIMATES))),
        xticklabelrotation=π / 6, yreversed=true,
        title="How much farther from the walls a $(g(delay; d=3)) ms cloud call keeps the car (change in mean least gap, m)",
        titlealign=:left, titlefont=:regular)
    hm = heatmap!(ax, permutedims(M); colormap=cmap, colorrange=(-lim, lim), nan_color=:white)
    for i in eachindex(ESTIMATES), j in eachindex(SITUATIONS)
        isfinite(M[i, j]) && text!(ax, j, i; text=g(M[i, j]), align=(:center, :center), fontsize=11, color=colorant"#0b0b0b")
    end
    Colorbar(fig[1, 2], hm; label="blue: safer with the cloud; red: less safe")
    save(joinpath(OUT, "cloud_benefit.png"), fig; px_per_unit=2)
end

# --- interactions (interactions.jl): Sobol indices

const FACTOR_NAMES = (String.(ESTIMATES)..., "seed")
const INTER_BOOT = 500

# Per course: the outputs `col` on A, B and each A-with-B's-column (rows in order).
function sobol_groups(rows, col)
    groups = []
    for c in unique(r["course"] for r in rows)
        rs = filter(r -> r["course"] == c, rows)
        y(m) = (v = [(parse(Int, r["row"]), col(r)) for r in rs if r["matrix"] == m]; sort!(v); last.(v))
        A, B = y("A"), y("B")
        AB = reduce(hcat, (y("AB$i") for i in eachindex(FACTOR_NAMES)))
        length(A) == length(B) == size(AB, 1) && push!(groups, (A, B, AB))
    end
    return groups
end

# Pooled over courses (each course's indices weighted by its variance and size).
function pooled_indices(groups)
    k = length(FACTOR_NAMES)
    num1, numT, den = zeros(k), zeros(k), 0.0
    for (A, B, AB) in groups
        S, ST, V = sobol_indices(A, B, AB)
        (isfinite(V) && V > 0) || continue
        w = length(A) * V
        num1 .+= w .* S; numT .+= w .* ST; den += w
    end
    return den > 0 ? (num1 ./ den, numT ./ den) : (fill(NaN, k), fill(NaN, k))
end

# The indices with 95% intervals: bootstrap over the rows of each course's design.
function sobol_ci(groups; rng=Xoshiro(1))
    S, ST = pooled_indices(groups)
    bs = map(1:INTER_BOOT) do _
        pooled_indices([(idx = rand(rng, eachindex(A), length(A)); (A[idx], B[idx], AB[idx, :])) for (A, B, AB) in groups])
    end
    q(v, i, p) = (x = filter(isfinite, [b[v][i] for b in bs]); isempty(x) ? NaN : quantile(x, p))
    return [(S[i], q(1, i, 0.025), q(1, i, 0.975)) for i in eachindex(S)],
           [(ST[i], q(2, i, 0.025), q(2, i, 0.975)) for i in eachindex(ST)]
end

function read_csv_dicts(name)
    path = joinpath(OUT, name * ".csv")
    isfile(path) || return nothing
    lines = readlines(path)
    head = split(lines[1], ",")
    return [Dict(zip(head, split(l, ","))) for l in lines[2:end]]
end

num(x) = something(tryparse(Float64, x), NaN)

function interactions_section(rows, title)
    ncourse = length(unique(r["course"] for r in rows))
    N = maximum(parse(Int, r["row"]) for r in rows)
    outputs = [("RMS deviation", r -> num(r["rms_m"])), ("Least gap to a wall", r -> num(r["min_gap_m"])),
        ("Crashed", r -> r["crashed"] == "true" ? 1.0 : 0.0)]
    lines = String["""

        ## $title

        $ncourse course$(ncourse > 1 ? "s (indices pooled, each course weighted by its variance)" : ""), $N samples
        each ($(length(rows)) runs). First-order index: the share of the outcome's variance an estimate's
        errors cause alone; total: with all their interactions; interactions: the difference. `seed`: the
        replay's randomness (which recorded errors come when), not an estimate.
        """]
    for (name, col) in outputs
        groups = sobol_groups(rows, col)
        allv = reduce(vcat, (vcat(A, B) for (A, B, _) in groups))
        if !any(x -> x != allv[1], allv)
            push!(lines, "\n### $name\n\nNo variance: always $(g(allv[1])).")
            continue
        end
        S, ST = sobol_ci(groups)
        push!(lines, """

            ### $name

            | Factor | First-order | Total | Interactions |
            |---|---|---|---|""")
        for (i, f) in enumerate(FACTOR_NAMES)
            push!(lines, "| `$f` | $(ci(S[i])) | $(ci(ST[i])) | $(g(ST[i][1] - S[i][1])) |")
        end
        push!(lines, "| sum | $(g(sum(x[1] for x in S))) | $(g(sum(x[1] for x in ST))) | |")
    end
    # crashes: the two estimates whose errors interact the most, crash rates by both their gains
    S, ST = pooled_indices(sobol_groups(rows, r -> r["crashed"] == "true" ? 1.0 : 0.0))
    if all(isfinite, ST)
        top = sort(1:length(ESTIMATES); by=i -> -(ST[i] - S[i]))[1:2]
        a, b = String(ESTIMATES[top[1]]), String(ESTIMATES[top[2]])
        ab = filter(r -> r["matrix"] in ("A", "B"), rows)
        bins = [(0.0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.0)]
        cell(lo, hi, low) = begin
            rs = filter(r -> lo <= num(r["gain_$a"]) < hi && (num(r["gain_$b"]) <= 1.0) == low, ab)
            isempty(rs) ? "–" : "$(round(Int, 100count(r -> r["crashed"] == "true", rs) / length(rs)))% ($(length(rs)))"
        end
        push!(lines, """

            ### Crashes: `$a` and `$b` together

            The two estimates whose errors interact the most in crashes (total less first-order index). Share of
            runs that crashed (of how many), by the gain on `$a`'s errors, and whether `$b`'s are at most the car
            network's (gain ≤ 1) or larger; the other estimates' gains anywhere from 0 to 2.

            | `$a` gain | `$b` gain ≤ 1 | `$b` gain > 1 |
            |---|---|---|""")
        for (lo, hi) in bins
            push!(lines, "| $(lo)–$(hi) | $(cell(lo, hi, true)) | $(cell(lo, hi, false)) |")
        end
    end
    # per kind of situation: the total index of each estimate for the RMS deviation there
    push!(lines, """

        ### RMS deviation, by situation: total indices

        | Factor | $(join(SITUATIONS, " | ")) |
        |---|$(repeat("---|", length(SITUATIONS)))""")
    cols = map(SITUATIONS) do c
        key = "rms_$(replace(c, ' ' => '_'))_m"
        pooled_indices(sobol_groups(rows, r -> num(r[key])))[2]
    end
    for (i, f) in enumerate(FACTOR_NAMES)
        push!(lines, "| `$f` | $(join((g(cols[c][i]) for c in eachindex(SITUATIONS)), " | ")) |")
    end
    return join(lines, "\n")
end

function interactions_report()
    rep, all = read_csv_dicts("interactions"), read_csv_dicts("interactions_all")
    rep === nothing && all === nothing && return
    parts = String[]
    rep === nothing || push!(parts, interactions_section(rep, "The representative track"))
    all === nothing || push!(parts, interactions_section(all, "Every track"))
    write(joinpath(OUT, "interactions.md"), """
        # How the estimates' errors add up and interact

        Every estimate errs at once: the car network's recorded errors, replayed together (from the same
        recorded scans, so they err together as the networks do), each times its own gain, drawn uniformly
        from 0 (true) to 2 (twice the car network's). Variance-based (Sobol) indices split each outcome's
        variance among the estimates (Saltelli's design; Saltelli 2010's first-order and Jansen's total
        estimators). From [`interactions.jl`](../interactions.jl), summed up by [`report.jl`](../report.jl);
        95% intervals in brackets (bootstrap over the samples). A sum of first-order indices well under 1,
        or totals well over the first-order ones, means the estimates' errors interact.
        $(join(parts, "\n"))
        """)
end

function main()
    cal = read_rows("calibration")
    cal === nothing || calibration_report(cal)
    sen = read_rows("sensitivity")
    sen === nothing || sensitivity_report(sen)
    cld = read_rows("cloud")
    if cld !== nothing
        benefit, d = cloud_report(cld)
        benefit_plot(benefit, d)
    end
    interactions_report()
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
