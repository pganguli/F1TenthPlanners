# How the estimates' errors add up and interact: every estimate errs at once,
# each with the car network's recorded errors (replayed together, from the
# same recorded scans) times its own gain, drawn uniformly from 0 (true) to
# GAIN_MAX; variance-based (Sobol) indices then split the variance of each
# outcome among the estimates: what each causes alone (first-order index),
# and with its interactions (total index). The replay's draw (its seed) is a
# factor too, so that what the replay's randomness causes is not counted to
# any estimate. Saltelli's design: N (k + 2) runs per course, k = 7 factors.
# Into results/interactions.csv (the representative track alone) or
# interactions_all.csv; report.jl sums them up.
#
#     julia -t auto --project=. interactions.jl                        # the representative track, N = 512
#     julia -t auto --project=. interactions.jl --n 64 --tracks all    # every track, N = 64 each

include(joinpath(@__DIR__, "runs.jl"))

const GAIN_MAX = 2.0
const FACTORS = (ESTIMATES..., :seed)

# The experiment of one row of a sample matrix (factors on [0, 1]).
function experiment(x::AbstractVector)
    gains = Tuple(GAIN_MAX * x[i] for i in eachindex(ESTIMATES))
    return joint_accuracy(gains; seed=1 + floor(Int, x[end] * 2^30))
end

# One run against its reference: the RMS deviation (overall and per class
# of situation, by the reference's class), the least gap, whether it crashed.
function outcome(run::Run, ref::Run, at)
    cmp = compare(run.log, ref.log; distance=distance_driven(ref.log) - 1.0)
    s2, n = zeros(10), zeros(Int, 10)
    for (s, d) in zip(cmp.s, cmp.Δd)
        isfinite(d) || continue
        cl = at(s)
        s2[cl] += d^2
        n[cl] += 1
    end
    rms = sum(n) > 0 ? sqrt(sum(s2) / sum(n)) : NaN
    return (rms=rms, class_rms=[n[c] > 0 ? sqrt(s2[c] / n[c]) : NaN for c in 1:10],
        min_gap=minimum(run.log.gap; init=Inf), crashed=run.log.crashed, km=distance_driven(run.log) / 1000)
end

function parse_interaction_args(args)
    n, tracks = 512, [REPRESENTATIVE]
    i = 1
    while i <= length(args)
        if args[i] == "--n"
            n = parse(Int, args[i+1]); i += 2
        elseif args[i] == "--tracks"
            tracks = args[i+1] == "all" ? vcat(ALL_TRACKS, REPRESENTATIVE) : String.(split(args[i+1], ",")); i += 2
        else
            error("unknown argument $(args[i])")
        end
    end
    return n, tracks
end

function interactions_main(args=ARGS)
    N, names = parse_interaction_args(args)
    k = length(FACTORS)
    ins = inputs()
    F1Tenth.THREAD_AGENTS[] = false
    courses = Dict(n => Course(n) for n in names)
    refs = Dict{String,Tuple{Run,Any}}()
    Threads.@threads for n in names
        r = simulate(reference(), courses[n], ins)
        r.log.crashed && @warn "the reference crashed on $n: left out"
        lock(() -> (refs[n] = (r, class_lookup(r))), CSV_LOCK)
    end
    names = [n for n in names if !refs[n][1].log.crashed]
    # one design per course (seeded by its name), every matrix: A, B, then A with B's column i
    designs = Dict(n => saltelli_design(Xoshiro(hash(n)), N, k) for n in names)
    function matrix(n, m)
        A, B = designs[n]
        return m == 0 ? A : m == -1 ? B : mix(A, B, m)
    end
    tasks = [(n, m, j) for n in names for m in -1:k for j in 1:N]
    rows = Vector{String}(undef, length(tasks))
    done = Threads.Atomic{Int}(0)
    start = time()
    Threads.@threads :dynamic for t in eachindex(tasks)
        n, m, j = tasks[t]
        x = matrix(n, m)[j, :]
        ref, at = refs[n]
        o = outcome(simulate(experiment(x), courses[n], ins), ref, at)
        rows[t] = join((n, m == 0 ? "A" : m == -1 ? "B" : "AB$m", j, (_csv(GAIN_MAX * x[i]) for i in eachindex(ESTIMATES))...,
            experiment(x).seed, _csv(o.rms), (_csv(r) for r in o.class_rms)..., _csv(o.min_gap), o.crashed, _csv(o.km)), ",")
        c = Threads.atomic_add!(done, 1) + 1
        (c % 200 == 0 || c == length(tasks)) && @printf("interactions: %d / %d runs, %.0f s\n", c, length(tasks), time() - start)
        flush(stdout)
    end
    path = joinpath(OUT, names == [REPRESENTATIVE] ? "interactions.csv" : "interactions_all.csv")
    open(path, "w") do io
        println(io, join(("course", "matrix", "row", ("gain_$e" for e in ESTIMATES)..., "seed", "rms_m",
            ("rms_$(replace(c, ' ' => '_'))_m" for c in SITUATIONS)..., "min_gap_m", "crashed", "km"), ","))
        foreach(r -> println(io, r), rows)
    end
    println(path)
end

if abspath(PROGRAM_FILE) == @__FILE__
    interactions_main()
end
