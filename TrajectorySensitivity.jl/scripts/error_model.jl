# The error models the oracle draws from (TrajectorySensitivity.NetworkErrors),
# for the car's (small_weak) and the cloud's (large) networks, into
# results/error_model.toml, with a check against the networks' real errors in
# results/error_model.md.
#
#     julia -t auto --project=. error_model.jl
#
# Taken as measured in LidarLaneNets.jl/train: weights/fusion.toml, the
# Kalman filter's noise parameters for the car's networks and the tail
# weight of the cloud's (fusion.jl, on the validation tracks).
# Measured here, on the scans errors.jl measures the networks' errors on
# (the held-out tracks' drives; the networks run only here, never in the
# sensitivity runs):
# - the errors' scale per entry (the median |error|; a profile's errors grow
#   with the distance ahead) and their distribution over it, pooled over the
#   entries (the Student t of errors.md fits its bulk, but with ν < 1 for
#   some, too far in its tails to draw from);
# - the errors as Gaussian scores u: their correlation across an estimate's
#   entries; and from scan to scan (persist and τ), fitted so that the
#   errors drawn correlate from scan to scan as the network's do, as
#   fusion.jl measures it (1 to 20 scans apart: the large errors persist
#   more than the small ones, and fusion.jl's correlation, like the
#   trajectory, follows the large);
# - log σ per entry: mean and standard deviation; its correlation c with the
#   error's size; the rest's correlation across entries and over time;
# - how both the errors and σ scale with the situation (TrajectorySensitivity's
#   `situation`: each entry's true value; for e and ψe, the bend beside the
#   car): their medians per bin of it (the 50th, 75th, 90th and 97th
#   percentiles bound the bins);
# - for the cloud's networks, the filter's persist, τ and σ_bias, as
#   fusion.jl measures them for the car's, on the validation tracks.

const TRAIN = joinpath(@__DIR__, "..", "..", "LidarLaneNets.jl", "train")
include(joinpath(TRAIN, "fusion.jl"))   # common.jl, tdist.jl, autocorr, fit_persistence, driving_runs

using LinearAlgebra
using TrajectorySensitivity
using TrajectorySensitivity: size_score, error_entry, _Φ, from_score, err_factor, GaussProcess, NetworkErrors, ErrorDraws, draw!

const OUT_DIR = joinpath(@__DIR__, "results")
const SETS = (:small_weak, :large)

# Correlation of the rows of A over the columns where both are finite; then
# the nearest correlation matrix with eigenvalues at least `floor`.
function masked_corr(A; floor=1e-4)
    n = size(A, 1)
    C = Matrix{Float64}(I, n, n)
    F = isfinite.(A)
    for i in 1:n, j in i+1:n
        both = F[i, :] .& F[j, :]
        count(both) < 30 && continue
        x, y = A[i, both], A[j, both]
        x, y = x .- mean(x), y .- mean(y)
        C[i, j] = C[j, i] = sum(x .* y) / sqrt(sum(abs2, x) * sum(abs2, y))
    end
    E = eigen(Symmetric(C))
    C = E.vectors * Diagonal(max.(E.values, floor)) * E.vectors'
    d = sqrt.(diag(C))
    return Symmetric(C ./ (d * d'))
end

# The process of A's columns over the runs (autocorrelation, as fusion.jl fits it).
function persistence(A, runs)
    M = isfinite.(A)
    B = replace(A, NaN => 0.0)
    ρ = [autocorr(B, M, runs, k) for k in LAGS]
    p, a = fit_persistence(ρ)
    return p, -DT / log(a), ρ
end

q(x, p) = quantile(x, p)

# The situation of every entry of every scan (as TrajectorySensitivity.situation), from the true values.
function situations(est, Y)
    est in (:e, :ψe) && return abs.(Y[:κ_profile][1:1, :])
    est === :sight && return copy(Y[:sight])
    return abs.(Y[est])
end

# Bins of the situation f (bounded by its 0, 25, 50, 75, 90, 97 and 100th
# percentiles, merged where they coincide; the greatest value a bin of its
# own when many share it, as sight and clearance at their caps): each bin's
# median situation, and the medians of r and of ℓ over it.
function situation_bins(f, r, ℓ)
    edges = unique(quantile(f, [0.0, 0.25, 0.5, 0.75, 0.9, 0.97, 1.0]))
    top = maximum(f)
    bins = [k == length(edges) - 1 ? (edges[k] .<= f .< top) : (edges[k] .<= f .< edges[k+1]) for k in 1:length(edges)-1]
    push!(bins, f .>= top)
    at, rs, ls = Float64[], Float64[], Float64[]
    for sel in bins
        count(sel) < 200 && continue
        a = median(f[sel])
        !isempty(at) && a <= at[end] && continue
        push!(at, a); push!(rs, median(r[sel])); push!(ls, median(ℓ[sel]))
    end
    return at, rs, ls
end

function error_persistence(model, persist, τ, M, F, runs; rng=Xoshiro(2))
    g = GaussProcess(correlation(model.u), persist, τ)
    E = fill(NaN, size(M))
    n = size(M, 1)
    for run in runs
        slow, started = zeros(n), false
        for k in run
            u = TrajectorySensitivity._step!(slow, g, DT, started, rng)
            started = true
            E[:, k] = model.s .* err_factor.(Ref(model), F[:, k]) .* from_score.(Ref(model.quantiles), u)
        end
    end
    return persistence(masked(E, M), runs)[3]
end

# (persist, τ) of the scores u whose errors correlate over time as `ρ_real`
# (least squares over the lags; a grid, then refined around the best).
function fit_error_persistence(model, ρ_real, M, F, runs)
    loss(p, τ) = sum(abs2, error_persistence(model, p, τ, M, F, runs) .- ρ_real)
    best = (Inf, 1.0, 0.1)
    for p in 0.2:0.1:1.0, τ in exp.(range(log(0.02), log(3.0); length=12))
        l = loss(p, τ)
        l < best[1] && (best = (l, p, τ))
    end
    _, p0, τ0 = best
    for p in clamp.(p0 .+ (-0.08:0.02:0.08), 0.05, 1.0), τ in τ0 .* exp.(range(-0.4, 0.4; length=9))
        l = loss(p, τ)
        l < best[1] && (best = (l, p, τ))
    end
    return best[2], best[3]
end
masked(A, M) = ifelse.(M, A, NaN)

function main()
    fusion = TOML.parsefile(joinpath(WEIGHTS_DIR, "fusion.toml"))
    Xte, Yte, fte = load_all(TEST_TRACKS)
    Xva, Yva, fva = load_all(VAL_TRACKS)
    runs_te, runs_va = driving_runs(fte), driving_runs(fva)
    drive_va = reduce(vcat, runs_va)
    out = Dict{String,Any}()
    rows = String[]
    rng = Xoshiro(1)
    for name in SETS
        section = Dict{String,Any}()
        for est in ESTIMATES
            net = load_net(weights_file(WEIGHTS_DIR, est, name))
            # the generator, on the held-out tracks' drives
            μ, σ = means_sigmas(net, Xte)
            M = mask(est, Yte)
            E = μ .- Yte[est]
            n = size(E, 1)
            drive = reduce(vcat, runs_te)
            Ed, Md = E[:, drive], M[:, drive]
            s_j = [count(Md[j, :]) >= 30 ? median(abs.(Ed[j, :][Md[j, :]])) : NaN for j in 1:n]
            for j in 2:n   # an entry hardly ever seen: as the one before it
                isfinite(s_j[j]) || (s_j[j] = s_j[j-1])
            end
            L = log.(σ)
            m = [mean(L[j, drive][Md[j, :]]) for j in 1:n]
            # the situation: how the errors' size and σ scale with it
            F = situations(est, Yte)
            at, a, logb = situation_bins(F[:, drive][Md], (abs.(Ed)./s_j)[Md], (L[:, drive].-m)[Md])
            b = exp.(logb)
            scale(f) = TrajectorySensitivity._pwl(at, a, f)
            lscale(f) = TrajectorySensitivity._pwl(at, logb, f)
            A = s_j .* scale.(F)
            quantiles = quantile((Ed./A[:, drive])[Md], _Φ.(SCORES))
            U = masked(to_score.(Ref(quantiles), E ./ A), M)
            Lr = L .- m .- lscale.(F)
            sd = [max(std(Lr[j, drive][Md[j, :]]), 1e-6) for j in 1:n]
            H = masked(Lr ./ sd, M)
            G = size_score.(U)
            ok = isfinite.(G[:, drive])
            c = clamp(cor(G[:, drive][ok], H[:, drive][ok]), -0.99, 0.99)
            Ξ = (H .- c .* G) ./ sqrt(1 - c^2)
            u_corr, ξ_corr = masked_corr(U[:, drive]), masked_corr(Ξ[:, drive])
            u_p, u_τ, _ = persistence(U, runs_te)
            ξ_p, ξ_τ, _ = persistence(Ξ, runs_te)

            # what the Kalman filter is given for this network
            filter = if name === :small_weak
                s = fusion[String(name)][file_name(est)]
                EstimateNoise(persist=s["persist"], τ=s["tau"], σ_bias=s["sigma_bias"], ν_small=s["nu"],
                    ν_large=fusion["large"][file_name(est)]["nu"])
            else   # as fusion.jl measures the car's networks
                μv, _ = means_sigmas(net, Xva)
                Ev, Mv = μv .- Yva[est], mask(est, Yva)
                p, τ, _ = persistence(masked(Ev, Mv), runs_va)
                σ_bias = sqrt(p * mean(abs2, Ev[:, drive_va][Mv[:, drive_va]]))
                ν = fusion["large"][file_name(est)]["nu"]
                EstimateNoise(persist=p, τ=τ, σ_bias=σ_bias, ν_small=ν, ν_large=ν)
            end
            build(p, τ) = NetworkErrors(est, name, s_j, at, a, b, quantiles, GaussProcess(u_corr, p, τ), m, sd, c,
                GaussProcess(ξ_corr, ξ_p, ξ_τ), filter)
            _, _, ρ_real = persistence(masked(E, M), runs_te)
            u_p, u_τ = fit_error_persistence(build(u_p, u_τ), ρ_real, M, F, runs_te)
            model = build(u_p, u_τ)
            @printf("%-10s %-10s situation %s: error ×%s, σ ×%s\n", name, est, join((@sprintf("%.3g", x) for x in at), "/"),
                join((@sprintf("%.2g", x) for x in a), "/"), join((@sprintf("%.2g", x) for x in b), "/"))
            @printf("%-10s %-10s u: persist %.2f τ %.3f s   ξ: persist %.2f τ %.3f s   c %.2f\n", name, est, u_p, u_τ, ξ_p, ξ_τ, c)
            section[file_name(est)] = error_entry(model)

            # check: the model, drawn over the same drives and entries, against the real errors
            Es, Σs = fill(NaN, size(E)), fill(NaN, size(E))
            for run in runs_te
                d = ErrorDraws(model, rng)
                for k in run
                    e, s = draw!(d, DT, F[:, k])
                    Es[:, k], Σs[:, k] = e, s
                end
            end
            real_e, sim_e = abs.(E[:, drive][Md]), abs.(Es[:, drive][Md])
            fd = F[:, drive][Md]
            lo, hi = quantile(fd, 0.5), quantile(fd, 0.97)
            calm = lo < maximum(fd) ? fd .<= lo : fd .< maximum(fd)      # sight: mostly at its cap
            hard = hi > minimum(fd) ? fd .>= hi : fd .<= minimum(fd)
            est === :sight && ((calm, hard) = (hard, calm))   # a short sight is the hard one
            real_in = mean(abs.(E[:, drive][Md]) .< σ[:, drive][Md])
            sim_in = mean(abs.(Es[:, drive][Md]) .< Σs[:, drive][Md])
            _, _, ρr = persistence(masked(E, M), runs_te)
            _, _, ρs = persistence(masked(Es, M), runs_te)
            # neighbouring entries (profiles, fan): correlation of the errors 1 m (4 entries) apart
            near(A) = n < 5 ? NaN : mean(masked_corr(A[:, drive])[j, j+4] for j in 1:n-4 if !(est === :widths && j <= 33 < j + 4))
            row = @sprintf("| `%s` | %s | %.3g / %.3g | %.3g / %.3g | %.3g / %.3g | %.3g / %.3g | %.3g / %.3g | %.3g / %.3g | %.0f%% / %.0f%% | %.2f / %.2f | %.2f / %.2f | %s |",
                est, name, q(real_e, 0.5), q(sim_e, 0.5), q(real_e, 0.9), q(sim_e, 0.9), q(real_e, 0.99), q(sim_e, 0.99),
                median(real_e[calm]), median(sim_e[calm]), median(real_e[hard]), median(sim_e[hard]),
                sqrt(mean(abs2, real_e)), sqrt(mean(abs2, sim_e)), 100real_in, 100sim_in, ρr[1], ρs[1], ρr[20], ρs[20],
                n < 5 ? "–" : @sprintf("%.2f / %.2f", near(masked(E, M)),
                    near(masked(Es, M))))
            push!(rows, row)
            println(row); flush(stdout)
        end
        out[String(name)] = section
    end
    mkpath(OUT_DIR)
    open(io -> TOML.print(io, out; sorted=true), joinpath(OUT_DIR, "error_model.toml"), "w")
    write(joinpath(OUT_DIR, "error_model.md"), """
        # The error models against the networks' real errors

        Each network's errors as the oracle draws them (TrajectorySensitivity.jl's `NetworkErrors`,
        parameters in [error_model.toml](error_model.toml)) against the network's real errors, over the
        same drives on the held-out tracks ($(join(TEST_TRACKS, ", "))) and the same entries (profiles: those
        the car can see). The model's errors are drawn from scratch (stationary from the start of each
        drive), not replayed. Each cell: real / model. |error| quantiles and RMS in the estimate's unit;
        "calm" and "hard": the median |error| in the calmest half of the situations and the hardest 3%
        (`situation`: each entry's true value, for e and ψe the bend beside the car; for κ_profile,
        straight against sharply bending lane); "within 1σ": how often the error is smaller than the
        network's own σ; ρ: the errors' correlation
        1 and 20 scans apart (25 ms and 0.5 s); "1 m apart": their correlation between entries 1 m apart
        along a profile, or 4 arcs apart on the clearance fan. From [`error_model.jl`](../error_model.jl).

        | Estimate | Network | median | 90% | 99% | calm, median | hard, median | RMS | within 1σ | ρ 1 scan | ρ 20 scans | 1 m apart |
        |---|---|---|---|---|---|---|---|---|---|---|---|
        $(join(rows, "\n"))
        """)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
