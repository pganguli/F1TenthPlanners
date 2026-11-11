# Collect training data: scans with the true value of every estimate.
#
# Two kinds of samples, per track:
# - driving: the car laps on the true estimates (LidarRacer's planner,
#   steering and speed), with slowly wandering errors added to e and ψe and a
#   random speed limit, so it also visits states off the ideal line. The
#   scans carry the LiDAR's real motion distortion.
# - static: the car placed at random across the lane and at random headings,
#   standing still, for states the driving never reaches.
#
#     julia -t auto --project=. collect.jl            # every track, into data/
#     julia --project=. collect.jl Spa Monza          # some tracks
#
# The data is deterministic: the same code gives the same files (each track
# has its own fixed seed), so it is regenerated rather than committed.

include("truth.jl")   # the true estimates; common.jl: the tracks, DATA_DIR
using LidarRacer
using Printf
using Random

# Ornstein-Uhlenbeck noise: wanders with standard deviation σ, changing over about τ seconds.
mutable struct Wander
    x::Float64
    σ::Float64
    τ::Float64
end
wander!(w::Wander, dt, rng) = (w.x += -w.x * dt / w.τ + w.σ * sqrt(2dt / w.τ) * randn(rng); w.x)

# What the LiDAR reports for traced distances (as F1Tenth.jl's sensor model):
# noise on valid distances, an error code for too close or out of range.
function sensor!(scan, noise_sd, far, rng)
    for i in eachindex(scan)
        d = scan[i]
        scan[i] = d >= far ? 65.533 : d < 0.021 ? 0.004 : d + noise_sd * randn(rng)
    end
    return scan
end

mutable struct Samples
    X::Vector{Vector{Float32}}
    Y::Dict{Symbol,Vector{Vector{Float32}}}
    v::Vector{Float32}
    r::Vector{Float32}
    static::Vector{Bool}
end
Samples() = Samples(Vector{Float32}[], Dict(est => Vector{Float32}[] for est in ESTIMATES), Float32[], Float32[], Bool[])

function add!(S::Samples, scan, labels, v, r, static)
    push!(S.X, Float32.(scan))
    for est in ESTIMATES
        push!(S.Y[est], Float32.(labels[est]))
    end
    push!(S.v, v); push!(S.r, r); push!(S.static, static)
    return S
end

# A pose on the centerline at a random place, facing along it.
function random_start(track, rng)
    s = rand(rng) * track.length
    x, y = center_point(track, s)
    return x, y, center_heading(track, s)
end

"""
    drive_truth(name; seconds, seed, v_max, wander, laps, samples) -> (time, crashes, laps)

Drive on track `name` steering by the true estimates (plus wandering errors
if `wander`), for `seconds` of simulated time or `laps` laps, adding every
scan to `samples` if given. After a crash, start again at a random place.
"""
function drive_truth(name; seconds=60.0, seed=1, v_max=nothing, a_lat=5.0, wander=true, laps=typemax(Int),
                     samples=nothing, start=nothing)
    rng = Xoshiro(seed)
    track = Track(name)
    env = F110Env(; map=name, num_agents=1, timestep=0.0025, integrator=Integrator.RK4, seed)
    car, agent, p = CarParams(env.params), env.sim.agents[1], env.params
    beams = agent.lidar.scan_angles
    sweep = (beams[end] - beams[1]) / 2π / car.scan_rate
    planner = Planner(car; sweep, v_max=something(v_max, car.speed_cmd_max), a_lat)
    x_ref, hw = car.lf - env.lidar_dist, car.width / 2
    e_err, ψ_err = Wander(0.0, 0.15, 1.5), Wander(0.0, 0.06, 1.0)

    pose = something(start, (center_point(track, 0.0)..., center_heading(track, 0.0)))
    obs, _, _, _ = reset!(env, [pose[1] pose[2] pose[3]])
    t, crashes, hint, last_scan = 0.0, 0, 0, -Inf
    steer, v_cmd, pending = 0.0, 0.0, Tuple{Float64,Float64,Float64}[]
    restart() = begin
        crashes += 1
        x, y, θ = random_start(track, rng)
        obs, _, _, _ = reset!(env, [x y θ])
        hint, last_scan, steer, v_cmd = 0, -Inf, 0.0, 0.0
        empty!(pending)
    end
    while t < seconds && obs.lap_counts[1] < laps
        v, r = obs.linear_vels_x[1], obs.ang_vels_z[1]
        if obs.scan_times[1] != last_scan
            dt = isfinite(last_scan) ? obs.scan_times[1] - last_scan : 1 / car.scan_rate
            last_scan = obs.scan_times[1]
            res = truth(track, agent.scan_origin, x_ref; hw, scan_range=car.scan_range, hint)
            if res === nothing
                restart()
                continue
            end
            labels, hint = res
            samples === nothing || add!(samples, obs.scans[1], labels, v, r, false)
            if wander
                labels = merge(labels, (e=labels.e .+ wander!(e_err, dt, rng), ψe=labels.ψe .+ wander!(ψ_err, dt, rng)))
            end
            new_steer, new_speed, _ = drive(planner, LaneState(labels), v, r)
            push!(pending, (last_scan + sweep + LidarRacer.default_latency(), new_steer, new_speed))
        end
        while !isempty(pending) && env.current_time >= pending[1][1] - 1e-9
            _, steer, v_cmd = popfirst!(pending)
        end
        obs, dt_step, _, _ = step!(env, [steer v_cmd])
        t += dt_step
        if obs.collisions[1] > 0
            samples === nothing && return t, crashes + 1, obs.lap_counts[1]
            restart()
        end
    end
    return t, crashes, obs.lap_counts[1]
end

"""
    add_static!(samples, name, n; seed)

`n` scans from random standing poses on track `name`: anywhere across the
lane (keeping the car's body off the walls), heading up to about 40° off
the lane's direction.
"""
function add_static!(samples::Samples, name, n; seed=1)
    rng = Xoshiro(seed)
    track = Track(name)
    env = F110Env(; map=name, num_agents=1, seed)
    car, agent, p = CarParams(env.params), env.sim.agents[1], env.params
    x_ref, hw = car.lf - env.lidar_dist, car.width / 2
    scan = zeros(length(agent.lidar.scan_angles))
    m, added = track.map, 0
    while added < n
        s = rand(rng) * track.length
        (cx, cy), h = center_point(track, s), center_heading(track, s)
        wl = free_run(m, cx, cy, -sin(h), cos(h), WIDTH_MAX)
        wr = free_run(m, cx, cy, sin(h), -cos(h), WIDTH_MAX)
        offset = -wr + 0.15 + rand(rng) * max(wl + wr - 0.3, 0.0)   # the front axle, left of the center
        θ = h - clamp(0.3randn(rng), -0.7, 0.7)
        fx, fy = cx - offset * sin(h), cy + offset * cos(h)
        F1Tenth.body_hits_wall(m, fx - car.lf * cos(θ), fy - car.lf * sin(θ), θ, p.length, p.width) && continue
        lidar = (fx - x_ref * cos(θ), fy - x_ref * sin(θ), θ)
        res = truth(track, lidar, x_ref; hw, scan_range=car.scan_range)
        res === nothing && continue
        get_scan!(scan, agent.scan_simulator, lidar)
        sensor!(scan, p.scan_noise, agent.scan_simulator.max_range, rng)
        add!(samples, scan, res[1], 0.0, 0.0, true)
        added += 1
    end
    return samples
end

function save_samples(path, S::Samples, name)
    X = reduce(hcat, S.X)
    Y = Dict(String(est) => reduce(hcat, S.Y[est]) for est in ESTIMATES)
    jldsave(path; X, Y, v=S.v, r=S.r, static=S.static, track=name)
end

# A track's seed: its place among all racetracks, so a track's data is the
# same however many tracks are collected at once.
track_seed(name) = findfirst(==(name), sort!(collect(keys(F1Tenth.RACETRACKS))))

# Per track: `runs` drives of `seconds` each, with random speed limits, plus `static` standing scans.
function collect_track(name; runs=2, seconds=60.0, static=2500, seed=track_seed(name), dir=DATA_DIR)
    S = Samples()
    crashes = 0
    for k in 1:runs
        rng = Xoshiro(1000seed + k)
        v_max, a_lat = 3.0 + 11.0rand(rng), 3.0 + 4.0rand(rng)
        _, c, _ = drive_truth(name; seconds, seed=1000seed + k, v_max, a_lat, samples=S,
            start=random_start(Track(name), rng))
        crashes += c
    end
    add_static!(S, name, static; seed=seed + 7)
    mkpath(dir)
    save_samples(joinpath(dir, name * ".jld2"), S, name)
    @printf("%-14s %6d samples (%d driving), %d crashes\n", name, length(S.X), count(!, S.static), crashes)
    return length(S.X)
end

if abspath(PROGRAM_FILE) == @__FILE__
    names = isempty(ARGS) ? sort!(collect(keys(F1Tenth.RACETRACKS))) : ARGS
    Threads.@threads :dynamic for i in eachindex(names)
        collect_track(names[i])
    end
end
