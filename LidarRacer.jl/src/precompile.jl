# Compile a scan's whole path (networks, selector, filter, planner, steering,
# speed) when the package is installed, so the first scan of a session
# doesn't wait for the compiler. The networks here are untrained.
@setup_workload begin
    # NUC5 and its Hokuyo UST-10LX, as in F1Tenth.jl's defaults
    car = CarParams(lf=0.130, lr=0.127, width=0.228, s_max=0.4189, steer_cmd_min=-0.327, steer_cmd_max=0.292,
        speed_cmd_max=14.375, decel_cmd=2.5, scan_rate=40.0, scan_range=10.0)
    beams = collect(range(-3π / 4, 3π / 4; length=1081))
    # a straight lane 2 m wide, seen from its center (65.533: no echo)
    lane = [abs(sin(a)) < 1e-3 || 1 / abs(sin(a)) > 10 ? 65.533 : 1 / abs(sin(a)) for a in beams]
    @compile_workload begin
        tier(t) = NamedTuple{ESTIMATES}(Tuple(EstimatorNet(est, t, length(beams)) for est in ESTIMATES))
        nets = (small=tier(:small), large=tier(:large))
        racer = Racer(car, beams, 0.143, nets)
        for _ in 1:3
            control!(racer, lane, 5.0, 0.1, 0.025)
        end
        truth = lane_labels(e=0.1, ψe=0.05, κ_profile=fill(0.2, length(PROFILE_S)), w_left=fill(1.0, length(PROFILE_S)),
            w_right=fill(1.0, length(PROFILE_S)), sight=8.0, clearance=fill(5.0, length(CLEARANCE_K)))
        drive(racer.planner, LaneState(truth), 5.0, 0.1)
    end
end
