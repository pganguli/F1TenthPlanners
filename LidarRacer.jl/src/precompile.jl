# Compile a scan's whole path (perception, planner, filter, steering, speed)
# when the package is installed, so the first scan of a session doesn't wait
# about 2.5 s for the compiler.
@setup_workload begin
    # NUC5 and its Hokuyo UST-10LX, as in F1Tenth.jl's defaults
    car = CarParams(lf=0.130, lr=0.127, width=0.228, s_max=0.4189, steer_cmd_min=-0.327, steer_cmd_max=0.292,
        speed_cmd_max=14.375, decel_cmd=2.5, scan_rate=40.0, scan_range=10.0)
    beams = collect(range(-3π / 4, 3π / 4; length=1081))
    # a straight lane 2 m wide, seen from its center (65.533: no echo)
    lane = [abs(sin(a)) < 1e-3 || 1 / abs(sin(a)) > 10 ? 65.533 : 1 / abs(sin(a)) for a in beams]
    one_wall = [a > 0 ? 65.533 : r for (a, r) in zip(beams, lane)]   # the left wall missing
    nothing_seen = fill(65.533, length(beams))
    @compile_workload begin
        for angles in (beams, range(-3π / 4, 3π / 4; length=1081))
            racer = Racer(car, angles, 0.143)
            for ranges in (lane, lane, one_wall, lane, nothing_seen)
                control!(racer, ranges, 5.0, 0.1, 0.025)
            end
        end
    end
end
