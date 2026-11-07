# Compile the common paths when the package is installed, so the first scan
# of a session doesn't wait for the compiler.
@setup_workload begin
    # a straight lane 2 m wide, seen from its center
    angles = range(-3π / 4, 3π / 4; length=1081)
    corridor = [abs(sin(a)) < 1e-3 ? 30.0 : min(30.0, 1.0 / abs(sin(a))) for a in angles]
    # the same with the left wall missing
    one_wall = [a > 0 ? 30.0 : r for (a, r) in zip(angles, corridor)]
    @compile_workload begin
        for line in (:center, :min_curvature)
            p = LanePerception(1081, 3π / 2, 0.13; cfg=PerceptionConfig(; line))
            f = LaneFilter(0.257)
            for ranges in (corridor, corridor, one_wall, corridor)
                track!(f, estimate_lane!(p, ranges), 5.0, 0.1, 0.01)
            end
        end
    end
end
