# MinimumCurvaturePlanner.jl

Minimum-curvature path planning, as in Heilmeier, Wischnewski, Hermansdorfer,
Betz, Lienkamp and Lohmann, "Minimum curvature trajectory planning and control
for an autonomous race car" (Vehicle System Dynamics 58(10), 2019). Given a
reference line and how far each of its points may move sideways, it finds the
path that bends least: in wide before a corner, close to the inside at its
apex, wide again after. Such a path lets a vehicle corner faster, and turns
less sharply than the reference where the reference is tighter than the
vehicle can steer.

Nothing is tied to a vehicle, sensor or track format. It plans over a stretch
of road (an open line: a lane seen by a sensor, say) or a whole lap (a closed
line: a track's centerline and widths).

## Use

```julia
using MinimumCurvaturePlanner

# reference points (about evenly spaced) and how far each may move along its
# left normal: lower[i] ≤ α[i] ≤ upper[i], with lower[i] ≤ 0 ≤ upper[i]
px, py = min_curvature_path(x, y, lower, upper)

# a whole lap, refined on the exact curvature
px, py = min_curvature_path(x, y, -w_right, w_left; closed=true, iterations=20)

# the shifts themselves, and the normals they are along
α = min_curvature_offsets(x, y, lower, upper)
nx, ny = left_normals(x, y)
```

For a track given as a centerline with widths to each side, the bounds are
`lower = -(w_right .- margin)` and `upper = w_left .- margin`, where `margin`
is half the vehicle's width plus whatever room it needs. Pass your own
`normals = (nx, ny)` if the bounds were measured along other directions.

## How it works

The path is the reference shifted along its left normals, `p_i = r_i + α_i n_i`.
It minimizes the summed squared curvature, linearized as Heilmeier et al. do:
with the tangents held at the reference's,
`κ_i ≈ n_i ⋅ (p_{i-1} - 2p_i + p_{i+1}) / Δs²`, which is linear in `α`. That
makes a convex quadratic program with box constraints, solved exactly by a
primal active-set method (Nocedal and Wright, *Numerical Optimization*,
Algorithm 16.3). A small `tiebreak` weight on `Σ α²` picks the path nearest
the reference among those that bend equally little (on a straight, every
parallel path does).

Each `κ_i` involves only a point and its two neighbors, so the program's
matrix is zero beyond two places from its diagonal (pentadiagonal), and
holding some points at their bounds keeps the rest pentadiagonal. Each step
of the active-set method is then a banded Cholesky solve, in time linear in
the number of points. A closed line's ends are neighbors too, which adds
entries in the matrix's corners; those couple only the first two free points
to the last two, so the last two are eliminated through their 2×2 Schur
complement, and the rest is again a banded solve (three right-hand sides
instead of one). That keeps the package free of dependencies beyond
LinearAlgebra, where a sparse Cholesky (CHOLMOD, from SparseArrays) would add
one and a fixed cost per solve. The method adds one bound per step, so the
number of steps grows with the number of bounds the path ends up at: a lap
of 5000 points pressed against 400 bounds takes about 0.2 s, one of 165
points about 0.2 ms, and a 50-point lane about 11 µs.

Holding the tangents fixed favors the inside of a corner: shifting a corner's
points outward spreads them apart, which the linearization counts as more
bending. On a narrow corridor that only keeps the path a little closer to the
reference. On a wide one, around long corners, it can make the corners
*tighter* than the reference: on a stadium with ±0.8 m of room around 2 m
corners, the quadratic program alone raises the peak curvature from 0.50 to
0.83 1/m. `iterations > 0` refines the path, as the iterative variant of the
method does (`mincurv_iqp` in TUM's global_racetrajectory_optimization): the
exact curvature, that of the circle through each point and its neighbors, is
linearized around the current path and the quadratic program solved again
(Gauss-Newton), each step shortened until the summed squared curvature falls.
On the stadium, 5 to 20 iterations bring the peak to 0.40 1/m and halve the
summed squared curvature.

Which to use depends on the job. For a vehicle that tracks the path with lag,
the quadratic program alone can be the better choice: its path stays closer
to the reference, away from the corridor's edges. For a lap time, refine.

The ends of an open line are free. Expect the path to straighten toward them:
past the last point, nothing is known to turn for.

## Tests

```sh
julia --project=. -e 'import Pkg; Pkg.test()'
```
