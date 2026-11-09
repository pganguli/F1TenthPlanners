# F1TenthPlanners

[![CI](https://github.com/pganguli/F1TenthPlanners/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/pganguli/F1TenthPlanners/actions/workflows/CI.yml?query=branch%3Amain)

Julia packages for racing an F1TENTH car with only its LiDAR. Each folder is
its own package.

| Package | What it does |
|---|---|
| [LidarLaneNets.jl](LidarLaneNets.jl) | estimates the lane from a LiDAR scan with neural networks: a small one per estimate on the car, a large one in the cloud |
| [LidarLaneEstimation.jl](LidarLaneEstimation.jl) | the hand-written estimator: finds the lane between two walls in a LiDAR scan, and where the car is in it (unused on this branch; kept for comparison) |
| [MinimumCurvaturePlanner.jl](MinimumCurvaturePlanner.jl) | plans the line through a lane that bends least |
| [VelocityProfilePlanner.jl](VelocityProfilePlanner.jl) | sets the speed along a path by the forward-backward method |
| [StanleyController.jl](StanleyController.jl) | steers along a path with the Stanley method |
| [LidarRacer.jl](LidarRacer.jl) | combines them to race an [F1Tenth.jl](https://github.com/pganguli/F1Tenth.jl) car (on this branch, with LidarLaneNets.jl) |

The trained weights (`LidarLaneNets.jl/weights/`) are stored with Git LFS:
run `git lfs install` once before cloning, or `git lfs pull` after.

To run a package's tests, from its folder:

```sh
julia --project=. -e 'import Pkg; Pkg.test()'
```
