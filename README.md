# F1TenthPlanners

Julia packages for racing an F1TENTH car with only its LiDAR. Each folder is
its own package.

| Package | What it does |
|---|---|
| [LidarLaneEstimation.jl](LidarLaneEstimation.jl) | finds the lane between two walls in a LiDAR scan, and where the car is in it |
| [MinimumCurvaturePlanner.jl](MinimumCurvaturePlanner.jl) | plans the line through a lane that bends least |
| [VelocityProfilePlanner.jl](VelocityProfilePlanner.jl) | sets the speed along a path by the forward-backward method |
| [StanleyController.jl](StanleyController.jl) | steers along a path with the Stanley method |
| [LidarRacer.jl](LidarRacer.jl) | combines them to race an [F1Tenth.jl](https://github.com/pganguli/F1Tenth.jl) car |

To run a package's tests, from its folder:

```sh
julia --project=. -e 'import Pkg; Pkg.test()'
```
