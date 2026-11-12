# The real networks on the representative track

The oracle stands in for the networks; this checks the track and the oracle against the real
networks, driving the representative track ([track.md](track.md)) through LidarRacer.jl's race loop,
10 runs each (LiDAR noise seeds), 2 laps from a standing start. From
[`validate.jl`](../validate.jl).

| Networks | Crashes | Where [m from the start, about] | Least gap to a wall [m] | Mean speed [m/s] |
|---|---|---|---|---|
| car only (small_weak) | 10 / 10 | 107, 107, 107, 107, 107, 107, 107, 107, 107, 107 | 0.05 | 4.90 |
| cloud only (large, all six, 25 ms) | 0 / 10 | – | 0.18 | 5.27 |

Against [sensitivity.md](sensitivity.md): there, the oracle's `κ_profile` alone crashes in most runs,
in the chicane (about 97 m), with either network's errors. The real car networks crash too, at the
tight hairpin just after it (all six estimates erring at once). The real cloud networks never crash,
so the oracle's `κ_profile` errors are pessimistic there, at least for the cloud's.
