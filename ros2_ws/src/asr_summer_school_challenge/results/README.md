# Measured results

Every run below is a full mission in the simulated 20 x 20 m maze
(`worlds/hard_maze_apriltag.world`), scored by `score_report.py` against the tag
coordinates parsed out of the world file itself. Reproduce any of them with:

```bash
./sim_run.sh <tag> <seconds> [launch args...]
```

## Full-length runs (600 s window)

| Run | Camera sweep | Tags | Goals | Elapsed | Mean tag error | Home | **Score** |
|---|---|---|---|---|---|---|---|
| `best_run_600s` | full turn (default) | **5** | 19/19 | 516 s | 0.076 m | 0.046 m | **730** |
| sweep on | full turn | 4 | 19/21 | 573 s | 0.073 m | 0.029 m | 680 |
| sweep halved | π | 4 | 24/24 | 531 s | 0.377 m | 0.207 m | 650 |
| sweep off | none | 4 | 17/22 | 502 s | 0.677 m | 0.208 m | 650 |

The sweep rows are an A/B, not four attempts at the same thing — see
[the reasoning in PROJECT_GUIDE](../../../PROJECT_GUIDE.md#7-before-the-real-run).
Cutting the sweep saves 40–70 s and costs the localisation, not the tag count.

## Shorter and early-exit runs

| Run | Window | Tags | Goals | Elapsed | Home | Why it stopped |
|---|---|---|---|---|---|---|
| 300 s mission | 300 s | 3 | 9/9 | 244 s | 0.257 m | return budget spent |
| `stop_after_tags:=2` | 600 s | 2 | 4/5 | **107 s** | 0.042 m | found all 2 tags |

The second row is the `stop_after_tags` switch working: the run ended 493 s
inside its window because it already had everything it was told to look for.

## What is in `best_run_600s/`

Exactly what a run writes to `output_directory` — this is the reference for what
good output looks like, and the format to hand the organisers.

| File | What it is |
|---|---|
| `map.pgm`, `map.yaml` | The 2D occupancy grid, in the format `nav2_map_server` writes. **+100** |
| `semantic_map.yaml` | Tag IDs and map-frame positions, shaped like the course's own `landmarks.yaml`. **+100** |
| `semantic_map.json` | The same tags plus per-tag provenance and full mission metadata |
| `semantic_map.csv` | One row per tag |
| `mission_report.json` | Timings, goal counts, map statistics, any detected SLAM jumps |
| `mission_overlay.png` | The grid with the tags, the start and the finish drawn on it |

Its five tags landed at a mean error of 7.6 cm, worst 10.7 cm — inside the 15 cm
bracket that pays the full accuracy award — and the robot finished 4.6 cm from
its start, well inside the 50 cm circle.

**These numbers are from simulation.** The physical arena is smaller and
differently shaped; treat them as evidence the pipeline works, not as a
prediction of Friday's score.
