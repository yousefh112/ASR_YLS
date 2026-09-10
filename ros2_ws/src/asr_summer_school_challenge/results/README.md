# Measured results

> **These are all 600 s runs of the 400 m² practice maze, recorded before the
> organisers announced the brief.** The real run is **240 s, 12 tags, ~40 m²**
> — a tenth of the area in 40% of the window — so none of the scores below
> predicts the day. What they are good for is the *relative* evidence: which
> changes moved throughput, and how noisy a single run is.
>
> The simulation still runs this maze deliberately; it exercises the code, it
> does not model the arena. See RUNBOOK section 0.

Every run below is a full mission in the simulated 20 x 20 m maze
(`worlds/hard_maze_apriltag.world`), scored by `score_report.py` against the tag
coordinates parsed out of the world file itself. Reproduce any of them with:

```bash
sim_run.sh <tag> <seconds> [launch args...]
```

## Full-length runs (600 s window)

### Before the speed work — the old configuration

| Run | Camera sweep | Tags | Goals | Elapsed | Mean tag error | Home | **Score** |
|---|---|---|---|---|---|---|---|
| `best_run_600s` | full turn (default) | **5** | 19/19 | 516 s | 0.076 m | 0.046 m | **730** |
| sweep on | full turn | 4 | 19/21 | 573 s | 0.073 m | 0.029 m | 680 |
| sweep halved | π | 4 | 24/24 | 531 s | 0.377 m | 0.207 m | 650 |
| sweep off | none | 4 | 17/22 | 502 s | 0.677 m | 0.208 m | 650 |

### After — sweeps at 1.9 rad/s, retuned return budget, patrol

| Run | Tags | Goals | Elapsed | Area | Mean tag error | Home | **Score** |
|---|---|---|---|---|---|---|---|
| **`final2`** — patrol active | **6** | 24/24 | 560 s | **158.0 m²** | 0.173 m | 0.045 m | **765** |
| `final600` — patrol declined | 5 | 23/23 | 476 s | 126.2 m² | 0.098 m | 0.218 m | 730 |
| `faster600` | 4 | 21/23 | 577 s | 137.4 m² | 0.098 m | 0.045 m | 680 |
| `faster600b` | 4 | 24/25 | 561 s | — | 0.323 m | 0.243 m | 650 |

`final2` is the best run recorded against this arena: **six tags, where nothing
before it found more than five**, 158 m² mapped against a previous best of 137,
and home to 4.5 cm. It is also the first run in which patrolling actually fired
— two patrol goals, after the frontier search ran dry at t+357 with 101 s of
slack still on the clock. Under the old behaviour that run ends there and goes
home; `final600`, one commit earlier, did exactly that at 476 s with 124 s
unused and five tags.

The accuracy award fell to +15 in that run: one tag landed 0.40 m out and pulled
the mean into the 15–30 cm band. That is the right trade and it is worth being
explicit about, because the temptation is to tighten `max_detection_range` to
fix it. Six tags at +50 with +15 accuracy is 315 points; five tags with the full
+30 is 280. **A tag detected imprecisely scores 50; a tag not detected scores
nothing**, and the whole accuracy category is capped at 30.

Three of the four runs still fall inside the old configuration's own spread
(730/680/650/650), so treat 765 as the top of a distribution rather than a level
that reproduces on demand.

What *is* separable are the metrics that do not depend on a lucky loop closure:

| | Old (4 runs) | New (3 runs) |
|---|---|---|
| Goals dispatched | 17–24, mean 19.8 | 23–25, mean 23.7 |
| Seconds of the window used | 502–573 | 476–577 |
| Time spent turning | ~48% of one run | 27% |
| Seconds per goal | 27.2 | 16.1 |

Goal throughput is up about 40% per goal, and that is a real, repeatable
change: a 2π sweep at 1.0 rad/s took 7.4 s and a 5.30 rad sweep at 1.9 rad/s
takes 3.0 s, measured from consecutive log timestamps rather than inferred.

Throughput alone did not convert into tags — `faster600` and `faster600b` were
faster than the baseline and found four. The limiter is not how fast the robot
covers ground; it is that the camera — 59° in Gazebo, 69° on the robot — has to
be *pointing* at a tag, and
`final600` mapped less area than `faster600` while finding more tags.

What converted it was patrol: spending the leftover clock looking from places
the camera had not looked. `final2` is the run where speed and patrol compound —
the extra throughput buys the slack, and patrol spends it on viewing angles
instead of on parking at the start.

Still unvalidated here: the `decimate` change, which **cannot** be measured in
this simulation at all. Gazebo's camera is 1920×1080 with no motion blur and no
rolling shutter; the robot's RealSense runs at 1280×720 and has both. `apriltag_sim.yaml` exists to keep the
simulated detector's effective resolution comparable to the robot's, so these
numbers are not flattered by a detector nobody will run. Detection range and
sweep blur are hardware checks — see the pre-flight section of the runbook.

**Read that table as a distribution, not as four measurements of four things.**
It was originally presented as an A/B showing the camera sweep buys localisation.
It does not, and the mechanism given for it — a full turn feeding slam_toolbox
pose-graph nodes — does not exist: `shouldProcessScan` gates on translation
only, so a stationary turn is discarded before the mapper sees it (CLAUDE.md §2).

What the four rows actually show is **how noisy one run is**. Same arena, same
stack, tag counts 5/4/4/4 and scores 730/680/650/650. The accuracy award is
bimodal — it is 30 or it is 0, depending on whether a particular loop closure
lands — and it takes the return distance with it (0.03–0.05 m against
0.21 m). Any single run is therefore worth ±40 points of noise before any change
is applied, and **no conclusion about tag count or accuracy can be drawn from
n=1.** The original sweep conclusion was exactly that mistake.

Low-variance metrics — goals dispatched, area mapped, seconds of the window
actually used — are the ones to compare between configurations.

## Shorter and early-exit runs

| Run | Window | Tags | Goals | Elapsed | Home | Why it stopped |
|---|---|---|---|---|---|---|
| 300 s mission | 300 s | 3 | 9/9 | 244 s | 0.257 m | return budget spent |
| `stop_after_tags:=2` | 600 s | 2 | 4/5 | **107 s** | 0.042 m | found all 2 tags |

The second row is the `stop_after_tags` switch working: the run ended 493 s
inside its window because it already had everything it was told to look for.

## What is in `best_run_765/`

The deliverables from `final2`, the 765-point run — six tags, 158 m² mapped,
home to 4.5 cm. This is the reference for what good output looks like and the
format to hand the organisers. `best_run_600s/` is kept alongside it as the
pre-speed-work reference.

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
| `coverage_report.txt` | What the run looked at and what it did not. Newer runs only |

Its five tags landed at a mean error of 7.6 cm, worst 10.7 cm — inside the 15 cm
bracket that pays the full accuracy award — and the robot finished 4.6 cm from
its start, well inside the 50 cm circle.

**These numbers are from simulation.** The physical arena is smaller and
differently shaped; treat them as evidence the pipeline works, not as a
prediction of Friday's score.
