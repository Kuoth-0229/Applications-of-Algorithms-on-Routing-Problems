# Lab3 — Grid-based 2-pin net router (MaxSAT)

[spec](https://hackmd.io/@kpXY7wd4S22745J0JNeO_A/HJm0ijVkzg)

Routes all nets simultaneously by encoding the routing problem as a (weighted
partial) MaxSAT instance and solving it with **Open-WBO**.  Only `router.cpp` is
implemented; `main.cpp` is unmodified.

## Results (single-threaded, each under 10 minutes, confirmed by the verifier)

| Case  | Wirelength | Vias | Cost      | Time  |
|-------|-----------:|-----:|----------:|------:|
| case1 |         64 |    0 |        64 |  ~0s  |
| case2 |        180 |    0 |       180 |  ~0s  |
| case3 |        295 |   43 |       338 | ~120s |
| case4 |    740 000 |   32 | 1 060 000 | ~120s |
| case5 |  1 618 000 |   74 | 2 728 000 | ~250s |
| bonus |      2 000 |   68 |     3 360 | ~120s |

## Formulation

Per net `n`, the grid graph (nodes = gridline intersections; edges between
index-adjacent nodes, plus vias between layers) is encoded with Boolean
variables.

* **Monotone (primary).** Directed arcs that only move *toward* the net's target
  (forward planar arcs + both-direction vias) with single-commodity flow
  conservation (source emits 1, sink absorbs 1, every other node conserves
  ≤ 1).  Every source→sink path is therefore a staircase, so the **wirelength is
  exactly the Manhattan distance (optimal)** and the search space is small.
* **Undirected (fallback).** General edge model with degree constraints
  (pin = exactly one incident edge, others = zero or two).  Used automatically
  when monotone routing is infeasible (e.g. single-layer convergent nets in
  case1).
* **No overlap.** A node-occupancy variable per net+node with a pairwise
  at-most-one-net constraint forbids the verifier's node sharing (which covers
  short / crossing too).
* **Objective.** Unit soft clauses penalise each used arc/edge by its cost
  (wirelength for planar, `via_cost` for vias); minimising = the verifier's
  `wirelength + via_num * via_cost`.

All constraints are **global and order-independent** — each net depends only on
its own two pins plus a fixed margin rule; there is no sequential pre-routing and
no input-specific hardcoding.

### Search strategy

1. Grid starts at the **Hanan grid** (gridlines = pin coordinates); on UNSAT the
   grid is refined (extra gridlines spaced by `min_pitch_size`) and then the
   per-net bounding-box margin is widened.
2. **Two-phase per case:** phase 0 finds a legal routing fast (feasibility only)
   and stores it as a guaranteed fallback; phase 1 re-solves at the same grid
   with the cost objective to minimise vias, keeping the best model the solver
   returns within a time budget (`std::chrono`-based, ≈ 540 s, well under the
   10-minute limit).

## Building & running

```bash
make                              # builds ./Lab3
export LD_LIBRARY_PATH=$PWD/lib   # provides libgmp.so.3 for the old open-wbo
./Lab3 case/bonus.txt out.txt
./verifier out.txt case/bonus.txt
```

`lib/libgmp.so.3` is a symlink to the system `libgmp.so.10` (the bundled
`open-wbo` was linked against the old soname).  During grading the TA supplies
`open-wbo` and the correct environment.

## Reproduce everything

```bash
scripts/run_all.sh        # route every case and run the verifier
scripts/gen_artifacts.sh  # write result/csv/<case>.csv and result/png/<case>.png (uses venv)
```

## Layout

```
router.cpp                 # the solver-input generator / result reader (the work)
main.cpp, Makefile         # provided, unmodified
scripts/run_all.sh         # route + verify all cases
scripts/gen_artifacts.sh   # CSV + PNG per case (via venv)
scripts/plot_case.py       # headless 3-D plotter
result/<case>.out.txt      # routing outputs
result/csv/, result/png/   # Kaggle CSVs and visualisations
venv/                      # python venv (pandas, matplotlib, numpy)
```
