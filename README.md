# Sokoban Solver — per-room push-optimal A*

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/akbarzn1311/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/akbarzn1311/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A Sokoban solver for the make.fun bounty "Build the best Sokoban solver, 21 November 2026".
It answers a set of 5 instances in one run, writing one `solution.txt` per
instance (player moves in LURD).

## How it works

Every level in this competition is a hallway (one row, where the player
starts) with 1–4 rooms off it, each room holding 3–4 boxes and as many goals
and opening onto the hallway through exactly one door. A box pushed out
through a door can never be pushed back, so every box stays in its room and
a level's push-optimal count P\* is the sum of its rooms' optima. The solver
exploits that structure:

1. **Split** — the hallway row is the player's row; removing it splits the
   floor into the rooms. Each room has exactly one door cell.
2. **Solve each room** — a push-optimal A\* search over pushes, with player
   reachability by BFS, a Hungarian box-to-goal assignment heuristic over
   cell distances, and dead-square + frozen-box pruning.
3. **Stitch** — the per-room move strings are joined with hallway walks
   between doors, trying room orders until every walk (including the exit
   walk out of each room) is possible.

The search is deterministic by construction: fixed iteration orders, a
monotonic tie-break counter, and work-based expansion caps only. It never
reads the wall clock, so two runs produce byte-identical output.

## Quickstart

Requires Node 22 (the program runs as `node main.ts`, erasable TypeScript).

```sh
MAKEFUN_INPUT_DIR=<input dir> MAKEFUN_OUTPUT_DIR=<output dir> node main.ts
```

`<input dir>` holds one subdirectory per instance (`i001` … `i005`), each
with a `level.txt` in XSB format. The program writes
`<output dir>/<id>/solution.txt` for every instance, creating the
directories itself.

## Grade locally

From the unpacked competition kit (which also holds the practice set):

```sh
node cli/makefun.mjs grade-local --kit . --bot <path to this repository> --driver process
```

## Layout

- `main.ts` — the whole solver (parsing, room detection, A\*, stitching).
- `LICENSE` — MIT.
- `.makefun/owner.json` — the payout wallet of the entry.
