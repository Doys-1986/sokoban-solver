# sokoban-solver

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Doys-1986/sokoban-solver/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Doys-1986/sokoban-solver/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-level A* Sokoban solver that exploits the generated levels' structure: each level is a one-wide hallway plus separate rooms, and a box pushed out of its room can never return, so the solver decomposes the level by removing the hallway row, solves each room independently with push-level A* — states are sorted box cells plus the player's canonical reachable cell, the heuristic is an admissible minimum-cost bipartite matching of boxes to goals over true BFS grid distances (Hungarian algorithm), with reverse-pull dead-square pruning and 2x2 block deadlock detection — and joins the room solutions with plain walks through the hallway, falling back to a whole-level push A* if decomposition fails. The search is budgeted purely by deterministic expansion counts, never the clock, so repeated runs are byte-identical. On the kit's practice set it solves all five instances at the push-optimal counts (13, 16, 29, 47, 60 pushes).

## Run it

```sh
node main.ts
```

Reads instances from `/input` (or `$MAKEFUN_INPUT_DIR`) and writes `/output/<id>/solution.txt` (or `$MAKEFUN_OUTPUT_DIR`).

---

<sub>Entered on [make.fun](https://make.fun/?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
