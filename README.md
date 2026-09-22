# The Applications of Algorithms on Routing Problems (演算法在繞線問題的實務應用)

- School: NYCU
- 開課單位: 資工系
- Instructor: 李毅郎
- Semester: 114-2 (2026 Spring)

| Directory | Lab | Topic | Spec |
|---|---|---|---|
| [`lab1-channel-routing/`](lab1-channel-routing) | Lab1 | Two-layer detailed router using greedy channel routing | [spec](https://hackmd.io/@kpXY7wd4S22745J0JNeO_A/H11UV41F-g) |
| [`lab2-global-routing/`](lab2-global-routing) | Lab2 | DP based global router for the ISPD 2008 Global Routing Contest | [spec](https://hackmd.io/@kpXY7wd4S22745J0JNeO_A/BJZlOGmsWx) |
| [`lab3-maxsat-routing/`](lab3-maxsat-routing) | Lab3 | Grid-based 2-pin net router encoded as MaxSAT and solved with Open-WBO | [spec](https://hackmd.io/@kpXY7wd4S22745J0JNeO_A/HJm0ijVkzg) |

`drafts/` holds other versions written along the way: an earlier Lab2 main, a separate 2D
router, and a different take on Lab1. They are kept out of the lab directories so each lab
builds on its own.

## Build notes

Lab1 builds and both testcases pass its internal legality checks. Lab2 builds, and the `3d.txt`
example gives TOF 0, MOF 0, WL 14 under both `eval2008.pl` and `verifier.py`. Lab3 builds;
running it needs an Open-WBO binary, which is not in the repository.

## Not included

The ISPD 2008 benchmarks (`adaptec*`, `bigblue*`, `newblue*.gr`, about 600 MB) and the routed
`.sol` / `.out` files — download the benchmarks from the
[ISPD 2008 contest page](https://www.ispd.cc/contests/08/ispd08rc.html). Also excluded: the
Open-WBO binary and `libgmp.so.3`, Python virtualenvs, build artifacts and reference papers.
