# Lab2 — ISPD 2008 global routing

3D global router ([spec](https://hackmd.io/@kpXY7wd4S22745J0JNeO_A/BJZlOGmsWx)): it reads an
ISPD 2008 benchmark, decomposes nets into two-pin nets, routes them in 2D with pattern and maze
routing plus rip-up and reroute, and finally assigns layers (`LayerAssignment.*` is the
NCTU-GR code provided with the assignment).

## Build and run

```bash
make
./router <benchmark.gr> <output.sol>
./router 3d.txt out.txt                     # small example from the spec

perl eval2008/eval2008.pl <benchmark.gr> <output.sol>
python3 verifier.py <benchmark.gr> <output.sol>
```

Benchmarks are not in the repository; see the top-level README. `testall.sh` and `verifyall.sh`
run the whole benchmark set.
