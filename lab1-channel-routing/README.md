# Lab1 — Channel router

Greedy channel router, C++17, single translation unit ([spec](https://hackmd.io/@kpXY7wd4S22745J0JNeO_A/H11UV41F-g)).

The router parses the top and bottom pin rows and the coupling constraints, then sweeps columns
left to right: existing tracks are carried forward, pins are connected onto tracks by a greedy
cost, jogs and split nets are allowed, and spillover columns on the right collapse whatever is
still split. Before printing it checks legality (no shorts, no illegal boundary use, every pin
connected) and reports wirelength, spillover, via and coupling costs.

## Build and run

```bash
make
./Lab1 testcase/testcase1.txt out.txt      # with no arguments it reads stdin and writes stdout
python3 plotter.py --in_file testcase/testcase1.txt --out_file out.txt --img_name out.png
```

`sample_output/` contains the result for testcase1 (`testcase1.out.txt`) and its plot.
