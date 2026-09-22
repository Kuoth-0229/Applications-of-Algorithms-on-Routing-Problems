#!/usr/bin/env python3
"""Render a routing output file to a PNG (non-interactive).

Usage: python plot_case.py <routing_output.txt> <out.png>

This is a headless, parametrised variant of the provided plotter.py: it never
calls plt.show() and writes to an explicit path so artifacts can be generated
per case in batch.
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def main():
    if len(sys.argv) != 3:
        print("usage: plot_case.py <routing_output.txt> <out.png>", file=sys.stderr)
        sys.exit(2)
    in_path, out_path = sys.argv[1], sys.argv[2]

    with open(in_path) as f:
        lines = f.readlines()

    # parse x_coors / y_coors header (4 lines) so node indices map to real coords
    x_coors = [int(v) for v in lines[1].split()]
    y_coors = [int(v) for v in lines[3].split()]
    body = lines[4:]

    paths, current = {}, None
    for line in body:
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) == 2:  # "<netname> <count>"
            current = []
            paths[parts[0]] = current
        else:
            i, j, k = map(int, parts)
            current.append((x_coors[i], y_coors[j], k))

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.computed_zorder = False
    elev, azim = 30, -60
    ax.view_init(elev=elev, azim=azim)
    cam = np.array([
        np.cos(np.radians(elev)) * np.cos(np.radians(azim)),
        np.cos(np.radians(elev)) * np.sin(np.radians(azim)),
        np.sin(np.radians(elev)),
    ])

    elements = []
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    ci = 0
    for name, path in paths.items():
        if len(path) < 2:
            # still draw a marker for degenerate (single-node) nets
            if path:
                x, y, z = path[0]
                elements.append({"type": "point", "x": x, "y": y, "z": z,
                                 "color": colors[ci % len(colors)], "label": name,
                                 "center": np.array([x, y, z])})
                ci += 1
            continue
        xs, ys, zs = zip(*path)
        c = colors[ci % len(colors)]
        ci += 1
        for i in range(len(xs) - 1):
            xseg, yseg, zseg = [xs[i], xs[i+1]], [ys[i], ys[i+1]], [zs[i], zs[i+1]]
            elements.append({"type": "line", "x": xseg, "y": yseg, "z": zseg,
                             "color": c, "label": name if i == 0 else "",
                             "center": np.array([np.mean(xseg), np.mean(yseg), np.mean(zseg)])})
        for idx in (0, -1):
            elements.append({"type": "point", "x": xs[idx], "y": ys[idx], "z": zs[idx],
                             "color": c, "label": "",
                             "center": np.array([xs[idx], ys[idx], zs[idx]])})

    for el in elements:
        el["depth"] = float(np.dot(el["center"], cam))
    elements.sort(key=lambda e: e["depth"])
    for z, el in enumerate(elements):
        if el["type"] == "line":
            ax.plot(el["x"], el["y"], el["z"], color=el["color"], label=el["label"], zorder=z)
        else:
            ax.scatter([el["x"]], [el["y"]], [el["z"]], color=el["color"], marker="o",
                       depthshade=False, zorder=z)

    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("layer")
    ax.set_zlim(0, None)
    handles, labels = ax.get_legend_handles_labels()
    pairs = [(h, l) for h, l in zip(handles, labels) if l]
    if pairs:
        h, l = zip(*pairs)
        ax.legend(h, l, fontsize=6, loc="upper left", bbox_to_anchor=(1.05, 1))
    fig.tight_layout()
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    print(f"wrote {out_path}  ({len(paths)} nets)")


if __name__ == "__main__":
    main()
