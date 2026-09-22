#!/usr/bin/env python3
"""Pure Python verifier for ISPD 2008 global routing format.

This file re-implements the core behavior used by eval2008.pl:
1) parse benchmark (.gr) and route output
2) check route legality/connectivity
3) compute TOF / MOF / WL

No Perl dependency is required, so it can run in Kaggle environments.
"""

from __future__ import annotations

import argparse
import bz2
import gzip
import json
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Set, Tuple
import csv

VIA_COST = 1


class ParticipantVisibleError(Exception):
    """User-facing verification error."""


Node = Tuple[int, int, int]  # (grid_x, grid_y, layer0)
Seg = Tuple[int, int, int, int, int, int]  # x1,y1,l1,x2,y2,l2 (grid coords)
HKey = Tuple[int, int, int]  # horizontal edge key: left-x, y, layer0
VKey = Tuple[int, int, int]  # vertical edge key: x, bottom-y, layer0


@dataclass
class NetDef:
    name: str
    net_id: int
    num_pins: int
    min_width: int
    pins_xy_layer: List[Tuple[int, int, int]]
    pins_grid_layer: List[Node]


@dataclass
class Benchmark:
    path_label: str
    gridx: int
    gridy: int
    layers: int
    vcap: List[int]
    hcap: List[int]
    minw: List[int]
    mins: List[int]
    llx: int
    lly: int
    xsize: int
    ysize: int
    nets: List[NetDef]
    net_index_by_name: Dict[str, int]
    hcap_override: Dict[HKey, int]
    vcap_override: Dict[VKey, int]

    def xytogrid(self, x: int, y: int) -> Tuple[int, int]:
        return (x - self.llx) // self.xsize, (y - self.lly) // self.ysize

    def cap_h(self, key: HKey) -> int:
        return self.hcap_override.get(key, self.hcap[key[2]])

    def cap_v(self, key: VKey) -> int:
        return self.vcap_override.get(key, self.vcap[key[2]])


@dataclass
class RouteEvalResult:
    tof: int
    mof: int
    wl: int
    overflowed_nets: int
    overflowed_edges: int
    warnings: List[str]


def _open_text(path: Path) -> str:
    if not path.is_file():
        raise ParticipantVisibleError(f"Missing file: {path}")
    if path.suffix == ".gz":
        with gzip.open(path, "rt", encoding="utf-8", errors="replace") as f:
            return f.read()
    if path.suffix == ".bz2":
        with bz2.open(path, "rt", encoding="utf-8", errors="replace") as f:
            return f.read()
    return path.read_text(encoding="utf-8", errors="replace")


def _non_empty_lines(text: str) -> List[str]:
    return [ln.strip() for ln in text.splitlines() if ln.strip()]


def _parse_prefixed_int_list(line: str, prefix: str, expected_len: int) -> List[int]:
    pattern = re.compile(rf"^\s*{re.escape(prefix)}\s+(.*)\s*$")
    m = pattern.match(line)
    if not m:
        raise ParticipantVisibleError(f"Bad line, expect '{prefix} ...': {line}")
    vals = [int(t) for t in m.group(1).split()]
    if len(vals) != expected_len:
        raise ParticipantVisibleError(
            f"Bad '{prefix}' count: expected {expected_len}, got {len(vals)}"
        )
    return vals


def parse_benchmark_text(text: str, path_label: str = "<design>") -> Benchmark:
    lines = _non_empty_lines(text)
    idx = 0

    m = re.match(r"^grid\s+(\d+)\s+(\d+)\s+(\d+)\s*$", lines[idx])
    if not m:
        raise ParticipantVisibleError("Bad benchmark: missing 'grid x y layers'")
    gridx, gridy, layers = int(m.group(1)), int(m.group(2)), int(m.group(3))
    idx += 1

    vcap = _parse_prefixed_int_list(lines[idx], "vertical capacity", layers)
    idx += 1
    hcap = _parse_prefixed_int_list(lines[idx], "horizontal capacity", layers)
    idx += 1
    minw = _parse_prefixed_int_list(lines[idx], "minimum width", layers)
    idx += 1
    mins = _parse_prefixed_int_list(lines[idx], "minimum spacing", layers)
    idx += 1
    _ = _parse_prefixed_int_list(lines[idx], "via spacing", layers)
    idx += 1

    m = re.match(r"^(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*$", lines[idx])
    if not m:
        raise ParticipantVisibleError("Bad benchmark: missing 'llx lly tile_w tile_h'")
    llx, lly, xsize, ysize = map(int, m.groups())
    idx += 1

    m = re.match(r"^num\s+net\s+(\d+)\s*$", lines[idx])
    if not m:
        raise ParticipantVisibleError("Bad benchmark: missing 'num net N'")
    numnet = int(m.group(1))
    idx += 1

    nets: List[NetDef] = []
    net_index_by_name: Dict[str, int] = {}
    for _ in range(numnet):
        m = re.match(r"^(\S+)\s+(\d+)\s+(\d+)\s+(\d+)\s*$", lines[idx])
        if not m:
            raise ParticipantVisibleError(f"Bad net header line: {lines[idx]}")
        name = m.group(1)
        net_id = int(m.group(2))
        npin = int(m.group(3))
        nminw = int(m.group(4))
        idx += 1

        pins_xy_layer: List[Tuple[int, int, int]] = []
        pins_grid_layer: List[Node] = []
        for _ in range(npin):
            m = re.match(r"^(\d+)\s+(\d+)\s+(\d+)\s*$", lines[idx])
            if not m:
                raise ParticipantVisibleError(f"Bad pin line: {lines[idx]}")
            x, y, layer1 = map(int, m.groups())
            if layer1 <= 0:
                raise ParticipantVisibleError(
                    f"Layer index must be positive in net {name}"
                )
            l0 = layer1 - 1
            xg = (x - llx) // xsize
            yg = (y - lly) // ysize
            pins_xy_layer.append((x, y, l0))
            pins_grid_layer.append((xg, yg, l0))
            idx += 1

        net_index_by_name[name] = len(nets)
        nets.append(
            NetDef(
                name=name,
                net_id=net_id,
                num_pins=npin,
                min_width=nminw,
                pins_xy_layer=pins_xy_layer,
                pins_grid_layer=pins_grid_layer,
            )
        )

    hcap_override: Dict[HKey, int] = {}
    vcap_override: Dict[VKey, int] = {}
    while idx < len(lines):
        try:
            numblock = int(lines[idx])
        except ValueError as exc:
            raise ParticipantVisibleError(
                f"Bad block section header: {lines[idx]}"
            ) from exc
        idx += 1

        for _ in range(numblock):
            parts = lines[idx].split()
            if len(parts) != 7:
                raise ParticipantVisibleError(f"Bad block line: {lines[idx]}")
            x1, y1, l1, x2, y2, l2, c = map(int, parts)
            if l1 <= 0 or l2 <= 0:
                raise ParticipantVisibleError(
                    "Layer index in blockage must be positive"
                )
            l1 -= 1
            l2 -= 1

            if x1 != x2:
                if not (y1 == y2 and l1 == l2 and abs(x1 - x2) == 1):
                    raise ParticipantVisibleError("Invalid horizontal blockage")
                xl = min(x1, x2)
                hcap_override[(xl, y1, l1)] = c
            elif y1 != y2:
                if not (x1 == x2 and l1 == l2 and abs(y1 - y2) == 1):
                    raise ParticipantVisibleError("Invalid vertical blockage")
                yb = min(y1, y2)
                vcap_override[(x1, yb, l1)] = c
            else:
                raise ParticipantVisibleError("Invalid blockage (null edge)")
            idx += 1

    return Benchmark(
        path_label=path_label,
        gridx=gridx,
        gridy=gridy,
        layers=layers,
        vcap=vcap,
        hcap=hcap,
        minw=minw,
        mins=mins,
        llx=llx,
        lly=lly,
        xsize=xsize,
        ysize=ysize,
        nets=nets,
        net_index_by_name=net_index_by_name,
        hcap_override=hcap_override,
        vcap_override=vcap_override,
    )


def _warn(buf: List[str], msg: str) -> None:
    buf.append(msg)



def _connected_nodes(adj: Dict[Node, Set[Node]], start: Node) -> Set[Node]:
    visited: Set[Node] = set()
    stack = [start]
    while stack:
        u = stack.pop()
        if u in visited:
            continue
        visited.add(u)
        for v in adj.get(u, set()):
            if v not in visited:
                stack.append(v)
    return visited


def evaluate_benchmark_and_route_text(
    design_text: str,
    route_text: str,
    design_label: str = "<design>",
    route_label: str = "<route>",
) -> RouteEvalResult:
    bench = parse_benchmark_text(design_text, path_label=design_label)

    lines = _non_empty_lines(route_text)
    line_idx = 0
    warnings: List[str] = []

    demand_h: Dict[HKey, int] = {}
    demand_v: Dict[VKey, int] = {}
    net_routes: List[List[Seg]] = [[] for _ in bench.nets]

    while line_idx < len(lines):
        m = re.match(r"^(\S+)\s+(\d+)(?:\s+(\d+))?\s*$", lines[line_idx])
        if not m:
            raise ParticipantVisibleError(f"Bad route net header: {lines[line_idx]}")
        net_name = m.group(1)
        net_id_in_route = int(m.group(2))
        declared_seg_count = int(m.group(3)) if m.group(3) is not None else None
        line_idx += 1

        if net_name not in bench.net_index_by_name:
            raise ParticipantVisibleError(f"Net not found in benchmark: {net_name}")
        ni = bench.net_index_by_name[net_name]
        net_def = bench.nets[ni]
        if net_id_in_route != net_def.net_id:
            _warn(
                warnings,
                f"WARNING net {net_name} wrong id: expected {net_def.net_id}, got {net_id_in_route}",
            )

        adj: Dict[Node, Set[Node]] = {}
        end_points: Set[Node] = set()
        seg_count = 0

        while line_idx < len(lines) and lines[line_idx] != "!":
            m = re.match(
                r"^\((\d+),(\d+),(\d+)\)-\((\d+),(\d+),(\d+)\)\s*$", lines[line_idx]
            )
            if not m:
                raise ParticipantVisibleError(
                    f"ERROR net {net_name} bad route segment: {lines[line_idx]}"
                )
            x1, y1, l1, x2, y2, l2 = map(int, m.groups())
            if l1 <= 0 or l2 <= 0:
                raise ParticipantVisibleError(
                    f"ERROR net {net_name} layer must be positive"
                )
            l1 -= 1
            l2 -= 1

            x1g, y1g = bench.xytogrid(x1, y1)
            x2g, y2g = bench.xytogrid(x2, y2)
            w = (
                max(net_def.min_width, bench.minw[l1])
                if l1 < bench.layers
                else net_def.min_width
            )

            if x1g != x2g:
                if not (y1g == y2g and l1 == l2):
                    raise ParticipantVisibleError(
                        f"ERROR net {net_name} diagonal route"
                    )
                if x2g < x1g:
                    x1g, x2g = x2g, x1g
                for x in range(x1g, x2g):
                    key = (x, y1g, l1)
                    demand_h[key] = demand_h.get(key, 0) + w + bench.mins[l1]
                    u = (x, y1g, l1)
                    v = (x + 1, y1g, l1)
                    adj.setdefault(u, set()).add(v)
                    adj.setdefault(v, set()).add(u)
            elif y1g != y2g:
                if not (x1g == x2g and l1 == l2):
                    raise ParticipantVisibleError(
                        f"ERROR net {net_name} diagonal route"
                    )
                if y2g < y1g:
                    y1g, y2g = y2g, y1g
                for y in range(y1g, y2g):
                    key = (x1g, y, l1)
                    demand_v[key] = demand_v.get(key, 0) + w + bench.mins[l1]
                    u = (x1g, y, l1)
                    v = (x1g, y + 1, l1)
                    adj.setdefault(u, set()).add(v)
                    adj.setdefault(v, set()).add(u)
            elif l1 != l2:
                if not (x1g == x2g and y1g == y2g):
                    raise ParticipantVisibleError(
                        f"ERROR net {net_name} diagonal route"
                    )
                if l2 < l1:
                    l1, l2 = l2, l1
                for lv in range(l1, l2):
                    u = (x1g, y1g, lv)
                    v = (x1g, y1g, lv + 1)
                    adj.setdefault(u, set()).add(v)
                    adj.setdefault(v, set()).add(u)
            else:
                raise ParticipantVisibleError(f"ERROR net {net_name} null route")

            s = (x1g, y1g, l1, x2g, y2g, l2)
            net_routes[ni].append(s)
            end_points.add((x1g, y1g, l1))
            end_points.add((x2g, y2g, l2))
            seg_count += 1
            line_idx += 1

        if line_idx >= len(lines) or lines[line_idx] != "!":
            raise ParticipantVisibleError(
                f"ERROR net {net_name} route missing '!' terminator"
            )
        line_idx += 1

        if declared_seg_count is not None and seg_count != declared_seg_count:
            _warn(
                warnings,
                f"WARNING net {net_name} bad route count: declared {declared_seg_count}, got {seg_count}",
            )

        if net_def.num_pins <= 1000 and net_def.pins_grid_layer:
            start = net_def.pins_grid_layer[0]
            visited = _connected_nodes(adj, start)

            for px, py, pl in net_def.pins_xy_layer:
                gx, gy = bench.xytogrid(px, py)
                pn = (gx, gy, pl)
                if pn not in visited:
                    _warn(
                        warnings,
                        f"net {net_name} pin ({px},{py},{pl + 1}) not attached",
                    )

            for ep in end_points:
                if ep not in visited:
                    raise ParticipantVisibleError(f"ERROR net {net_name} disjoint")

    totnetlen = 0
    tovovernet = 0
    has_unrouted = False

    for ni, net_def in enumerate(bench.nets):
        routes = net_routes[ni]
        if not routes and net_def.num_pins <= 1000:
            xs = [p[0] for p in net_def.pins_grid_layer]
            ys = [p[1] for p in net_def.pins_grid_layer]
            if xs and (min(xs) != max(xs) or min(ys) != max(ys)):
                has_unrouted = True
                _warn(warnings, f"ERROR net {net_def.name} unrouted")

        netlen = 0
        ov = False
        for x1, y1, l1, x2, y2, l2 in routes:
            if x1 < x2:
                for x in range(x1, x2):
                    key = (x, y1, l1)
                    if demand_h.get(key, 0) > bench.cap_h(key):
                        ov = True
                netlen += x2 - x1
            elif y1 < y2:
                for y in range(y1, y2):
                    key = (x1, y, l1)
                    if demand_v.get(key, 0) > bench.cap_v(key):
                        ov = True
                netlen += y2 - y1
            elif l1 < l2:
                netlen += VIA_COST * (l2 - l1)
            else:
                raise ParticipantVisibleError("ERROR inconsistent route segment")

        if ov:
            tovovernet += 1
        totnetlen += netlen

    oedge = 0
    otot = 0
    omax = 0
    for key, d in demand_h.items():
        ov = d - bench.cap_h(key)
        if ov > 0:
            oedge += 1
            otot += ov
            if ov > omax:
                omax = ov
    for key, d in demand_v.items():
        ov = d - bench.cap_v(key)
        if ov > 0:
            oedge += 1
            otot += ov
            if ov > omax:
                omax = ov

    if has_unrouted:
        raise ParticipantVisibleError("ERROR has unrouted net")

    return RouteEvalResult(
        tof=otot,
        mof=omax,
        wl=totnetlen,
        overflowed_nets=tovovernet,
        overflowed_edges=oedge,
        warnings=warnings,
    )


def verify(design_file: Path, route_file: Path) -> Dict[str, int]:
    result = evaluate_benchmark_and_route_text(
        design_text=_open_text(design_file),
        route_text=_open_text(route_file),
        design_label=str(design_file),
        route_label=str(route_file),
    )
    return {
        "tof": result.tof,
        "mof": result.mof,
        "wl": result.wl,
        "overflowed_nets": result.overflowed_nets,
        "overflowed_edges": result.overflowed_edges,
    }


def _pick_column(columns: Iterable[str], candidates: List[str]) -> Optional[str]:
    colset = set(columns)
    for c in candidates:
        if c in colset:
            return c
    return None


def _write_cost_csv(output_path: Path, cost: float) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["id", "cost"])
        writer.writerow([0, cost])


def _competition_name_from_design_path(design_path: Path) -> str:
    stem = design_path.stem.lower()
    normalized = re.sub(r"[^a-z0-9]+", "-", stem).strip("-")
    m = re.match(r"^(.*?)(\d+)$", normalized)
    if m:
        prefix = m.group(1).rstrip("-")
        suffix = m.group(2)
        if prefix:
            normalized = f"{prefix}-{suffix}"
    return f"aap-lab-2-{normalized}"


def _submit_to_kaggle(competition: str, submission_csv: Path, message: str) -> None:
    cmd = [
        "kaggle",
        "competitions",
        "submit",
        competition,
        "-f",
        str(submission_csv),
        "-m",
        message,
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.stdout:
        print(proc.stdout, end="")
    if proc.returncode != 0:
        stderr = proc.stderr.strip() if proc.stderr else "kaggle submit failed"
        raise ParticipantVisibleError(stderr)
    if proc.stderr:
        print(proc.stderr, file=sys.stderr, end="")


def score(solution, submission, row_id_column_name: str) -> float:
    """Kaggle metric entrypoint.

    Expected columns (auto-detected):
    - solution: one of ['design_text', 'benchmark_text', 'input_text', 'case_data']
    - submission: one of ['route_text', 'routing_result', 'output_text']

    Return value is a scalar minimized as lexicographic proxy:
    cost = TOF * 1e12 + MOF * 1e6 + WL
    """
    if row_id_column_name in solution.columns:
        solution = solution.drop(columns=[row_id_column_name])
    if row_id_column_name in submission.columns:
        submission = submission.drop(columns=[row_id_column_name])

    if len(solution) != len(submission):
        raise ParticipantVisibleError("solution/submission row count mismatch")

    design_col = _pick_column(
        solution.columns,
        ["design_text", "benchmark_text", "input_text", "case_data"],
    )
    route_col = _pick_column(
        submission.columns,
        ["route_text", "routing_result", "output_text"],
    )
    if design_col is None:
        raise ParticipantVisibleError("solution missing benchmark text column")
    if route_col is None:
        raise ParticipantVisibleError("submission missing route text column")

    total_cost = 0.0
    for i in range(len(solution)):
        try:
            r = evaluate_benchmark_and_route_text(
                design_text=str(solution.iloc[i][design_col]),
                route_text=str(submission.iloc[i][route_col]),
                design_label=f"row-{i}",
                route_label=f"row-{i}",
            )
        except ParticipantVisibleError:
            raise
        except Exception as exc:
            raise ParticipantVisibleError(f"Failed to evaluate row {i}: {exc}") from exc

        total_cost += float(r.tof) + float(r.mof) * 1e-3 + float(r.wl) * 1e-12

    return total_cost / float(len(solution))


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Pure Python verifier for ISPD2008 routing output"
    )
    parser.add_argument("design", help="Path to benchmark .gr input file")
    parser.add_argument("route", help="Path to routed output file")
    parser.add_argument(
        "--submit",
        action="store_true",
        help="Submit generated CSV using Kaggle CLI",
    )
    parser.add_argument(
        "--competition",
        default=None,
        help="Kaggle competition name override (default: infer from design filename)",
    )
    parser.add_argument(
        "-m",
        "--message",
        default="v1",
        help="Submission message used by Kaggle CLI",
    )
    parser.add_argument(
        "-H",
        "--no-header",
        action="store_true",
        help="Do not print table header",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print result in JSON format",
    )

    return parser


def _run_local() -> int:
    parser = _build_arg_parser()
    args = parser.parse_args()

    output_csv: Optional[Path] = None

    try:
        design_path = Path(args.design)
        res = evaluate_benchmark_and_route_text(
            design_text=_open_text(design_path),
            route_text=_open_text(Path(args.route)),
            design_label=args.design,
            route_label=args.route,
        )
        total_cost = float(res.tof) + float(res.mof) * 1e-3 + float(res.wl) * 1e-12

        if args.submit and len(res.warnings) == 0:
            with tempfile.NamedTemporaryFile(
                prefix=f"{design_path.stem}_", suffix=".csv", delete=False
            ) as tmp:
                output_csv = Path(tmp.name)
            _write_cost_csv(output_csv, total_cost)
            competition = args.competition or _competition_name_from_design_path(
                design_path
            )
            print(
                f"Submitting {output_csv} to competition '{competition}' with message '{args.message}'..."
            )
            _submit_to_kaggle(competition, output_csv, args.message)
    except ParticipantVisibleError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"UnexpectedError: {exc}", file=sys.stderr)
        return 3
    finally:
        if args.submit and output_csv is not None:
            try:
                output_csv.unlink(missing_ok=True)
            except OSError:
                pass

    for w in res.warnings:
        print(w, file=sys.stderr)

    if args.json:
        payload = {
            "tof": res.tof,
            "mof": res.mof,
            "wl": res.wl,
            "overflowed_nets": res.overflowed_nets,
            "overflowed_edges": res.overflowed_edges,
        }
        print(json.dumps(payload, ensure_ascii=True))
        return 0

    if not args.no_header:
        print(f"{'File Names(In, Out)':<35} {'Tot OF':>12} {'Max OF':>11} {'WL':>13}")
    file_label = f"{args.design}, {args.route}"
    print(f"{file_label:<35} {res.tof:>12} {res.mof:>11} {res.wl:>13}")
    return 0


if __name__ == "__main__":
    raise SystemExit(_run_local())