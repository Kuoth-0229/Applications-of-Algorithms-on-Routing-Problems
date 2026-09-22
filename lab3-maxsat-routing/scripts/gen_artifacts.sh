#!/usr/bin/env bash
# Generate CSV (Kaggle submission format) and a PNG visualisation for each
# routing output, using the project virtualenv.  All artifacts stay inside the
# project's result/ tree.
#
# Usage: scripts/gen_artifacts.sh [case1 case2 ...]   (default: all six cases)
set -euo pipefail
cd "$(dirname "$0")/.."

PY=venv/bin/python
CASES=("$@")
if [ ${#CASES[@]} -eq 0 ]; then
    CASES=(case1 case2 case3 case4 case5 bonus)
fi

mkdir -p result/csv result/png

for c in "${CASES[@]}"; do
    out="result/${c}.out.txt"
    if [ ! -s "$out" ]; then
        echo "[skip] $out missing/empty"
        continue
    fi
    "$PY" convert_submission_to_csv.py "$out" "result/csv/${c}.csv"
    "$PY" scripts/plot_case.py "$out" "result/png/${c}.png"
    echo "[ok] $c -> result/csv/${c}.csv , result/png/${c}.png"
done
