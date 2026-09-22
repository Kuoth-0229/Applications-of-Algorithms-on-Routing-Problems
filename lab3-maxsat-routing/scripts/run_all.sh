#!/usr/bin/env bash
# Build, route every case, and run the verifier.  Single-threaded; each case is
# independently bounded well under the 10-minute limit.
set -uo pipefail
cd "$(dirname "$0")/.."

export LD_LIBRARY_PATH="$PWD/lib"
make >/dev/null

CASES=("$@")
if [ ${#CASES[@]} -eq 0 ]; then
    CASES=(case1 case2 case3 case4 case5 bonus)
fi

mkdir -p result
for c in "${CASES[@]}"; do
    t0=$(date +%s)
    ./Lab3 "case/${c}.txt" "result/${c}.out"
    t1=$(date +%s)
    res=$(./verifier "result/${c}.out" "case/${c}.txt" 2>&1 \
          | grep -E 'wire length|via number|cost|correct' | tr '\n' ' ')
    printf '[%s] %ds | %s\n' "$c" "$((t1-t0))" "$res"
done
