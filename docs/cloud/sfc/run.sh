#!/bin/sh
# Usage: run.sh <matiec-tree> <label> <outdir>
# Compiles pulse.st with <matiec-tree>/iec2c, builds harness.c against the output, runs it.
set -u
MATIEC=$1; LABEL=$2; OUT=$3
HERE=$(cd "$(dirname "$0")" && pwd)
rm -rf "$OUT"; mkdir -p "$OUT"
cd "$OUT" || exit 2
echo "matiec: $(git -C "$MATIEC" rev-parse --short HEAD)"
timeout 30 "$MATIEC/iec2c" -I "$MATIEC/lib" "$HERE/pulse.st"; rc=$?
echo "iec2c exit=$rc"; [ $rc -eq 0 ] || exit 3
gcc -Wall -Wno-unused -I "$MATIEC/lib/C" -I . -o harness "$HERE/harness.c" config.c res.c -lm 2>&1
echo "gcc exit=$?"
timeout 30 ./harness "$LABEL"; echo "harness exit=$?"
