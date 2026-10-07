#!/bin/bash
set -e
timeout -k 5 120 halrun -f cmp.hal

# simple_tp with max_jerk: the jog planner of motion (simple_tp.c with the
# S-curve of sp_scurve.c), built as in tests/blendmath: only the functions
# it uses are linked
TOPDIR=$(dirname "$HEADERS")
trap 'rm -f ref_simple_tp' EXIT
gcc -O2 -Wall -ffunction-sections -fdata-sections -DULAPI \
    -I"$HEADERS" -I"$TOPDIR/src" -I"$TOPDIR/src/emc" -I"$TOPDIR/src/emc/motion" \
    -I"$TOPDIR/src/emc/tp" -I"$TOPDIR/src/emc/nml_intf" \
    -o ref_simple_tp ref_simple_tp.c "$TOPDIR/src/emc/motion/simple_tp.c" \
    "$TOPDIR/src/emc/tp/sp_scurve.c" -Wl,--gc-sections -lm
timeout -k 5 120 halrun -f cmpj.hal
