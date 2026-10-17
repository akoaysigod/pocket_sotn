#!/bin/sh
# pocket_audio48 (44.1 -> 48 kHz for the Pocket's I2S) against an ideal resample.
# usage: sim/audio48/run.sh IN.txt [NSAMP] [LATE]   (IN.txt: one "L R" pair of 44.1 kHz samples per line; LATE:
#        every LATE-th SPU sample finishes after the next tick)
set -e
here=$(cd "$(dirname "$0")" && pwd)
work="${WORKDIR:-$here/work}"
mkdir -p "$work"
cd "$work"
nvc --std=2008 -a "$here/../../pocket/src/fpga/core/pocket_audio48.vhd" "$here/tb_audio48.vhd"
nvc --std=2008 -e tb_audio48 -gINFILE="$1" -gOUTFILE="$work/out.txt" -gNSAMP=${2:-13230} -gLATE=${3:-0} -gINTERP=${INTERP:-true} -r
cc -O2 -o check "$here/check.c" -lm
./check "$1" "$work/out.txt"
