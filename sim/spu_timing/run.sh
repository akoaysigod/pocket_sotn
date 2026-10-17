#!/bin/sh
# SPU timing per output sample against the sound RAM latency (see tb_spu_timing.vhd).
# usage: ALTERA_MF_DIR=<compiled altera_mf dir> [SPUFILE=<spu.vhd without the CD queue> CDQ=0]
#        sim/spu_timing/run.sh [LAT] [LATJIT] [PITCH] [REVERB] [SAMPLES]
set -e
here=$(cd "$(dirname "$0")" && pwd)
rtl="$here/../../rtl"
work="${WORKDIR:-$here/work}"
mkdir -p "$work"
python3 -c '
import sys
s = sys.stdin.read().replace("entity spu is", "entity spu_new is").replace("architecture arch of spu is", "architecture arch of spu_new is")
a = s.find("goutput : if")
if a != -1:
    a = s.rfind("-- synthesis translate_off", 0, a + 40)
    b = s.index("-- synthesis translate_on", a)
    s = s[:a] + s[b:]
if "sound_valid" not in s:   # an SPU from before the CD queue: the new outputs, unused
    s = s.replace("      sound_out_right      : out std_logic_vector(15 downto 0) := (others => \x27" + "0\x27);",
                  "      sound_out_right      : out std_logic_vector(15 downto 0) := (others => \x27" + "0\x27);\n      sound_valid : out std_logic := \x270\x27;\n      late : out std_logic := \x270\x27;", 1)
sys.stdout.write(s)' < "${SPUFILE:-$rtl/spu.vhd}" > "$work/spu_new.vhd"
cd "$work"
L="-L . -L ${ALTERA_MF_DIR:-.}"
nvc --std=2008 $L --work=mem -a "$rtl/SyncFifo.vhd" "$rtl/SyncFifoFallThrough.vhd" "$rtl/RamMLAB.vhd" "$rtl/RamM10Kn.vhd" "$rtl/dpram.vhd"
nvc --std=2008 $L -a "$rtl/dpram.vhd" "$rtl/spu_gauss.vhd" "$rtl/spu_ram.vhd" spu_new.vhd "$here/tb_spu_timing.vhd"
nvc --std=2008 $L -e tb_spu_timing -gLAT=${1:-5} -gLATJIT=${2:-0} -gPITCH=${3:-4096} -gREVERB=${4:-1} -gSAMPLES=${5:-1500} -gCDQ=${CDQ:-1} \
    -r --ieee-warnings=off
