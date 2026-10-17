#!/bin/sh
# CD controller equivalence test: rtl/cd_top.vhd against the frozen reference ref/pocket/rtl/cd_top.vhd, every
# output compared on every clock (see tb_cd_equiv.vhd). Both have their synthesis translate_off regions removed.
# usage: ALTERA_MF_DIR=<compiled altera_mf dir> [NEWFILE=<cd_top.vhd variant>] [REFFILE=<reference cd_top.vhd>]
#        [EXTRA="<rtl files the new instance needs, default: cd_fw_pkg, cd_engine, cd_ctrl>"] [FREEZE=1] [CPUSRC=1]
#   FREEZE=1: the testbench holds everything else while the engine runs: cycle exact comparison (PASS = no mismatch)
#   CPUSRC=1 (without FREEZE): the CPU model uses the new instance, which runs as in hardware; compare the summary
#   line (interrupts, sectors, data checksum) with the same script's reference run
#        sim/cd_equiv/run.sh SCRIPT [SEED] [NCMD] [HASCD] [RANDCE]
#   SCRIPT: 0 BIOS-like boot, 1 SotN-like loading + XA, 2 CD audio, 3 random commands (SEED, NCMD),
#           4 SotN music (XA only zone, XAMS ms of play; with XAMS=<ms>),
#           5 a song from real sectors: DISCWIN=<hex window> WINLBA= WINN= PLAYLBA= XAFILE= XACHAN= T1N= XAMS=
#             [SLOWEVERY=n SLOWUS=us: every n-th sector read slow] [OUTPRE=<path prefix>: output samples per instance]
set -e
here=$(cd "$(dirname "$0")" && pwd)
rtl="$here/../../rtl"
work="${WORKDIR:-$here/work}"
mkdir -p "$work"
prep() { python3 -c '
import re, sys
s = sys.stdin.read().replace("entity cd_top is", "entity cd_top_%s is" % sys.argv[1]).replace("architecture arch of cd_top is", "architecture arch of cd_top_%s is" % sys.argv[1])
s = re.sub(r"--\s*synthesis translate_off.*?--\s*synthesis translate_on[^\n]*", "", s, flags=re.S)
# a controller without the engine (the reference as NEWFILE): add the engine ports the testbench connects
if sys.argv[1] == "new" and "eng_busy" not in s:
   q = chr(39) + "0" + chr(39)
   s = re.sub(r"(entity cd_top_new is\s*\n)(\s*port\s*\n\s*\(\s*\n)",
              lambda m: m.group(1) + "   generic (SIM_FREEZE : boolean := false);\n" + m.group(2) +
              "      clk_engine : in std_logic := " + q + ";\n      eng_busy : out std_logic := " + q + ";\n", s, count=1)
# the parameter fifo fill count is a 5 bit counter in hardware (wraps); its overflow guard is simulation only
s = s.replace("signal fifoParamCount            : integer range 0 to 16;", "signal fifoParamCount            : integer range 0 to 31;")
s = s.replace("fifoParamCount <= fifoParamCount + 1;", "fifoParamCount <= (fifoParamCount + 1) mod 32;")
s = s.replace("fifoParamCount <= fifoParamCount - 1;", "fifoParamCount <= (fifoParamCount + 31) mod 32;")
# simulation only (as in pocket/sim/boot): to_unsigned of negative values aborts in numeric_std, wraps in hardware
fn = ("   function to_unsigned_w(arg : integer; size : natural) return unsigned is\n"
      "   begin\n      if (arg < 0) then return unsigned(to_signed(arg, size)); else return to_unsigned(arg, size); end if;\n"
      "   end function;\n")
m = re.search(r"architecture \w+ of \w+ is[ \t]*(--[^\n]*)?\n", s, flags=re.I)
s = s[:m.end()] + fn + re.sub(r"\bto_unsigned\(", "to_unsigned_w(", s[m.end():])
sys.stdout.write(s)' "$1"; }
prep ref < "${REFFILE:-$here/../../ref/pocket/rtl/cd_top.vhd}" > "$work/cd_top_ref.vhd"
prep new < "${NEWFILE:-$rtl/cd_top.vhd}" > "$work/cd_top_new.vhd"
cd "$work"
L="-L . -L ${ALTERA_MF_DIR:-.}"
nvc --std=2008 $L --work=mem -a "$rtl/SyncFifo.vhd" "$rtl/SyncFifoFallThrough.vhd" "$rtl/RamMLAB.vhd" "$rtl/dpram.vhd"
nvc --std=2008 $L -a "$rtl/dpram.vhd" "$rtl/cd_xa_zigzag.vhd" "$rtl/cd_xa.vhd" ${EXTRA-"$rtl/cd_fw_pkg.vhd" "$rtl/cd_engine.vhd" "$rtl/cd_ctrl.vhd"} \
    cd_top_ref.vhd cd_top_new.vhd "$here/tb_cd_equiv.vhd"
F=false; [ -n "$FREEZE" ] && F=true
opt=""   # script 5 / disc window / slow reads / output capture: only the generics that are set (paths without spaces)
for g in T1N DISCWIN WINLBA WINN SLOWEVERY SLOWUS PLAYLBA XAFILE XACHAN OUTPRE; do
   eval v=\${$g:-}
   [ -n "$v" ] && opt="$opt -g$g=$v"
done
nvc --std=2008 --ieee-warnings=off -H 1g $L -e tb_cd_equiv -gSCRIPT=${1:-3} -gSEED=${2:-1} -gNCMD=${3:-200} \
    -gHASCD=${4:-1} -gRANDCE=${5:-1} -gFREEZE=$F -gCPUSRC=${CPUSRC:-0} -gDBGMS=${DBGMS:--1} -gXAMS=${XAMS:-2000} $opt -r
