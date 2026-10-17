#!/bin/sh
# CD audio playback test of rtl/cd_top.vhd against the upstream original (ref/upstream/rtl/cd_top.vhd), see
# tb_cd_cdda.vhd. Both have their synthesis translate_off regions removed (debug output; the hardware code paths).
# The default "new" side is ref/pocket/rtl/cd_top.vhd, the Pocket controller before the CD engine (docs/cd_engine.md):
# the engine version is checked against that one, clock by clock, by sim/cd_equiv (script 2: CD audio).
# usage: ALTERA_MF_DIR=<compiled altera_mf dir> [NEWFILE=<cd_top.vhd variant>] [REFFILE=<reference cd_top.vhd>] sim/cd_cdda/run.sh
set -e
here=$(cd "$(dirname "$0")" && pwd)
rtl="$here/../../rtl"
work="${WORKDIR:-$here/work}"
mkdir -p "$work"
prep() { python3 -c '
import re, sys
s = sys.stdin.read().replace("entity cd_top is", "entity cd_top_%s is" % sys.argv[1]).replace("architecture arch of cd_top is", "architecture arch of cd_top_%s is" % sys.argv[1])
s = re.sub(r"--\s*synthesis translate_off.*?--\s*synthesis translate_on[^\n]*", "", s, flags=re.S)
# simulation only (as in pocket/sim/boot): to_unsigned of negative values aborts in numeric_std, wraps in hardware
fn = ("   function to_unsigned_w(arg : integer; size : natural) return unsigned is\n"
      "   begin\n      if (arg < 0) then return unsigned(to_signed(arg, size)); else return to_unsigned(arg, size); end if;\n"
      "   end function;\n")
m = re.search(r"architecture \w+ of \w+ is[ \t]*(--[^\n]*)?\n", s, flags=re.I)
s = s[:m.end()] + fn + re.sub(r"\bto_unsigned\(", "to_unsigned_w(", s[m.end():])
sys.stdout.write(s)' "$1"; }
prep ref < "${REFFILE:-$here/../../ref/upstream/rtl/cd_top.vhd}" > "$work/cd_top_ref.vhd"
prep new < "${NEWFILE:-$here/../../ref/pocket/rtl/cd_top.vhd}" > "$work/cd_top_new.vhd"
cd "$work"
L="-L . -L ${ALTERA_MF_DIR:-.}"
nvc --std=2008 $L --work=mem -a "$rtl/SyncFifo.vhd" "$rtl/SyncFifoFallThrough.vhd" "$rtl/RamMLAB.vhd" "$rtl/dpram.vhd"
nvc --std=2008 $L -a "$rtl/dpram.vhd" "$rtl/cd_xa_zigzag.vhd" "$rtl/cd_xa.vhd" cd_top_ref.vhd cd_top_new.vhd "$here/tb_cd_cdda.vhd"
nvc --std=2008 --ieee-warnings=off -H 1g $L -e tb_cd_cdda -r
