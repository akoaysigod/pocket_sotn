#!/bin/sh
# Boot simulation of the Pocket core with the real BIOS (see tb_boot.vhd). Compiles the same source set as
# pocket/src/fpga/psx_pocket.qip (reset-only savestates, black MDEC, small GTE), with the synthesis
# translate_off regions removed (debug file output) and the simulation-only fixups of sim/gpu_vram.
# usage: BIOS=<scph1001.bin> ALTERA_MF_DIR=<compiled altera_mf dir> [RTL_OVERRIDE="<file.vhd> ..."] [WORKDIR=<dir>] [VRAM_PSRAM=<vram_psram variant>] \
#        pocket/sim/boot/run.sh [RUN_MS] [DUMP_EVERY]
set -e
here=$(cd "$(dirname "$0")" && pwd)
repo="$here/../../.."
rtl="$repo/rtl"
core="$repo/pocket/src/fpga/core"
work="${WORKDIR:-$here/work}"; mkdir -p "$work/src"; cd "$work"

python3 - "$rtl" "$work/src" <<'EOF'
import os, re, sys
rtl, out = sys.argv[1], sys.argv[2]
swap = {'savestates.vhd': 'savestates_reset.vhd', 'gte.vhd': 'gte_small.vhd', 'mdec.vhd': 'mdec_black.vhd'}
names = [n for n in os.listdir(rtl) if n.endswith('.vhd')]
override = {os.path.basename(f): f for f in os.environ.get('RTL_OVERRIDE', '').split()}   # same-named replacements
for n in names:
    s = open(override.get(n, os.path.join(rtl, n))).read()
    # drop synthesis translate_off .. translate_on (debug file output, not in the hardware)
    s = re.sub(r'--\s*synthesis translate_off.*?--\s*synthesis translate_on[^\n]*', '', s, flags=re.S)
    open(os.path.join(out, n), 'w').write(s)
for dst, src in swap.items():
    os.replace(os.path.join(out, src), os.path.join(out, dst + '.pocket'))
    os.rename(os.path.join(out, dst + '.pocket'), os.path.join(out, dst))
# simulation only: to_unsigned / to_signed of out-of-range values (e.g. negative right after reset) abort in
# numeric_std but wrap to the low bits in hardware; use wrapping versions in every architecture
fn = ('   function to_unsigned_w(arg : integer; size : natural) return unsigned is\n'
      '   begin\n      if (arg < 0) then return unsigned(to_signed(arg, size)); else return to_unsigned(arg, size); end if;\n'
      '   end function;\n')
for n in os.listdir(out):
    p = os.path.join(out, n)
    s = open(p).read()
    m = re.search(r'architecture \w+ of \w+ is[ \t]*(--[^\n]*)?\n', s, flags=re.I)
    if m and 'to_unsigned(' in s[m.end():]:
        s = s[:m.end()] + fn + re.sub(r'\bto_unsigned\(', 'to_unsigned_w(', s[m.end():])
        open(p, 'w').write(s)
# simulation only (as in sim/gpu_vram): undriven inout record fields of the divider ports default to 'Z'
for m in ('poly', 'line'):
    p = os.path.join(out, 'gpu_%s.vhd' % m)
    s = open(p).read()
    s = re.sub(r"(div[1-6] +: inout div_type)", r"\1 := (start => 'Z', done => 'Z', dividend => (others => 'Z'), divisor => (others => 'Z'), quotient => (others => 'Z'), remainder => (others => 'Z'))", s)
    open(p, 'w').write(s)
EOF

# BIOS as text, one little-endian word per line
python3 -c '
import struct, sys
d = open(sys.argv[1], "rb").read()
open("bios.hex", "w").write("".join("%08X\n" % w for w in struct.unpack("<%dI" % (len(d) // 4), d)))' "${BIOS:?set BIOS to the BIOS file}"

L="-L . -L ${ALTERA_MF_DIR:-.}"
S=src
nvc --std=2008 $L --work=mem -a --relaxed $S/RamMLAB.vhd $S/RamM10Kn.vhd $S/dpram.vhd $S/SyncFifo.vhd $S/SyncFifoFallThrough.vhd \
   $S/SyncFifoFallThroughMLAB.vhd
nvc --std=2008 $L -a --relaxed $S/RamMLAB.vhd $S/RamM10Kn.vhd $S/dpram.vhd $S/SyncFifo.vhd $S/SyncFifoFallThrough.vhd \
   $S/SyncFifoFallThroughMLAB.vhd $S/SyncRam.vhd $S/SyncRamDual.vhd $S/SyncRamDualNotPow2.vhd $S/SyncRamDualByteEnable.vhd \
   $S/ValidBits256.vhd $S/export.vhd $S/divider.vhd $S/pGPU.vhd $S/mul32u.vhd $S/mul9s.vhd $S/cheats.vhd \
   $S/gpu_fillVram.vhd $S/gpu_cpu2vram.vhd $S/gpu_vram2vram.vhd $S/gpu_vram2cpu.vhd $S/gpu_line.vhd $S/gpu_rect.vhd \
   $S/gpu_poly.vhd $S/gpu_pixelpipeline.vhd $S/gpu_overlay.vhd $S/gpu_dither.vhd $S/gpu_videoout_async.vhd \
   $S/gpu_videoout_sync.vhd $S/gpu_crosshair.vhd $S/justifier_sensor.vhd $S/gpu_videoout.vhd $S/gpu.vhd $S/irq.vhd \
   $S/pJoypad.vhd $S/joypad_pad.vhd $S/joypad_mem.vhd $S/joypad.vhd $S/timer.vhd $S/dma.vhd $S/exp2.vhd $S/pGTE.vhd \
   $S/gte_mac0.vhd $S/gte_mac123.vhd $S/gte_UNRDivide.vhd $S/pGTEs.vhd $S/gte_mac123s.vhd $S/gte.vhd $S/mdec.vhd \
   $S/cd_xa_zigzag.vhd $S/cd_xa.vhd $S/cd_fw_pkg.vhd $S/cd_engine.vhd $S/cd_ctrl.vhd $S/cd_top.vhd $S/memctrl.vhd $S/sio.vhd $S/spu_ram.vhd $S/spu_gauss.vhd $S/spu.vhd \
   $S/datacache.vhd $S/cpu.vhd $S/memorymux.vhd $S/memcard.vhd $S/statemanager.vhd $S/savestates.vhd $S/psx_top.vhd \
   $S/psx_mister.vhd
nvc --std=2008 $L -a --relaxed "$core/psram_async.vhd" "${VRAM_PSRAM:-$core/vram_psram.vhd}" "$core/psx_pocket_core.vhd" \
   "$here/../vram_psram/psram_model.vhd" "$here/ram_model.vhd" "$here/tb_boot.vhd"
nvc --std=2008 --ieee-warnings=off $L -H 3g -M 64m -e tb_boot -gRUN_MS=${1:-2000} -gDUMP_EVERY=${2:-20} -gDEBUG_SS=${DEBUG_SS:-0} -gTRACE_IO_N=${TRACE_IO_N:-0} -gTRACE_FROM_MS=${TRACE_FROM_MS:-0} -gFAST_SHELL=${FAST_SHELL:-false} -gSKIP_INTRO=${SKIP_INTRO:-false} -r
