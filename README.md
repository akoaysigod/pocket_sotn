# PSX core on the Analogue Pocket

A PlayStation core for the Analogue Pocket, made by trimming the MiSTer PSX core until it fits the Pocket's FPGA
(Cyclone V 5CEBA4F23C8: 18,480 ALMs, 308 M10K, 66 DSP), with a Pocket wrapper for SDRAM, PSRAM, video, audio,
controller, CD images from the SD card and memory card saves. The trims were chosen with Symphony of the Night
(USA, SLUS-00067) as the test game; other games may use features that were removed.

Based on [MiSTer-devel/PSX_MiSTer](https://github.com/MiSTer-devel/PSX_MiSTer) at commit `cd17b5a`
(2026-08-21), GPL-2.0 (see `LICENSE`). Upstream originals of the files the equivalence tests compare against are
in `ref/upstream/rtl/`.

## Licences

- The PSX core (`rtl/`), the Pocket wrapper code written for this port and the tests: GPL, see `LICENSE`
  (GPL-2.0, as upstream PSX_MiSTer; a few upstream files such as `rtl/sdram.sv` say GPL-3.0 or later in their
  headers). Sharing the source or a built bitstream means sharing it under the GPL, with the source available.
- `pocket/src/fpga/apf/`: Analogue Pocket Framework, under Analogue's licence (header of each file and the Pocket
  EULA), for building Pocket cores; GPL terms prevail where they apply.
- `pocket/src/fpga/core/sound_i2s.sv`, `sync_fifo.sv` and `psram_async.vhd` (a port of `psram.sv`): MIT, Adam
  Gastineau ([agg23/analogue-pocket-utils](https://github.com/agg23/analogue-pocket-utils)); the notice is in
  each file.
- Not included and not redistributable: the BIOS (Sony), game images, and Intel's `altera_mf` simulation library
  (`sim/altera_lib`).

## Status (version 0.1.4, same core as 0.1.2: build cc seed 1, 2026-10-15)

Tested on hardware: the game runs at full speed (60 fps), the opening movies (FMVs) play, memory card saves work
(saved per game by the Pocket, quit the core to write the file), L2 / R2 come from button combos on the handheld
(Core Settings, roadmap item 1), and CD audio tracks (CD-DA) play in the BIOS CD player (roadmap item 2). SotN
itself plays no CD audio: its music, very likely including the ending song, is XA-ADPCM; its track 2 is a spoken
"this is a PlayStation disc" warning followed by music, for people who put the disc in a CD player.

Branch `cd-audio`: CD audio tracks (CD-DA) are on again, for the BIOS CD player and SotN's track 2 (a spoken
"this is a PlayStation disc" warning followed by music, for people who put the disc in a CD player); tested on
hardware on its own (roadmap item 2). Build cc seed 1 has CD audio and the L2 / R2 combos together (to be tested
on hardware). SotN itself plays no CD audio: its music, very likely including the ending song, is XA-ADPCM, which
works in every build.

Not supported:
- Save states, memory card 2, EXE loading, LibCrypt, cheats, the newest CD Pause timing refinement (upstream `cd17b5a`).

After a build from this core has run, the Pocket's PSRAM stays in synchronous mode until power-off: power the
Pocket off before switching to another core.

## Roadmap

- Random freeze during boot when pressing button?
- Long loads
- Boss fight/particles cause frame rate to dip

## Layout

| Path | What |
| --- | --- |
| `rtl/` | the PSX core (MiSTer sources, trimmed). The build uses the 68 files listed in `pocket/src/fpga/psx_pocket.qip`; the others are upstream originals used by tests (e.g. `gte.vhd`, `savestates.vhd`) |
| `pocket/src/fpga/` | Quartus project (`ap_core.qpf`), APF framework (`apf/`), Pocket wrapper (`core/`) |
| `pocket/dist/` | Pocket JSON files (`Cores/akoaysigod.PSX`, `Platforms/psx.json`) |
| `pocket/tools/` | `package.py` (SD card folders), `cue2instance.py` (game JSON), `rbf_reverse.py` |
| `pocket/sim/`, `sim/` | simulation tests (nvc), see below |
| `ref/upstream/rtl/` | upstream versions of `dma`, `spu`, `gpu`, `gpu_poly`, `gpu_line` (references for the tests) |
| `tools/` | build in Docker, seed search, timing reports |
| `docs/` | `NIGHT_LOG.md` (every change, its test and its measurement, in order), SotN hardware usage, notes |

Main pieces of the Pocket wrapper (`pocket/src/fpga/core/`):

| Part | Where it lives on the Pocket |
| --- | --- |
| Main RAM, BIOS, SPU RAM | SDRAM (`rtl/sdram.sv`, clk3x = 101.6 MHz) |
| VRAM, memory card | both PSRAMs side by side as one 32-bit memory, synchronous burst mode (`vram_psram.vhd`) |
| CD image | read sector by sector from the game's `.bin` files through APF data slots (`pocket_cd.vhd`) |
| Memory card file | APF save slot, loaded into / written back from PSRAM (`pocket_memcard.vhd`) |
| Video, audio, controller | `pocket_video.vhd`, `sound_i2s.sv`, `pocket_psx.vhd`, `core_top.v` |

## Build

Needs Docker with the image, only because I have an ARM mac, `ghcr.io/nikleberg/quartus:25.1` and a container holding the Cyclone V device files
(created from `ghcr.io/nikleberg/quartus:25.1-cyclonev`, by default named `mirlo-cyclonev`; another name can be
given in `QUARTUS_DEVICES`). Quartus runs single-threaded (its parallel helpers hang under Rosetta on Apple
Silicon); one full build takes about 35 minutes and ~2 GB of Docker memory.

    tools/build.sh NAME              # full build -> builds/NAME/pocket/src/fpga/output_files/ap_core.rbf_r
    ELAB=1 tools/build.sh NAME       # HDL check only (~1 minute)
    MAPONLY=1 tools/build.sh NAME    # synthesis only (resource estimate)
    SEED=1 tools/build.sh NAME       # fitter seed (releases 0.1.2 to 0.1.4 = build cc seed 1)

The chip is about 99 % full: depending on the fitter seed, a build may not fit or may miss timing by a fraction of a
nanosecond (with the SDRAM change most seeds pass). `tools/seeds.sh NAME` tries seeds until one fits and meets timing at all four corners (two chains with
different `SEEDS="..."` can run side by side). Then:

    tools/report.sh NAME                          # resources, slack per clock and corner, biggest modules
    tools/sta.sh NAME tools/worst_paths.tcl       # worst paths
    tools/sta.sh NAME tools/m10kn_clocks.tcl      # check the falling-edge RAMs (RamM10Kn) are fed by their own clock only

## Put it on the SD card

    tools/release.sh            # newest build -> releases/akoaysigod.PSX_<version>_<build>.zip

The zip has the full layout the Pocket expects, with every file except the copyrighted ones. Unzip it at the SD card's
root, then add your BIOS and games (see "Adding a game" below):

    Cores/akoaysigod.PSX/                         bitstream.rbf_r, JSON files, LICENSE.txt
    Platforms/psx.json
    Assets/psx/common/                            <- your BIOS, named scph1001.bin (512 KB)
    Assets/psx/akoaysigod.PSX/BIOS (no disc).json    boots the BIOS menu without a disc
    Assets/psx/akoaysigod.PSX/<Game>.json         <- one entry per game (see below)
    Assets/psx/akoaysigod.PSX/<Game>/             <- that game's .bin files

The zip can be shared (with the source, GPL). `Example.json` and `Example/README.txt` in the zip repeat the steps below.

### Adding a game

Games are one `.bin` per track (Redump style) plus the `.cue`. Keep them in `games/<Game>/` (git-ignored), then
either build a card with the game in it:

    tools/release.sh --bios SCPH1001.BIN --game "games/<Game>/<Game>.cue" --copy-bins

or make the entry by hand: `python3 pocket/tools/cue2instance.py "games/<Game>/<Game>.cue" "<Game>.json"` writes the
instance JSON (track 1 = data slot 16, track 2 = slot 17, ...); copy it to `Assets/psx/akoaysigod.PSX/` and the `.bin`
files to `Assets/psx/akoaysigod.PSX/<folder named in data_path>/`.

Worked example, Symphony of the Night (`games/Castlevania - Symphony of the Night/`, two tracks):
`pocket/examples/Castlevania - Symphony of the Night.json` is what `cue2instance.py` writes for it, and
`pocket/examples/CD player test (SotN track 2 as an audio CD).json` shows the audio CD trick (data slot 15 instead
of 16 marks track 1 as audio, so the BIOS opens its CD player). Redump names with "(USA)" must be renamed to match
the `.json`, or generate the entry from the `.cue`.

Other forms:

    tools/release.sh NAME                       # a given build in builds/
    tools/release.sh path/to/bitstream.rbf_r    # a given bitstream file
    tools/release.sh NAME --bios SCPH1001.BIN \
        --game "games/<Game>/<Game>.cue" --copy-bins
                                                # with your BIOS and game in it: ..._personal.zip, don't share it
    python3 pocket/tools/package.py --rbf BITSTREAM.rbf_r --out SDROOT [--bios ...] [--game ... --copy-bins]
                                                # the same layout as a folder

To update only the core on a card that already has everything, copying `bitstream.rbf_r` to `Cores/akoaysigod.PSX/` is
enough.

The BIOS (`SCPH1001.BIN`) and the game files (`games/`) are not part of this repository (`.gitignore`); bring your own.

## Tests

The changes to the core were checked in simulation against the upstream originals, most also with a deliberately
broken variant to show the test catches errors; `docs/NIGHT_LOG.md` lists each change with its test. The tests use [nvc](https://github.com/nickg/nvc)
and Intel's `altera_mf` library compiled for nvc in `sim/altera_lib` (not in git: compile `altera_mf_components.vhd`
and `altera_mf.vhd` from a Quartus installation into a library named `altera_mf` there).

    ALTERA_MF_DIR=$PWD/sim/altera_lib sh sim/dma_equiv/run.sh 300000 1
    ALTERA_MF_DIR=$PWD/sim/altera_lib sh sim/spu_equiv/run.sh 2000000 1
    BIOS=$PWD/SCPH1001.BIN ALTERA_MF_DIR=$PWD/sim/altera_lib sh pocket/sim/boot/run.sh 400   # BIOS boot, ms

Each `run.sh` has its usage on the first lines. `sim/`: equivalence of the trimmed modules (CD helpers, CPU
multiplier, CD audio playback against upstream's `cd_top`, DMA, GPU dividers and VRAM, GTE, MDEC stub, memory
card 2 removal, RAM primitives, reset-only save states, SPU). `pocket/sim/`: the Pocket wrapper (PSRAM controller
with a PSRAM model, CD reader, memory card, L2 / R2 button combos,
video, diagnostics page) and a full BIOS boot of the whole core.

`sim/sdram` tests the SDRAM controller (`rtl/sdram.sv`, SystemVerilog) with [Verilator](https://verilator.org) 5 in
Docker (the `mirlo-dev` image, or `VERILATOR_IMAGE`): it and a reference controller each get a behavioural SDRAM
chip that checks command timing and the same random CPU / cache / DMA / SPU / DMA-fifo traffic; every read is
checked, and with `LOCKSTEP` both must drive the same outputs on every clock:

    sh sim/sdram/run.sh 20000 1 2        # 20 ms, seed 1, at least 2 clk1x between CPU requests
