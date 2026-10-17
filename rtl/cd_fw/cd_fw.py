"""Firmware of the CD controller engine (rtl/cd_engine.vhd, rtl/cd_ctrl.vhd; design: docs/cd_engine.md).

Runs once per clock edge at which the original command processing / drive processes of cd_top
(ref/pocket/rtl/cd_top.vhd, "P2" lines 889-1717 and "P3" lines 1781-2397) do more than cd_ctrl's fast path.
It is a translation of those process bodies in their statement order:
  * reads see the values before the edge: hardware values through the snapshots (IN ports), the engine's own
    variables through their "old" copy V_*; writes go to the "new" copy N_* (committed at the end) or straight to
    the hardware registers (counters, flags: the last write of the edge wins, as in VHDL)
  * pulses: V_PB / V_PC hold the values before the edge, N_PB / N_PC the new ones (bits 0..7 / 0..12 are the
    hardware pulses of cd_ctrl, the rest only exist here)
Generates rtl/cd_fw_pkg.vhd:  python3 rtl/cd_fw/cd_fw.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'tools'))
from cdasm import Asm, IO, IMM_MAX  # noqa: E402

a = Asm()

# ---------------------------------------------------------------- I/O (cd_ctrl.vhd)
(IN_EV, IN_FLAGS, IN_CMDIN, IN_CMDDLY, IN_WORKDLY, IN_DRIVEDLY, IN_LASTREAD, IN_TIA, IN_MSF, IN_PTR,
 IN_HDR, IN_SUBHDR, IN_SUB0, IN_SUB1, IN_SUB2, IN_PEAK) = [IO + i for i in range(16)]
(OUT_STROBE, OUT_PB, OUT_PC, OUT_FSET, OUT_FCLR, OUT_CMDDLY, OUT_WORKDLY, OUT_DRIVEDLY, OUT_DSTATE, OUT_ISTAT,
 OUT_MODE, OUT_XAF, OUT_CURLBA, OUT_PHYLBA, OUT_READLBA, OUT_WSP, OUT_RSP, OUT_PUSH) = [IO + i for i in range(18)]

# event / status bits (IN_EV)
(EV_RESET, EV_SOFT, EV_CE, EV_BEGIN, EV_HANDLE, EV_LONG, EV_WORKDONE, EV_GETID, EV_ACKREAD, EV_ACKPEND,
 EV_ACKPENDNEXT, EV_NEWCD, EV_DRIVEDONE, EV_CONT, EV_SUBDONE, EV_ACKSECTOR, EV_HOLD11, EV_SMRESET, EV_HASCD,
 EV_AUDIOCD, EV_PEMPTY, EV_REMPTY, EV_HDRDATA, EV_HDRDATASEC, EV_ISAUDIO, EV_DMAREAD, EV_SSWREN) = range(27)
# IN_FLAGS bits
FI_CMDPENDING, FI_CMDBUSY, FI_WORKING, FI_DRIVEBUSY, FI_HANDLE, FI_INITED = 8, 9, 10, 11, 17, 18
# OUT_FSET / OUT_FCLR bits
F_CMDPENDING, F_CMDBUSY, F_WORKING, F_DRIVEBUSY, F_MUTED, F_WORKGETID, F_LONGCMD, F_CONT, F_PHYWAIT, F_HANDLE, F_INITED, \
    F_REPORT = [1 << i for i in range(12)]

# drive states (tdrivestate'pos)
(D_IDLE, D_SEEKPHYSICAL, D_SEEKLOGICAL, D_SEEKIMPLICIT, D_READING, D_PLAYING, D_SPEEDCHANGE, D_SPINNINGUP,
 D_CHANGESESSION, D_OPENING) = range(10)

READSPEED1X = 44100 * 0x300 // 75
READSPEED2X = 44100 * 0x300 // 150

# ---------------------------------------------------------------- registers
regs = {}


def R(name):
    if name not in regs:
        regs[name] = len(regs)
        assert len(regs) <= 64, 'register file full'
    return regs[name]


T0, T1, T2, T3, T4, T5 = [R('T%d' % i) for i in range(6)]
DOUBLE = ['FL', 'PB', 'PC', 'CMD', 'STEPS', 'MISC', 'ERR', 'DST', 'IST', 'MODE', 'CUR', 'PHYLBA', 'DIFF', 'STMUL']
for n in DOUBLE:
    R('V_' + n)
    R('N_' + n)
SINGLE = ['SETLOC', 'XAF', 'LASTREP', 'ERRD', 'PLAYLBA', 'TRK', 'PHYP', 'PHYBASE', 'SUB0', 'SUB1', 'SUB2']
for n in SINGLE:
    R(n)


def V(n): return regs['V_' + n]
def N(n): return regs['N_' + n]


# flags in FL
FL_BITS = ['setLocActive', 'playTrack', 'readAfterSeek', 'playAfterSeek', 'lastSectorHeaderValid', 'seekOK',
           'afterSeek', 'limitDelayTime']
FLB = {n: i for i, n in enumerate(FL_BITS)}
# pulses
PB_BITS = ['cmdAck', 'cmdIRQ', 'driveAck', 'getIDAck', 'softReset', 'errorResponseNext_new', 'ClearXACurrentSet',
           'cdda_out8', 'seekOnDiskCmd', 'stop_afterseek', 'setMode', 'readSN', 'play', 'setSession', 'cmdStartMotor',
           'cmdStop', 'drive_stop', 'startMotorCMD', 'shell_close', 'errorResponseCmd_new', 'updatePhysicalPosition']
PC_BITS = ['readOnDisk', 'ackDrive', 'ackDriveEnd', 'seekOnDiskDrive', 'startReading', 'startPlaying', 'ackRead',
           'processDataSector', 'processCDDASector', 'processSeekHeader', 'clearSectorBuffers',
           'triggerUpdateSubchannel', 'handleDrive', 'seekOnDiskPlay', 'errorResponseDrive_new', 'calcSeekTime',
           'addSeekTime', 'startMotor']
PULSE = {}
for i, n in enumerate(PB_BITS): PULSE[n] = ('PB', i)
for i, n in enumerate(PC_BITS): PULSE[n] = ('PC', i)

# packed fields: (word, lo, width); word in DOUBLE -> V/N, in SINGLE -> itself
FIELDS = {
    'nextCmd': ('CMD', 0, 8), 'paramCount': ('CMD', 8, 3), 'workCommand': ('CMD', 16, 8),
    'setLocReadStep': ('STEPS', 0, 8), 'CDDA_outcnt': ('STEPS', 8, 8), 'setFilterReadStep': ('STEPS', 16, 8),
    'physicalUpdateState': ('STEPS', 24, 3),
    'session': ('MISC', 0, 8), 'fastForwardRate': ('MISC', 8, 8), 'newMode': ('MISC', 16, 8),
    'errCmd': ('ERR', 0, 8), 'reasonCmd': ('ERR', 8, 8), 'reasonNext': ('ERR', 16, 8),
    'setLocMinute': ('SETLOC', 0, 8), 'setLocSecond': ('SETLOC', 8, 8), 'setLocFrame': ('SETLOC', 16, 8),
    'errDrive': ('ERRD', 0, 8), 'reasonDrive': ('ERRD', 8, 8),
    'currentTrackBCD': ('TRK', 0, 8), 'nextTrack': ('TRK', 8, 8),
    'phy_spt': ('PHYP', 0, 5), 'phy_oldOffset': ('PHYP', 8, 5), 'phy_newOffset': ('PHYP', 16, 5),
}

# physical update states
(PU_IDLE, PU_START, PU_CHECK, PU_CALC1, PU_CALC2, PU_CALCDONE, PU_READSUB) = range(7)

# ---------------------------------------------------------------- helpers


def wreg(word, new):
    if word in DOUBLE:
        return N(word) if new else V(word)
    return regs[word]


def get(name, new=False):
    """A = field (old value unless new)"""
    word, lo, w = FIELDS[name]
    if lo % 8 == 0 and w <= 8:
        a.ldb(wreg(word, new), lo // 8)
        if w < 8:
            a.andi((1 << w) - 1)
    else:
        a.ldfield(wreg(word, new), lo, w)


def put(name):
    """field (new) = A; A must fit; A is shifted to the field"""
    word, lo, w = FIELDS[name]
    a.stfield(wreg(word, True), lo, w)


def puti(name, v):
    """field (new) = v; A and the flags unchanged"""
    word, lo, w = FIELDS[name]
    r = wreg(word, True)
    mask = ((1 << w) - 1) << lo
    assert mask <= IMM_MAX and 0 <= v < (1 << w)
    a.andr(r, IMM_MAX & ~mask)
    if v:
        a.orr(r, v << lo)


def flag_set(n, v=1):
    if v:
        a.setbit(N('FL'), FLB[n])
    else:
        a.clrbit(N('FL'), FLB[n])


def if_flag(n):
    return a.if_bs(V('FL'), FLB[n])


def if_not_flag(n):
    return a.if_bc(V('FL'), FLB[n])


def pulse(n):
    w, b = PULSE[n]
    a.setbit(N(w), b)


def if_pulse(n):
    w, b = PULSE[n]
    return a.if_bs(V(w), b)


def if_ev(b):
    return a.if_bs(IN_EV, b)


def if_not_ev(b):
    return a.if_bc(IN_EV, b)


# Drive countdown loads: set F_DRIVEBUSY before writing OUT_DRIVEDLY. In hardware the countdown goes on while the
# engine runs, but only while driveBusy is set: a load before the flag loses the clocks in between (one per
# sector when reading = drive 4 ppm slow; XA / CD-DA would underrun by a sample every few seconds).
def fset(mask):
    a.ldi(mask); a.st(OUT_FSET)


def fclr(mask):
    a.ldi(mask); a.st(OUT_FCLR)


# strobes of this edge, written straight to cd_ctrl (bits are or-ed; a response write replaces the byte, so the
# last write of the edge wins as in the original)
def resp_reset(): a.ldi(1); a.st(OUT_STROBE)
def param_rd(): a.ldi(4); a.st(OUT_STROBE)
def param_reset(): a.ldi(8); a.st(OUT_STROBE)
def xa_reset(): a.ldi(16); a.st(OUT_STROBE)


def push_a():
    """response fifo write of byte A (bits 7..0)"""
    a.st(OUT_PUSH)


def push_i(v):
    a.ldi(v); a.st(OUT_PUSH)


def ist_clr(mask):
    a.clrbits(N('IST'), mask)


def ist_set(mask):
    a.setbits(N('IST'), mask)


def dst(v):
    a.movi(N('DST'), v)


def if_dst(v):
    return a.if_eqi(V('DST'), v)


def if_not_dst(v):
    return a.if_nei(V('DST'), v)


def if_dst_seek():
    """old driveState in SEEKPHYSICAL, SEEKLOGICAL, SEEKIMPLICIT"""
    a.ld(V('DST')); a.subi(1); a.cmpi(3)
    return a.if_c()


def if_not_dst_seek():
    a.ld(V('DST')); a.subi(1); a.cmpi(3)
    return a.if_nc()


def ld_readspeed():
    """A = driveREADSPEED (old modeReg)"""
    with a.if_bs(V('MODE'), 7):
        a.ldi(READSPEED2X)
    with a.else_():
        a.ldi(READSPEED1X)


def wr_workdelay():
    """workDelay <= A, unless the hold at 11 applies at this edge (it is the last assignment in the original)"""
    with if_not_ev(EV_HOLD11):
        a.st(OUT_WORKDLY)


def ld_cmdin_byte(k):
    a.ldb(IN_CMDIN, k)


def ld_param(): ld_cmdin_byte(1)          # FifoParam_Dout


def ld_byte(r, k):
    a.ldb(r, k)


def ld_nextsub(k):
    ld_byte(IN_SUB0 + k // 4, k % 4)


def ld_seeklba():
    """A = seekLBA = minute * 4500 + second * 75 + frame (19 bit)"""
    get('setLocSecond'); a.muli(75); a.st(T3)
    get('setLocMinute'); a.muli(9); a.muli(500); a.add(T3); a.st(T3)
    get('setLocFrame'); a.add(T3); a.andi(0x7FFFF)


def if_nextcmd(v, new=False):
    get('nextCmd', new); a.st(T5)
    return a.if_eqi(T5, v)


def bcd2bin_param():
    """A = FifoParam_Dout(7..4) * 10 + FifoParam_Dout(3..0) (8 bit)"""
    ld_param(); a.shr(4); a.muli(10); a.st(T3)
    ld_param(); a.andi(0x0F); a.add(T3); a.andi(0xFF)


def err_cmd(err, reason):
    pulse('errorResponseCmd_new')
    a.ld(N('ERR')); a.andi(IMM_MAX & ~0xFFFF); a.ori(err | (reason << 8)); a.st(N('ERR'))


def err_drive(err, reason):
    pulse('errorResponseDrive_new')
    a.movi(regs['ERRD'], err | (reason << 8))


# ---------------------------------------------------------------- program
ENTRY = a.here('entry')

# power-on values of the original's registers that reset does not set
with a.if_bc(IN_FLAGS, FI_INITED):
    for r in list(regs.values()):
        a.movi(r, 0)
    a.movi(V('FL'), 1 << FLB['seekOK']); a.movi(N('FL'), 1 << FLB['seekOK'])
    a.movi(V('IST'), 0x10); a.movi(N('IST'), 0x10)
    a.movi(V('MODE'), 0x20); a.movi(N('MODE'), 0x20)
    a.movi(regs['PHYP'], 8)
    fset(F_INITED)

# new copies start as the old values
for n in DOUBLE:
    a.mov(N(n), V(n))

COMMIT = a.label('commit')

# ================================================================ reset (both processes)
with if_ev(EV_RESET):
    # P2 reset branch: fifo resets, cmdPending, cmd_busy, cmd_delay, working, workDelay, muted, XA filter: hardware
    puti('nextCmd', 0)
    puti('workCommand', 0)
    flag_set('setLocActive', 0)
    a.movi(regs['SETLOC'], 0)
    puti('session', 0)
    puti('fastForwardRate', 0)
    a.movi(regs['XAF'], 0)
    a.movi(regs['LASTREP'], 0x1F)
    puti('CDDA_outcnt', 0)
    # P3 reset branch
    dst(D_IDLE)
    a.movi(N('IST'), 0x10)
    a.movi(N('MODE'), 0x20)
    a.movi(N('CUR'), 0)
    a.movi(N('PHYLBA'), 0)
    flag_set('readAfterSeek', 0)
    flag_set('playAfterSeek', 0)
    flag_set('lastSectorHeaderValid', 0)
    puti('currentTrackBCD', 0)
    for k in ('SUB0', 'SUB1', 'SUB2'):
        a.movi(regs[k], 0)
    with if_ev(EV_SMRESET):
        pulse('startMotor')
    a.clrbits(N('PC'), (1 << PC_BITS.index('calcSeekTime')) | (1 << PC_BITS.index('addSeekTime')))
    a.j(COMMIT)

# ================================================================ pulse defaults
# P2 clears its pulses at ce edges; P3 at ce edges outside a soft reset
with if_ev(EV_CE):
    a.movi(N('PB'), 0)
    with if_not_ev(EV_SOFT):
        a.movi(N('PC'), 0)

# ================================================================ P2: command processing (ce)
P2_END = a.label('p2end')
a.jbc(IN_EV, EV_CE, P2_END)

# ---- receive a new command
with if_ev(EV_BEGIN):
    ld_cmdin_byte(0); a.st(T0)                                  # T0 = newCmd
    # paramCountNew -> T1
    a.movi(T1, 0)
    for code, n in ((0x02, 3), (0x0D, 2), (0x0E, 1), (0x12, 1), (0x14, 1), (0x19, 1), (0x1D, 2), (0x1F, 6)):
        a.ld(T0); a.cmpi(code)
        with a.if_z():
            a.movi(T1, n)
    # applyNewcommand -> T2
    a.movi(T2, 1)
    with a.if_bs(IN_FLAGS, FI_CMDPENDING):
        a.ld(T0); a.cmpi(0x0E)
        with a.if_z():
            with if_nextcmd(0x02):
                a.movi(T2, 0)
    with a.if_bs(T2, 0):
        with a.if_bs(IN_FLAGS, FI_CMDPENDING):
            get('paramCount'); a.st(T4)
            a.ld(T1); a.cmp(T4)                   # paramCountNew < paramCount
            with a.if_c():
                param_reset()
        fclr(F_WORKING)
        fset(F_CMDPENDING | F_CMDBUSY)
        with if_dst(D_OPENING):
            a.ldi(15000 - 2)
        with a.else_():
            a.ldi(25000 - 2)
        a.st(T3)
        a.ld(T0); a.cmpi(0x1C)
        with a.if_z():
            a.movi(T3, 120000 - 2)
        a.mov(OUT_CMDDLY, T3)
        a.ld(T0); put('nextCmd')
        a.ld(T1); put('paramCount')
        with if_dst(D_IDLE):
            with a.if_bs(V('IST'), 1):
                a.ld(T0); a.cmpi(0x11)
                with a.if_z():
                    pulse('updatePhysicalPosition')

# ---- command processing time is up: handle it
HC_END = a.label('hcend')
WORK = a.label('work')
a.jbc(IN_FLAGS, FI_HANDLE, WORK)
# fifoParamCount < paramCount
ld_byte(IN_PTR, 2); a.andi(0x1F); a.st(T0)
get('paramCount'); a.st(T1)
a.ld(T0); a.cmp(T1)
with a.if_c():
    err_cmd(0x01, 0x20)
    fclr(F_CMDPENDING)
    param_reset()
    a.j(HC_END)
# response fifo reset unless the command reads it
with if_not_ev(EV_REMPTY):
    get('nextCmd'); a.st(T0)
    skip = a.label()
    for code in (0x0F, 0x10, 0x11, 0x13, 0x14, 0x19):
        a.jeq(T0, code, skip)
    resp_reset()
    a.place(skip)
get('nextCmd'); a.st(T0)
a.ld(T0); a.cmpi(0x02)
with a.if_nz():
    a.ld(T0); a.cmpi(0x0D)
    with a.if_nz():
        param_reset()


def cmd_arm(*codes):
    """body for nextCmd in codes (T0 = nextCmd)"""
    return _Arm(codes)


class _Arm:
    def __init__(self, codes):
        self.codes = codes

    def __enter__(self):
        self.nxt = a.label()
        body = a.label()
        for c in self.codes:
            a.jeq(T0, c, body)
        a.j(self.nxt)
        a.place(body)

    def __exit__(self, *x):
        a.j(HC_END)
        a.place(self.nxt)


def no_cd_error():
    """if hasCD = '0': error 01 / 80, returns the else context"""


with cmd_arm(0x00):        # Sync
    err_cmd(0x01, 0x40)
    fclr(F_CMDPENDING)
with cmd_arm(0x01):        # Getstat
    pulse('cmdAck')
    fclr(F_CMDPENDING)
    with if_ev(EV_HASCD):
        with if_not_dst(D_OPENING):
            pulse('shell_close')
with cmd_arm(0x02):        # Setloc
    puti('setLocReadStep', 5)
    flag_set('setLocActive')
    pulse('cmdAck')
    fclr(F_CMDPENDING)
with cmd_arm(0x03):        # Play
    fclr(F_CMDPENDING)
    with if_not_ev(EV_HASCD):
        err_cmd(0x01, 0x80)
    with a.else_():
        flag_set('playTrack', 0)
        puti('nextTrack', 0)
        with if_not_ev(EV_PEMPTY):
            a.ld(IN_TIA); a.addi(150); a.andi(0x7FFFF); a.st(regs['PLAYLBA'])
            flag_set('playTrack')
            ld_param(); put('nextTrack')
        pulse('cmdAck')
        puti('fastForwardRate', 0)
        # (FifoParam_Empty or FifoParam_Dout = 0) and (setLocActive = 0 or seekLBA = lastReadSector) and
        # (driveState = PLAYING or (seeking and playAfterSeek))
        NOPARAM = T1
        a.movi(NOPARAM, 0)
        with if_ev(EV_PEMPTY):
            a.movi(NOPARAM, 1)
        ld_param(); a.cmpi(0)
        with a.if_z():
            a.movi(NOPARAM, 1)
        a.movi(T2, 0)        # T2 = whole condition
        with a.if_bs(NOPARAM, 0):
            a.movi(T0, 0)    # setLoc part
            with if_not_flag('setLocActive'):
                a.movi(T0, 1)
            ld_seeklba(); a.cmp(IN_LASTREAD)
            with a.if_z():
                a.movi(T0, 1)
            with a.if_bs(T0, 0):
                with if_dst(D_PLAYING):
                    a.movi(T2, 1)
                with if_dst_seek():
                    with if_flag('playAfterSeek'):
                        a.movi(T2, 1)
        with a.if_bs(T2, 0):
            a.mov(regs['PLAYLBA'], V('CUR'))
        with a.else_():
            pulse('play')
            a.movi(regs['LASTREP'], 0x1F)
            with a.if_bs(NOPARAM, 0):
                a.mov(regs['PLAYLBA'], V('CUR'))
with cmd_arm(0x04, 0x05):  # Forward / Backward
    with if_not_dst(D_PLAYING):
        err_cmd(0x01, 0x80)
    with a.else_():
        get('fastForwardRate'); a.st(T1)          # signed byte
        get('nextCmd'); a.cmpi(0x04)
        with a.if_z():     # forward: rate < 0 -> 4, rate < 12 -> rate + 4
            with a.if_bs(T1, 7):
                puti('fastForwardRate', 4)
            with a.else_():
                a.ld(T1); a.cmpi(12)
                with a.if_c():
                    a.ld(T1); a.addi(4); put('fastForwardRate')
        with a.else_():    # backward: rate > 0 -> -4, rate > -12 -> rate - 4
            a.ld(T1); a.cmpi(0)
            with a.if_nz():
                with a.if_bc(T1, 7):
                    puti('fastForwardRate', 0xFC)
                with a.else_():
                    a.ld(T1); a.cmpi(0xF5)     # rate > -12  <=>  byte >= 0xF5
                    with a.if_nc():
                        a.ld(T1); a.subi(4); a.andi(0xFF); put('fastForwardRate')
            with a.else_():  # rate = 0 > -12
                puti('fastForwardRate', 0xFC)
        pulse('cmdAck')
        fclr(F_CMDPENDING)
with cmd_arm(0x07):        # MotorOn
    fclr(F_CMDPENDING)
    with a.if_bs(V('IST'), 1):
        err_cmd(0x01, 0x20)
    with a.else_():
        pulse('cmdAck')
        with a.if_bc(IN_FLAGS, FI_WORKING):
            fset(F_WORKING)
            a.ldi(400000 - 2); wr_workdelay()
            get('nextCmd'); put('workCommand')
            with if_ev(EV_HASCD):
                pulse('cmdStartMotor')
with cmd_arm(0x08):        # Stop
    pulse('cmdAck')
    fclr(F_CMDPENDING)
    with a.if_bs(V('IST'), 1):
        with a.if_bs(V('MODE'), 7):
            a.ldi(25000000 - 2)
        with a.else_():
            a.ldi(13000000 - 2)
    with a.else_():
        a.ldi(7000 - 2)
    wr_workdelay()
    fset(F_WORKING)
    get('nextCmd'); put('workCommand')
    pulse('cmdStop')
with cmd_arm(0x09):        # Pause
    a.movi(T1, 0)
    with if_dst(D_SEEKLOGICAL):
        a.movi(T1, 1)
    with if_dst(D_SEEKPHYSICAL):
        a.movi(T1, 1)
    with a.if_bs(V('IST'), 6):
        with if_dst(D_READING):
            a.movi(T1, 1)
        with if_dst(D_PLAYING):
            a.movi(T1, 1)
    with a.if_bs(T1, 0):
        fclr(F_CMDPENDING)
        err_cmd(0x01, 0x80)
        with if_flag('readAfterSeek'):
            pulse('stop_afterseek')
    with a.else_():
        pulse('cmdAck')
        fclr(F_CMDPENDING)
        fset(F_WORKING)
        get('nextCmd'); put('workCommand')
        xa_reset()
        a.movi(T2, 7000 - 2)
        a.movi(T1, 0)
        with if_dst(D_READING):
            a.movi(T1, 1)
        with if_dst(D_PLAYING):
            a.movi(T1, 1)
        with a.if_bs(T1, 0):
            with a.if_bs(V('MODE'), 7):
                a.ldi(1066874)
            with a.else_():
                a.ldi(2157295)
            a.add(IN_DRIVEDLY); a.st(T2)
        a.ld(T2); a.andi(0x1FFFFFF); wr_workdelay()
        pulse('drive_stop')
with cmd_arm(0x0A):        # Init (reset)
    a.movi(T1, 0)
    with a.if_bs(IN_FLAGS, FI_WORKING):
        get('workCommand'); a.cmpi(0x0A)
        with a.if_z():
            a.movi(T1, 1)
    with a.if_bs(T1, 0):
        fclr(F_CMDPENDING)
    with a.else_():
        pulse('cmdAck')
        pulse('softReset')
        fset(F_WORKING)
        a.ld(IN_WORKDLY); a.addi(3999999); a.andi(0x1FFFFFF); wr_workdelay()
        get('nextCmd'); put('workCommand')
        a.ldi(24999 - 2); a.st(OUT_CMDDLY)
        fset(F_CMDBUSY)
with cmd_arm(0x0B):        # Mute
    fset(F_MUTED)
    pulse('cmdAck')
    fclr(F_CMDPENDING)
with cmd_arm(0x0C):        # Demute
    fclr(F_MUTED)
    pulse('cmdAck')
    fclr(F_CMDPENDING)
with cmd_arm(0x0D):        # Setfilter
    puti('setFilterReadStep', 3)
    pulse('ClearXACurrentSet')
    pulse('cmdAck')
    fclr(F_CMDPENDING)
with cmd_arm(0x0E):        # Setmode
    pulse('setMode')
    ld_param(); put('newMode')
    pulse('cmdAck')
    fclr(F_CMDPENDING)
with cmd_arm(0x0F):        # GetParam
    fclr(F_CMDPENDING)
    pulse('cmdIRQ')
with cmd_arm(0x10):        # GetLocL
    fclr(F_CMDPENDING)
    with if_not_flag('lastSectorHeaderValid'):
        err_cmd(0x01, 0x80)
    with a.else_():
        pulse('cmdIRQ')
with cmd_arm(0x11, 0x13):  # GetLocP, GetTN
    fclr(F_CMDPENDING)
    with if_not_ev(EV_HASCD):
        err_cmd(0x01, 0x80)
    with a.else_():
        pulse('cmdIRQ')
with cmd_arm(0x12):        # SetSession
    fclr(F_CMDPENDING)
    a.movi(T1, 0)
    with if_not_ev(EV_HASCD):
        a.movi(T1, 1)
    with if_dst(D_READING):
        a.movi(T1, 1)
    with if_dst(D_PLAYING):
        a.movi(T1, 1)
    with a.if_bs(T1, 0):
        err_cmd(0x01, 0x80)
    with a.else_():
        ld_param(); put('session')
        pulse('setSession')
        pulse('cmdAck')
with cmd_arm(0x14):        # GetTD
    fclr(F_CMDPENDING)
    with if_not_ev(EV_HASCD):
        err_cmd(0x01, 0x80)
    with a.else_():
        ld_byte(IN_PTR, 1); a.st(T1)     # trackcountBCD
        ld_param(); a.st(T2)
        a.ld(T1); a.cmp(T2)               # trackcount < param  <=>  param > trackcount
        with a.if_c():
            err_cmd(0x01, 0x10)
        with a.else_():
            pulse('cmdIRQ')
with cmd_arm(0x15, 0x16):  # SeekL / SeekP
    pulse('cmdAck')
    fclr(F_CMDPENDING)
    pulse('seekOnDiskCmd')
    flag_set('setLocActive', 0)
    xa_reset()
with cmd_arm(0x17, 0x18):  # SetClock / GetClock
    fclr(F_CMDPENDING)
    err_cmd(0x01, 0x40)
with cmd_arm(0x19):        # Test
    fclr(F_CMDPENDING)
    ld_param(); a.st(T1)
    ok = a.label()
    for v in (0x04, 0x05, 0x20, 0x22):
        a.jeq(T1, v, ok)
    err_cmd(0x01, 0x40)
    a.j(HC_END)
    a.place(ok)
    pulse('cmdIRQ')
with cmd_arm(0x1A):        # GetID
    fclr(F_CMDPENDING)
    with if_not_ev(EV_HASCD):
        err_cmd(0x01, 0x80)
    with a.else_():
        pulse('cmdAck')
        fset(F_WORKING)
        a.ldi(33867 - 2); wr_workdelay()
        get('nextCmd'); put('workCommand')
with cmd_arm(0x06, 0x1B):  # ReadN / ReadS
    fclr(F_CMDPENDING)
    with if_not_ev(EV_HASCD):
        err_cmd(0x01, 0x80)
    with a.else_():
        a.movi(T1, 0)
        with if_ev(EV_AUDIOCD):
            with a.if_bc(V('MODE'), 0):
                a.movi(T1, 1)
        with a.if_bs(T1, 0):
            err_cmd(0x01, 0x40)
        with a.else_():
            pulse('cmdAck')
            a.movi(T0, 0)    # (setLocActive = 0 or seekLBA = lastReadSector)
            with if_not_flag('setLocActive'):
                a.movi(T0, 1)
            ld_seeklba(); a.cmp(IN_LASTREAD)
            with a.if_z():
                a.movi(T0, 1)
            a.movi(T2, 0)
            with a.if_bs(T0, 0):
                with if_dst(D_READING):
                    a.movi(T2, 1)
                with if_dst_seek():
                    with if_flag('readAfterSeek'):
                        a.movi(T2, 1)
            with a.if_bs(T2, 0):
                flag_set('setLocActive', 0)
            with a.else_():
                pulse('readSN')
with cmd_arm(0x1C, 0x1D, 0x1F):   # Init / GetQ / VideoCD
    fclr(F_CMDPENDING)
    err_cmd(0x01, 0x40)
with cmd_arm(0x1E):        # ReadTOC
    fclr(F_CMDPENDING)
    with if_not_ev(EV_HASCD):
        err_cmd(0x01, 0x80)
    with a.else_():
        pulse('cmdAck')
        fset(F_WORKING)
        a.ldi(16934400 - 2); wr_workdelay()
        get('nextCmd'); put('workCommand')
# others
err_cmd(0x01, 0x40)
fclr(F_CMDPENDING)
a.j(HC_END)

# ---- second processing of commands that work: elsif working
a.place(WORK)
with a.if_bs(IN_FLAGS, FI_WORKING):
    a.ld(IN_WORKDLY); a.cmpi(0)
    with a.if_nz():
        get('workCommand'); a.cmpi(0x1A)
        with a.if_z():     # GetID response bytes
            a.ld(IN_WORKDLY); a.st(T1)
            a.ld(T1); a.cmpi(10)
            with a.if_z():
                resp_reset()
            a.ld(T1); a.cmpi(9)
            with a.if_z():
                a.ld(V('IST')); a.st(T2)
                with if_not_ev(EV_HASCD):
                    a.ld(T2); a.ori(0x08); a.st(T2)
                with if_ev(EV_HASCD):
                    a.ld(T2); a.ori(0x02); a.st(T2)
                with if_ev(EV_AUDIOCD):
                    a.ld(T2); a.ori(0x08); a.st(T2)
                a.ld(T2); push_a()
            a.ld(T1); a.cmpi(8)
            with a.if_z():
                a.movi(T2, 0)
                with if_not_ev(EV_HASCD):
                    a.ld(T2); a.ori(0x40); a.st(T2)
                with if_ev(EV_AUDIOCD):
                    a.ld(T2); a.ori(0x90); a.st(T2)
                a.ld(T2); push_a()
            for v, b in ((7, 0x20), (6, 0x00), (5, ord('S')), (4, ord('C')), (3, ord('E'))):
                a.ld(T1); a.cmpi(v)
                with a.if_z():
                    push_i(b)
            a.ld(T1); a.cmpi(2)
            with a.if_z():     # region: 00 'A', 01 'I', 10 'E'
                a.ld(IN_CMDIN); a.shr(30); a.st(T2)
                for rv, b in ((0, ord('A')), (1, ord('I')), (2, ord('E'))):
                    a.ld(T2); a.cmpi(rv)
                    with a.if_z():
                        push_i(b)
    with a.else_():
        fclr(F_WORKING)
        get('workCommand'); a.cmpi(0x1A)
        with a.if_z():
            with if_ev(EV_HASCD):
                pulse('startMotorCMD')
            pulse('getIDAck')
        with a.else_():
            pulse('driveAck')
a.place(HC_END)

# ---- seek clears setLocActive
a.ld(V('PB')); a.andi(1 << PULSE['seekOnDiskCmd'][1]); a.st(T0)
a.ld(V('PC')); a.andi(1 << PULSE['seekOnDiskDrive'][1]); a.or_(T0)
with a.if_nz():
    flag_set('setLocActive', 0)

# ---- Setloc: parameters (one every other clock)
get('setLocReadStep'); a.st(T1)
a.cmpi(0)
with a.if_nz():
    a.ld(T1); a.subi(1); put('setLocReadStep')
    a.ld(T1); a.cmpi(5)
    with a.if_z():
        bcd2bin_param(); put('setLocMinute'); param_rd()
    a.ld(T1); a.cmpi(3)
    with a.if_z():
        bcd2bin_param(); put('setLocSecond'); param_rd()
    a.ld(T1); a.cmpi(1)
    with a.if_z():
        bcd2bin_param(); put('setLocFrame'); param_reset()

# ---- Setfilter: parameters
get('setFilterReadStep'); a.st(T1)
a.cmpi(0)
with a.if_nz():
    a.ld(T1); a.subi(1); put('setFilterReadStep')
    a.ld(T1); a.cmpi(3)
    with a.if_z():
        ld_param(); a.st(T2); a.ld(regs['XAF']); a.andi(0xFF00); a.or_(T2); a.st(regs['XAF']); param_rd()
    a.ld(T1); a.cmpi(1)
    with a.if_z():
        ld_param(); a.shl8(); a.st(T2); a.ld(regs['XAF']); a.andi(0x00FF); a.or_(T2); a.st(regs['XAF']); param_reset()

# ---- responses
a.ld(V('PB')); a.andi(1 << PULSE['cmdAck'][1]); a.st(T0)
with if_ev(EV_ACKREAD):
    a.movi(T0, 1)
a.ld(T0)
with a.if_nz():
    a.ld(V('IST')); push_a()
a.ld(V('PB')); a.andi(1 << PULSE['driveAck'][1]); a.st(T0)
a.ld(V('PC')); a.andi((1 << PULSE['ackDrive'][1]) | (1 << PULSE['ackDriveEnd'][1])); a.or_(T0)
with a.if_nz():
    a.ld(IN_FLAGS); a.andi(0x1F)
    with a.if_z():
        with if_pulse('ackDriveEnd'):
            a.ld(IN_FLAGS); a.shr(20); a.andi(0xFF)   # internalStatus_1
        with a.else_():
            a.ld(V('IST'))
        push_a()
with if_pulse('errorResponseCmd_new'):
    get('errCmd'); a.or_(V('IST')); push_a()
    pulse('errorResponseNext_new')
    get('reasonCmd'); put('reasonNext')
with if_pulse('errorResponseDrive_new'):
    a.ld(regs['ERRD']); a.andi(0xFF); a.or_(V('IST')); push_a()
    pulse('errorResponseNext_new')
    a.ld(regs['ERRD']); a.shr8(); a.andi(0xFF); put('reasonNext')
with if_pulse('errorResponseNext_new'):
    get('reasonNext'); push_a()
with if_ev(EV_ACKPEND):
    resp_reset()
with if_ev(EV_ACKPENDNEXT):
    ld_cmdin_byte(2); push_a()

# ---- long responses (cmd_delay 11 .. 1 before the command is handled)
with if_ev(EV_LONG):
    a.ld(IN_CMDDLY); a.st(T1)      # cmd_delay(3 downto 0): enough for 11 .. 1
    get('nextCmd'); a.st(T0)

    def at(k):
        return a.if_eqi(T1, k)

    def reset_at(k):
        with at(k):
            with if_not_ev(EV_REMPTY):
                resp_reset()

    LR_END = a.label()

    def lr(code):
        return _LR(code)

    class _LR:
        def __init__(self, code): self.code = code
        def __enter__(self):
            self.nxt = a.label()
            a.jne(T0, self.code, self.nxt)
        def __exit__(self, *x):
            a.j(LR_END)
            a.place(self.nxt)

    with lr(0x0F):         # GetParam
        reset_at(6)
        with at(5): a.ld(V('IST')); push_a()
        with at(4): a.ld(V('MODE')); push_a()
        with at(3): push_i(0)
        with at(2): a.ld(regs['XAF']); push_a()
        with at(1): ld_byte(regs['XAF'], 1); push_a()
    with lr(0x10):         # GetLocL
        reset_at(9)
        with if_flag('lastSectorHeaderValid'):
            for k in range(4):
                with at(8 - k): ld_byte(IN_HDR, k); push_a()
            for k in range(4):
                with at(4 - k): ld_byte(IN_SUBHDR, k); push_a()
    with lr(0x11):         # GetLocP
        reset_at(9)
        with if_ev(EV_HASCD):
            for k, sb in ((8, 1), (7, 2), (6, 3), (5, 4), (4, 5), (3, 7), (2, 8), (1, 9)):
                with at(k): ld_byte(regs['SUB%d' % (sb // 4)], sb % 4); push_a()
    with lr(0x13):         # GetTN
        reset_at(4)
        with if_ev(EV_HASCD):
            with at(3): a.ld(V('IST')); push_a()
            with at(2): push_i(1)
            with at(1): ld_byte(IN_PTR, 1); push_a()
    with lr(0x14):         # GetTD
        reset_at(4)
        with if_ev(EV_HASCD):
            ld_param(); a.st(T2)
            a.ld(T2); a.cmpi(0)
            with a.if_z():
                with at(3): a.ld(V('IST')); push_a()
                with at(2): ld_byte(IN_MSF, 3); push_a()
                with at(1): ld_byte(IN_MSF, 2); push_a()
            with a.else_():
                ld_byte(IN_PTR, 1); a.st(T3)
                a.ld(T3); a.cmp(T2)          # trackcount < param -> nothing
                with a.if_nc():
                    with at(3): a.ld(V('IST')); push_a()
                    with at(2): ld_byte(IN_MSF, 1); push_a()
                    with at(1): ld_byte(IN_MSF, 0); push_a()
    with lr(0x19):         # Test
        reset_at(11)
        ld_param(); a.st(T2)
        with a.if_eqi(T2, 0x04):
            with at(1):
                a.ld(V('IST')); push_a()
                pulse('startMotorCMD')
        with a.if_eqi(T2, 0x05):
            with at(3): a.ld(V('IST')); push_a()
            with at(2): push_i(0)
            with at(1): push_i(0)
        with a.if_eqi(T2, 0x20):
            for k, b in ((4, 0x95), (3, 0x05), (2, 0x16), (1, 0xC1)):
                with at(k): push_i(b)
        with a.if_eqi(T2, 0x22):
            a.ld(IN_CMDIN); a.shr(30); a.st(T3)
            for rv, txt in ((0, 'for U/C'), (1, 'for Japan'), (2, 'for Europe')):
                with a.if_eqi(T3, rv):
                    for i, ch in enumerate(txt):
                        with at(len(txt) - i): push_i(ord(ch))
    a.place(LR_END)

# ---- CD-DA reports
with if_pulse('processCDDASector'):
    with if_dst(D_PLAYING):
        with a.if_bs(V('MODE'), 2):
            ld_nextsub(9); a.shr(4); a.st(T1)       # '0' & nextSubdata(9)(7 downto 4)
            a.ld(regs['LASTREP']); a.cmp(T1)
            with a.if_nz():
                a.mov(regs['LASTREP'], T1)
                get('CDDA_outcnt')
                with a.if_z():
                    a.ld(IN_FLAGS); a.andi(0x1F)
                    with a.if_z():
                        with if_ev(EV_REMPTY):
                            with a.if_bc(IN_FLAGS, FI_HANDLE):
                                with a.if_bc(IN_FLAGS, FI_CMDBUSY):
                                    puti('CDDA_outcnt', 8)
                                    pulse('cdda_out8')
                                    resp_reset()
get('CDDA_outcnt'); a.st(T1)
a.cmpi(0)
with a.if_nz():
    a.ld(T1); a.subi(1); put('CDDA_outcnt')

    def at(k):
        return a.if_eqi(T1, k)
    with at(8): a.ld(V('IST')); push_a()
    with at(7): ld_nextsub(1); push_a()
    with at(6): ld_nextsub(2); push_a()
    ld_nextsub(9); a.st(T2)
    with a.if_bs(T2, 4):
        with at(5): ld_nextsub(3); push_a()
        with at(4): ld_nextsub(4); a.ori(0x80); push_a()
        with at(3): ld_nextsub(5); push_a()
    with a.else_():
        with at(5): ld_nextsub(7); push_a()
        with at(4): ld_nextsub(8); push_a()
        with at(3): ld_nextsub(9); push_a()
    ld_nextsub(8); a.st(T2)
    with a.if_bs(T2, 0):
        with at(2): ld_byte(IN_PEAK, 2); push_a()
        with at(1): ld_byte(IN_PEAK, 3); a.andi(0x7F); a.ori(0x80); push_a()
    with a.else_():
        with at(2): ld_byte(IN_PEAK, 0); push_a()
        with at(1): ld_byte(IN_PEAK, 1); a.andi(0x7F); push_a()

# ---- new disc
with if_ev(EV_NEWCD):
    resp_reset()
    fclr(F_CMDPENDING | F_CMDBUSY)
    a.ldi(0); a.st(OUT_CMDDLY)

with if_pulse('softReset'):
    param_reset()

a.place(P2_END)

# ================================================================ P3: drive
P3_END = a.label('p3end')
with if_pulse('softReset'):
    a.movi(N('MODE'), 0x20)
    a.movi(N('IST'), 0x00)
    with if_ev(EV_HASCD):
        ist_set(0x02)
        a.ld(V('CUR')); a.cmpi(0)
        with a.if_nz():
            dst(D_SEEKIMPLICIT)
            pulse('readOnDisk')
            a.ldi(0); a.st(OUT_READLBA)
        with a.else_():
            dst(D_SPEEDCHANGE)
        fset(F_DRIVEBUSY)
        with a.if_bs(V('MODE'), 7):
            a.ldi(16934400 + 33868800 + 19999)
        with a.else_():
            a.ldi(16934400 + 19999)
        a.st(OUT_DRIVEDLY)
        pulse('calcSeekTime')
        flag_set('limitDelayTime', 0)
        a.mov(N('DIFF'), V('CUR'))
    with a.else_():
        fclr(F_DRIVEBUSY)
        dst(D_IDLE)
        a.ldi(0); a.st(OUT_DRIVEDLY)
    a.j(P3_END)

a.jbc(IN_EV, EV_CE, P3_END)

# ---- drive countdown done
with if_ev(EV_DRIVEDONE):
    pulse('handleDrive')
    fclr(F_DRIVEBUSY)
    with if_dst_seek():
        flag_set('seekOK')
        for k in range(3):
            a.mov(regs['SUB%d' % k], IN_SUB0 + k)
        with a.if_bs(IN_SUB0, 6):       # nextSubdata(0)(6): data
            with if_dst(D_SEEKLOGICAL):
                pulse('processSeekHeader')
                flag_set('lastSectorHeaderValid')
        with a.else_():
            with if_dst(D_SEEKLOGICAL):
                with a.if_bs(V('MODE'), 0):
                    flag_set('seekOK')
                with a.else_():
                    flag_set('seekOK', 0)
        ld_nextsub(1); a.cmpi(0xAA)
        with a.if_z():
            flag_set('seekOK', 0)
        a.mov(N('CUR'), IN_LASTREAD)
        a.mov(N('PHYLBA'), IN_LASTREAD)

# ---- the drive's work
with if_pulse('handleDrive'):
    a.ld(V('DST')); a.st(T0)
    with if_dst(D_SEEKLOGICAL):
        a.movi(T0, D_SEEKPHYSICAL)
    with a.if_eqi(T0, D_SEEKPHYSICAL):          # SEEKLOGICAL | SEEKPHYSICAL
        dst(D_IDLE)
        ist_clr(0xE0)
        with if_flag('seekOK'):
            with if_flag('readAfterSeek'):
                pulse('startReading')
                flag_set('afterSeek')
                flag_set('readAfterSeek', 0)
                ist_set(0x40)
            with a.else_():
                with if_flag('playAfterSeek'):
                    pulse('startPlaying')
                    flag_set('afterSeek')
                    flag_set('playAfterSeek', 0)
                with a.else_():
                    pulse('ackDrive')
        with a.else_():
            flag_set('lastSectorHeaderValid', 0)
            err_drive(0x04, 0x04)
    a.ld(V('DST')); a.st(T0)
    with if_dst(D_PLAYING):
        a.movi(T0, D_READING)
    with a.if_eqi(T0, D_READING):                # READING | PLAYING
        ld_nextsub(1); a.cmpi(0xAA)
        with a.if_z():
            ist_clr(0xE0)
            dst(D_IDLE)
            pulse('ackDriveEnd')
        with a.else_():
            a.movi(T1, 0)                        # skipreading
            with if_ev(EV_ISAUDIO):
                get('currentTrackBCD')
                with a.if_z():
                    ld_nextsub(1); put('currentTrackBCD')
                with a.else_():
                    ld_nextsub(1); a.st(T2)
                    get('currentTrackBCD'); a.cmp(T2)
                    with a.if_nz():
                        with a.if_bs(V('MODE'), 1):
                            a.movi(T1, 1)
                            pulse('ackDriveEnd')
                            ist_clr(0xE0)
                            dst(D_IDLE)
            with a.if_bc(T1, 0):
                a.ld(IN_LASTREAD); a.addi(1); a.andi(0x7FFFF); a.st(OUT_READLBA)
                a.movi(T2, 0)
                with if_not_ev(EV_ISAUDIO):
                    with if_dst(D_READING):
                        a.movi(T2, 1)
                with a.if_bs(T2, 0):
                    pulse('processDataSector')
                    flag_set('lastSectorHeaderValid')
                    ist_set(0x20)
                    ist_clr(0x40)
                    a.movi(T3, 0)    # (modeReg(6) = 0 or headerIsData) and (modeReg(5) = 1 or headerDataSector)
                    with a.if_bc(V('MODE'), 6):
                        a.movi(T3, 1)
                    with if_ev(EV_HDRDATA):
                        a.movi(T3, 1)
                    a.movi(T4, 0)
                    with a.if_bs(V('MODE'), 5):
                        a.movi(T4, 1)
                    with if_ev(EV_HDRDATASEC):
                        a.movi(T4, 1)
                    a.ld(T3); a.and_(T4)
                    with a.if_nz():
                        a.ld(IN_PTR); a.addi(1); a.andi(7); a.st(OUT_WSP)
                        pulse('ackRead')
                with a.else_():
                    with if_ev(EV_ISAUDIO):
                        a.movi(T2, 0)
                        with if_dst(D_PLAYING):
                            a.movi(T2, 1)
                        with if_dst(D_READING):
                            with a.if_bs(V('MODE'), 0):
                                a.movi(T2, 1)
                        with a.if_bs(T2, 0):
                            pulse('processCDDASector')
                            get('fastForwardRate'); a.st(T3)
                            a.cmpi(0)
                            with a.if_nz():
                                with a.if_bs(T3, 7):     # negative
                                    a.ld(T3); a.xori(0xFF); a.addi(1); a.st(T3)
                                    a.ld(IN_LASTREAD); a.sub(T3)
                                with a.else_():
                                    a.ld(IN_LASTREAD); a.add(T3)
                                a.andi(0x7FFFF); a.st(OUT_READLBA)
                fset(F_DRIVEBUSY)                   # before the delay (see fset)
                ld_readspeed(); a.subi(2); a.st(OUT_DRIVEDLY)
                a.mov(N('CUR'), IN_LASTREAD)
                a.mov(N('PHYLBA'), IN_LASTREAD)
                for k in range(3):
                    a.mov(regs['SUB%d' % k], IN_SUB0 + k)
                pulse('readOnDisk')
    with if_dst(D_SPEEDCHANGE):
        dst(D_IDLE)
    with if_dst(D_SPINNINGUP):
        dst(D_IDLE)
        ist_clr(0xE0)
        with if_ev(EV_HASCD):
            ist_set(0x02)
    with if_dst(D_CHANGESESSION):
        dst(D_IDLE)
        ist_clr(0xE0)
        with if_ev(EV_HASCD):
            ist_set(0x02)
        get('session'); a.cmpi(1)
        with a.if_z():
            pulse('ackDrive')
    with if_dst(D_OPENING):
        pulse('startMotor')

# ---- seek
a.ld(V('PB')); a.andi(1 << PULSE['seekOnDiskCmd'][1]); a.st(T0)
a.ld(V('PC')); a.andi((1 << PULSE['seekOnDiskDrive'][1]) | (1 << PULSE['seekOnDiskPlay'][1])); a.or_(T0)
with a.if_nz():
    with if_pulse('seekOnDiskPlay'):
        a.ld(regs['PLAYLBA'])
    with a.else_():
        ld_seeklba()
    a.st(T1)                                       # target LBA
    a.st(OUT_READLBA)
    a.ld(T1); a.cmp(V('CUR'))
    with a.if_c():                                 # target < current
        a.ld(V('CUR')); a.sub(T1)
    with a.else_():
        a.ld(T1); a.sub(V('CUR'))
    a.st(N('DIFF'))
    pulse('readOnDisk')
    with if_pulse('seekOnDiskCmd'):
        flag_set('readAfterSeek', 0)
        flag_set('playAfterSeek', 0)
    flag_set('lastSectorHeaderValid', 0)
    ist_clr(0xE0)
    ist_set(0x42)
    fset(F_DRIVEBUSY)
    ld_readspeed(); a.subi(2); a.st(T2)
    a.mov(OUT_DRIVEDLY, T2)
    flag_set('limitDelayTime', 0)
    pulse('calcSeekTime')
    with if_dst_seek():
        a.ld(T2); a.add(IN_DRIVEDLY); a.st(OUT_DRIVEDLY)
        flag_set('limitDelayTime')
    with a.else_():
        with if_dst(D_SPEEDCHANGE):
            with a.if_bc(V('PC'), PULSE['seekOnDiskPlay'][1]):
                with if_not_flag('playAfterSeek'):
                    a.ld(T2); a.add(IN_DRIVEDLY); a.st(OUT_DRIVEDLY)
    with if_nextcmd(0x15):
        dst(D_SEEKLOGICAL)
    with a.else_():
        dst(D_SEEKPHYSICAL)

with if_pulse('stop_afterseek'):
    flag_set('readAfterSeek', 0)
    flag_set('playAfterSeek', 0)

# ---- seek time
with if_pulse('calcSeekTime'):
    pulse('addSeekTime')
    a.ld(V('DIFF')); a.st(T1)
    a.cmpi(3)
    with a.if_c():
        a.ldi(3)
    with a.else_():
        a.ld(T1); a.cmpi(5)
        with a.if_c():
            a.ld(T1)
        with a.else_():
            a.ld(T1); a.cmpi(32)
            with a.if_c():
                a.ldi(5)
            with a.else_():
                a.ld(T1); a.cmpi(75)
                with a.if_c():
                    a.ld(T1); a.shr(3); a.addi(5)
                with a.else_():
                    a.ld(T1); a.cmpi(4500)
                    with a.if_c():
                        a.ld(T1); a.shr(8); a.addi(14)
                    with a.else_():
                        a.ld(T1); a.cmpi(250000)
                        with a.if_c():
                            a.ld(T1); a.shr(13); a.addi(31)
                        with a.else_():
                            a.ld(V('CUR')); a.cmpi(0)
                            with a.if_z():
                                a.ldi(137)
                            with a.else_():
                                a.ld(T1); a.shr(13); a.addi(31)
    a.st(N('STMUL'))

with if_pulse('addSeekTime'):
    ld_readspeed(); a.mul(V('STMUL')); a.add(IN_DRIVEDLY); a.st(T1)
    a.movi(T2, 0)
    with if_flag('limitDelayTime'):
        a.ldi(22880000); a.cmp(T1)                  # 22880000 < sum
        with a.if_c():
            a.movi(T2, 1)
    with a.if_bs(T2, 0):
        a.ldi(22880000)
    with a.else_():
        a.ld(T1)
    a.st(OUT_DRIVEDLY)

# ---- read / play requests from the command side
with if_pulse('readSN'):
    with if_flag('setLocActive'):
        pulse('seekOnDiskDrive')
        flag_set('readAfterSeek')
        flag_set('playAfterSeek', 0)
    with a.else_():
        ist_clr(0xE0)
        pulse('startReading')
        flag_set('afterSeek', 0)

with if_pulse('play'):
    get('nextTrack'); put('currentTrackBCD')
    with if_flag('playTrack'):
        pulse('seekOnDiskPlay')
        flag_set('readAfterSeek', 0)
        flag_set('playAfterSeek')
    with a.else_():
        with if_flag('setLocActive'):
            pulse('seekOnDiskDrive')
            flag_set('readAfterSeek', 0)
            flag_set('playAfterSeek')
        with a.else_():
            pulse('startPlaying')
            flag_set('afterSeek', 0)

with if_pulse('setSession'):
    dst(D_CHANGESESSION)
    fset(F_DRIVEBUSY)
    a.ldi(33868800 // 2); a.st(OUT_DRIVEDLY)


def rsp_zero():
    """readSectorPointer <= 0, unless the increment later in the process happens at this edge"""
    with if_not_ev(EV_ACKREAD):
        with if_not_ev(EV_ACKSECTOR):
            a.ldi(0); a.st(OUT_RSP)


with if_pulse('startReading'):
    pulse('clearSectorBuffers')
    with if_not_flag('afterSeek'):
        ist_set(0x40)
    with if_dst_seek():
        flag_set('readAfterSeek')
        flag_set('playAfterSeek', 0)
    with a.else_():
        ist_set(0x02)
        fset(F_DRIVEBUSY)
        ld_readspeed(); a.subi(2); a.st(T2); a.st(OUT_DRIVEDLY)
        dst(D_READING)
        a.ldi(0); a.st(OUT_WSP)
        rsp_zero()
        pulse('readOnDisk')
        a.mov(OUT_READLBA, V('CUR'))
        a.movi(N('STMUL'), 1)
        with if_not_flag('afterSeek'):
            with if_dst(D_SPEEDCHANGE):
                a.ld(T2); a.add(IN_DRIVEDLY); a.st(OUT_DRIVEDLY)
            pulse('addSeekTime')
            flag_set('limitDelayTime', 0)

with if_pulse('startPlaying'):
    pulse('clearSectorBuffers')
    with if_dst_seek():
        flag_set('readAfterSeek', 0)
        flag_set('playAfterSeek')
    with a.else_():
        ist_clr(0xE0)
        ist_set(0x82)
        fset(F_DRIVEBUSY)
        ld_readspeed(); a.subi(2); a.st(OUT_DRIVEDLY)
        dst(D_PLAYING)
        a.ldi(0); a.st(OUT_WSP)
        rsp_zero()
        pulse('readOnDisk')
        a.mov(OUT_READLBA, V('CUR'))
        a.movi(N('STMUL'), 1)
        with if_not_flag('afterSeek'):
            pulse('addSeekTime')
            flag_set('limitDelayTime', 0)

with if_pulse('cmdStop'):
    flag_set('lastSectorHeaderValid', 0)
    dst(D_IDLE)
    fclr(F_DRIVEBUSY)
    ist_clr(0xE2)
    a.movi(N('CUR'), 0)

with if_pulse('drive_stop'):
    dst(D_IDLE)
    fclr(F_DRIVEBUSY)
    ist_clr(0xE0)

a.ld(V('PC')); a.andi(1 << PULSE['startMotor'][1]); a.st(T0)
a.ld(V('PB')); a.andi(1 << PULSE['cmdStartMotor'][1]); a.or_(T0)
with a.if_nz():
    with if_not_dst(D_SPINNINGUP):
        dst(D_SPINNINGUP)
        fset(F_DRIVEBUSY)
        a.ldi(44100 * 300); a.st(OUT_DRIVEDLY)

with if_pulse('shell_close'):      # (LIDopen is '0' on the Pocket)
    ist_clr(0x10)

with if_pulse('startMotorCMD'):
    ist_set(0x02)

with if_pulse('setMode'):
    get('newMode'); a.st(N('MODE')); a.st(T1)
    a.ld(V('MODE')); a.xor(T1); a.andi(0x80)
    with a.if_nz():                                # speed change
        with if_not_dst(D_SPEEDCHANGE):
            with if_not_dst(D_SEEKIMPLICIT):
                with if_dst(D_IDLE):
                    dst(D_SPEEDCHANGE)
                    fset(F_DRIVEBUSY)
                with a.if_bs(T1, 7):
                    a.ldi(27095040)
                with a.else_():
                    a.ldi(33868800)
                a.st(OUT_DRIVEDLY)

# ---- physical position update (GetLocP while the motor runs)
get('physicalUpdateState'); a.st(T0)
with a.if_eqi(T0, PU_IDLE):
    with if_pulse('updatePhysicalPosition'):
        puti('physicalUpdateState', PU_START)
with a.if_eqi(T0, PU_START):
    puti('physicalUpdateState', PU_CHECK)
    # sectors per track of currentLBA (by minute)
    a.ld(V('CUR')); a.st(T1)
    a.movi(T2, 22)
    for lim, spt in ((75, 21), (68, 20), (61, 19), (53, 18), (45, 17), (40, 16), (33, 15), (28, 14), (24, 13),
                     (17, 12), (12, 11), (8, 10), (5, 9), (1, 8)):
        a.ld(T1); a.cmpi(lim * 4500)
        with a.if_c():
            a.movi(T2, spt)
    a.ld(T2); put('phy_spt')
    a.ld(T1); a.cmp(T2)
    with a.if_c():
        a.mov(regs['PHYBASE'], T1)
    with a.else_():
        a.ld(T2); a.subi(1); a.st(T3); a.ld(T1); a.sub(T3); a.st(regs['PHYBASE'])
with a.if_eqi(T0, PU_CHECK):
    puti('physicalUpdateState', PU_CALC1)
    a.ld(V('PHYLBA')); a.cmp(regs['PHYBASE'])
    with a.if_c():
        a.mov(N('PHYLBA'), regs['PHYBASE'])
with a.if_eqi(T0, PU_CALC1):
    puti('physicalUpdateState', PU_CALC2)
    a.ld(V('PHYLBA')); a.sub(regs['PHYBASE']); a.andi(0x1F); put('phy_oldOffset')
with a.if_eqi(T0, PU_CALC2):
    puti('physicalUpdateState', PU_CALCDONE)
    get('phy_spt'); a.st(T2)
    get('phy_oldOffset'); a.addi(1); a.st(T1)
    for _ in range(4):
        a.ld(T1); a.cmp(T2)
        with a.if_nc():
            a.ld(T1); a.sub(T2); a.st(T1)
    a.ld(T1); a.andi(0x1F); put('phy_newOffset')
with a.if_eqi(T0, PU_CALCDONE):
    get('phy_newOffset'); a.add(regs['PHYBASE']); a.andi(0x7FFFF); a.st(T1)
    a.st(N('PHYLBA'))
    a.ld(V('PHYLBA')); a.cmp(T1)
    with a.if_nz():
        puti('physicalUpdateState', PU_READSUB)
        pulse('triggerUpdateSubchannel')
    with a.else_():
        puti('physicalUpdateState', PU_IDLE)
with a.if_eqi(T0, PU_READSUB):
    with if_ev(EV_SUBDONE):
        puti('physicalUpdateState', PU_IDLE)
        for k in range(3):
            a.mov(regs['SUB%d' % k], IN_SUB0 + k)

# ---- new disc
with if_ev(EV_NEWCD):
    dst(D_OPENING)
    fset(F_DRIVEBUSY)
    with a.if_bs(V('MODE'), 7):
        a.ldi(25000000)
    with a.else_():
        a.ldi(13000000)
    a.st(OUT_DRIVEDLY)
    ist_clr(0xE2)
    ist_set(0x10)
    err_drive(0x01, 0x08)

a.place(P3_END)

# ================================================================ commit
a.place(COMMIT)
a.ld(N('PB')); a.andi(0xFF); a.st(OUT_PB)
a.ld(N('PC')); a.andi(0x1FFF); a.st(OUT_PC)
a.mov(OUT_DSTATE, N('DST'))
a.mov(OUT_ISTAT, N('IST'))
a.mov(OUT_MODE, N('MODE'))
a.mov(OUT_XAF, regs['XAF'])
a.mov(OUT_CURLBA, N('CUR'))
a.mov(OUT_PHYLBA, N('PHYLBA'))
for n in DOUBLE:
    a.mov(V(n), N(n))
# hardware flags derived from the new values
fclr(F_WORKGETID | F_LONGCMD | F_CONT | F_PHYWAIT)
get('workCommand'); a.cmpi(0x1A)
with a.if_z():
    fset(F_WORKGETID)
get('nextCmd'); a.st(T0)
for code in (0x0F, 0x10, 0x11, 0x13, 0x14, 0x19):
    a.ld(T0); a.cmpi(code)
    with a.if_z():
        fset(F_LONGCMD)
get('physicalUpdateState'); a.st(T0)
a.cmpi(PU_READSUB)
with a.if_z():
    fset(F_PHYWAIT)
# report (hold): CD-DA report bytes still to push after its interrupt, or Setloc / Setfilter parameters still to
# read, whose last step resets the parameter fifo (hardware: cd_top shows no new interrupt meanwhile, so the CPU
# can't write the next command's parameters before that reset; in the original the steps take a few clocks).
# Written once (set or clear): cd_ctrl looks at it on every clock, also while the engine runs
a.ld(V('STEPS')); a.andi(0xFFFFFF)                         # setLoc / setFilter steps, CDDA_outcnt
with a.if_nz():
    fset(F_REPORT)
with a.else_():
    fclr(F_REPORT)
# eng_cont: pulses pending, parameter steps, CD-DA report bytes, physical update states 1..5
a.ld(V('PB')); a.or_(V('PC')); a.st(T1)
a.ld(V('STEPS')); a.andi(0xFFFFFF); a.or_(T1); a.st(T1)     # setLoc / setFilter steps, CDDA_outcnt
a.ld(T0); a.subi(1); a.cmpi(5)
with a.if_c():
    a.movi(T1, 1)
a.ld(T1)
with a.if_nz():
    fset(F_CONT)
a.wait()

if __name__ == '__main__':
    out = os.path.join(HERE, '..', 'cd_fw_pkg.vhd')
    saved = a.optimize()
    n = a.vhdl(out, ENTRY.addr)
    print('cd_fw: %d words, %d registers -> %s' % (n, len(regs), os.path.normpath(out)))
