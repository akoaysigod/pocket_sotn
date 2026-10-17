"""Assembler for the CD controller engine (rtl/cd_engine.vhd), used by rtl/cd_fw/cd_fw.py.

Instruction word (40 bit): op[39:34] | r[33:27] | imm[26:0] (zero extended); op codes: rtl/cd_engine.vhd
  jumps: target (low bits), bit[16:12] (jbs / jbc), 8-bit value[19:12] (jeq / jne); byte[1:0] (ldb, movb);
  destination[26:20] (movr, movb: the operand is the source); setb: r |= imm, clrb: r &= imm (the assembler's setb / clrb take a bit),
  ors: r = r | A
Operand r: 0..63 register file, 64..95 I/O port r - 64 (read: input, store: output).
The firmware is written in Python with this class: one method per instruction, labels, and structured helpers
(`with a.if_bs(r, bit): ...`, `.else_()`) that emit the jumps.
"""

import contextlib

OPS = {
    # ALU 00ifff (i: immediate source)
    'ld': 0x00, 'and': 0x01, 'or': 0x02, 'xor': 0x03, 'add': 0x04, 'sub': 0x05, 'mul': 0x06, 'cmp': 0x07,
    'ldi': 0x08, 'andi': 0x09, 'ori': 0x0A, 'xori': 0x0B, 'addi': 0x0C, 'subi': 0x0D, 'muli': 0x0E, 'cmpi': 0x0F,
    # misc 01mmmm (only shr1, shr8, ldb change A and the flags)
    'shr1': 0x10, 'shr8': 0x11, 'ldb': 0x12, 'st': 0x13, 'setb': 0x14, 'clrb': 0x15, 'wait': 0x16,
    'sti': 0x17, 'movr': 0x18, 'movb': 0x19, 'ors': 0x1A,
    # jumps 10cccn (n inverts the condition)
    'j': 0x20, 'jz': 0x22, 'jnz': 0x23, 'jc': 0x24, 'jnc': 0x25, 'jbs': 0x26, 'jbc': 0x27,
    'jeq': 0x28, 'jne': 0x29, 'nop': 0x21,
}
IMM_MAX = (1 << 27) - 1
IO = 64          # I/O ports start at operand 64


class Label:
    def __init__(self, name):
        self.name = name
        self.addr = None


class Asm:
    def __init__(self):
        self.code = []          # [op, r, imm, bit, target(Label or None), comment]
        self.labels = []
        self.nlab = 0

    # ---- labels
    def label(self, name=None):
        self.nlab += 1
        lab = Label(name or 'L%d' % self.nlab)
        self.labels.append(lab)
        return lab

    def place(self, lab):
        assert lab.addr is None, 'label placed twice: ' + lab.name
        lab.addr = len(self.code)

    def here(self, name=None):
        lab = self.label(name)
        self.place(lab)
        return lab

    # ---- instructions
    def _e(self, op, r=0, imm=0, bit=0, target=None):
        assert 0 <= r < 96, r
        assert 0 <= imm <= IMM_MAX, ('immediate out of range', op, imm)
        self.code.append([OPS[op], r, imm, bit, target])

    def nop(self): self._e('nop')
    def ldi(self, v): self._e('ldi', imm=v)
    def ld(self, r): self._e('ld', r)
    def st(self, r): self._e('st', r)
    def add(self, r): self._e('add', r)
    def sub(self, r): self._e('sub', r)
    def and_(self, r): self._e('and', r)
    def or_(self, r): self._e('or', r)
    def xor(self, r): self._e('xor', r)
    def addi(self, v): self._e('addi', imm=v)
    def subi(self, v): self._e('subi', imm=v)
    def andi(self, v): self._e('andi', imm=v)
    def ori(self, v): self._e('ori', imm=v)
    def xori(self, v): self._e('xori', imm=v)
    def cmp(self, r): self._e('cmp', r)
    def cmpi(self, v): self._e('cmpi', imm=v)
    def muli(self, v):
        """A = A[26:0] * v (low 32 bits)"""
        self._e('muli', imm=v)
    def mul(self, r): self._e('mul', r)
    def shr1(self): self._e('shr1')
    def shr8(self): self._e('shr8')
    def shl8(self): self.muli(256)
    def j(self, lab): self._e('j', target=lab)
    def jz(self, lab): self._e('jz', target=lab)
    def jnz(self, lab): self._e('jnz', target=lab)
    def jc(self, lab): self._e('jc', target=lab)
    def jnc(self, lab): self._e('jnc', target=lab)
    def jbs(self, r, bit, lab): self._e('jbs', r, bit=bit, target=lab)
    def jbc(self, r, bit, lab): self._e('jbc', r, bit=bit, target=lab)
    def jeq(self, r, v, lab):
        assert 0 <= v < 256
        self._e('jeq', r, bit=v, target=lab)
    def jne(self, r, v, lab):
        assert 0 <= v < 256
        self._e('jne', r, bit=v, target=lab)
    def setb(self, r, bit):
        assert r < 64 and bit < 27
        self._e('setb', r, imm=1 << bit)
    def clrb(self, r, bit):
        """also clears bits 27 .. 31 (an AND with the zero extended immediate)"""
        assert r < 64 and bit < 27
        self._e('clrb', r, imm=IMM_MAX & ~(1 << bit))
    def ldb(self, r, k):
        assert 0 <= k < 4
        self._e('ldb', r, imm=k)
    def sti(self, r, v):
        """r = v (A and the flags unchanged)"""
        self._e('sti', r, imm=v)
    def movr(self, dst, src):
        """dst = src (A and the flags unchanged)"""
        self._e('movr', src, imm=dst << 20)
    def movb(self, dst, src, k):
        """dst = byte k of src"""
        assert 0 <= k < 4
        self._e('movb', src, imm=(dst << 20) | k)
    def orr(self, r, mask):
        """register r |= mask (A and the flags unchanged)"""
        assert r < 64 and 0 < mask <= IMM_MAX
        self._e('setb', r, imm=mask)
    def andr(self, r, mask):
        """register r &= mask, zero extended (A and the flags unchanged)"""
        assert r < 64 and 0 <= mask <= IMM_MAX
        self._e('clrb', r, imm=mask)
    def ors(self, r):
        """register r |= A (A and the flags unchanged)"""
        assert r < 64
        self._e('ors', r)
    def wait(self): self._e('wait')

    # ---- helpers
    def shr(self, n):
        for _ in range(n // 8): self.shr8()
        for _ in range(n % 8): self.shr1()

    def shl(self, n):
        """A <<= n (A < 2**27: the multiplier takes A[26:0])"""
        if n: self.muli(1 << n)

    def ldfield(self, r, lo, width):
        """A = r[lo + width - 1 .. lo]"""
        self.ld(r)
        self.shr(lo)
        if lo + width < 32:
            self.andi((1 << width) - 1)

    def stfield(self, r, lo, width):
        """r[lo + width - 1 .. lo] = A (A must fit in width bits; A is shifted; bits 27 .. 31 of r are cleared)"""
        mask = ((1 << width) - 1) << lo
        assert mask <= IMM_MAX, 'fields above bit 26 are not supported'
        self.shl(lo)
        self.andr(r, IMM_MAX & ~mask)
        self.ors(r)

    def setbit(self, r, bit):
        self.setb(r, bit)

    def clrbit(self, r, bit):
        self.clrb(r, bit)

    def setbits(self, r, mask):
        assert mask <= IMM_MAX
        self.ld(r); self.ori(mask); self.st(r)

    def clrbits(self, r, mask):
        assert mask <= IMM_MAX
        self.ld(r); self.andi(IMM_MAX & ~mask); self.st(r)

    def mov(self, dst, src):
        self.ld(src); self.st(dst)

    def movi(self, dst, v):
        self.ldi(v); self.st(dst)

    # ---- structured control flow
    # with a.if_bs(r, bit): ...     body when the bit is set
    # with a.else_(): ...           right after an if block: body otherwise
    @contextlib.contextmanager
    def _if(self, jump_skip):
        skip = self.label()
        jump_skip(skip)
        self.last_if = None
        try:
            yield
        finally:
            self.place(skip)
            self.last_if = skip

    @contextlib.contextmanager
    def else_(self):
        skip = getattr(self, 'last_if', None)
        assert skip is not None and skip.addr == len(self.code), 'else_ must follow its if block directly'
        end = self.label()
        self.j(end)                 # end of the then part: jump over the else part
        skip.addr += 1              # the condition's jump lands on the else part
        self.last_if = None
        try:
            yield
        finally:
            self.place(end)
            self.last_if = None

    def if_bs(self, r, bit):
        """body if bit set"""
        return self._if(lambda lab: self.jbc(r, bit, lab))

    def if_bc(self, r, bit):
        """body if bit clear"""
        return self._if(lambda lab: self.jbs(r, bit, lab))

    def if_z(self):
        return self._if(lambda lab: self.jnz(lab))

    def if_nz(self):
        return self._if(lambda lab: self.jz(lab))

    def if_c(self):
        """body if the last compare had a borrow (A < operand)"""
        return self._if(lambda lab: self.jnc(lab))

    def if_nc(self):
        """body if A >= operand"""
        return self._if(lambda lab: self.jc(lab))

    def if_eqi(self, r, v):
        if v < 256:
            return self._if(lambda lab: self.jne(r, v, lab))
        self.ld(r); self.cmpi(v)
        return self.if_z()

    def if_nei(self, r, v):
        if v < 256:
            return self._if(lambda lab: self.jeq(r, v, lab))
        self.ld(r); self.cmpi(v)
        return self.if_nz()

    # ---- optimization, where neither A nor the flags it changes are read afterwards (data flow over all paths)
    # and nothing jumps into the sequence:
    #   ldi v; st r -> sti r, v      ld x; st r -> movr r, x      ldb x, k; st r -> movb r, x, k
    #   ld r; ori m; st r -> setb (r |= m)      ld r; andi m; st r -> clrb (r &= m)      (r: register file)
    #   ld r; cmpi v; jnz / jz L -> jne / jeq r, v, L      (v < 256)
    USE_A = {'st', 'ors', 'and', 'or', 'xor', 'add', 'sub', 'mul', 'cmp', 'andi', 'ori', 'xori', 'addi', 'subi',
             'muli', 'cmpi', 'shr1', 'shr8'}
    DEF_A = {'ld', 'ldi', 'ldb', 'and', 'or', 'xor', 'add', 'sub', 'mul', 'andi', 'ori', 'xori', 'addi', 'subi',
             'muli', 'shr1', 'shr8'}
    DEF_Z = DEF_A | {'cmp', 'cmpi'}
    DEF_C = {'add', 'sub', 'cmp', 'addi', 'subi', 'cmpi'}
    JUMPS = {'j', 'jz', 'jnz', 'jc', 'jnc', 'jbs', 'jbc', 'jeq', 'jne'}

    def live_in(self):
        """per instruction: the set of {'A', 'Z', 'C'} read before written from there on"""
        name = {v: k for k, v in OPS.items()}
        code, n = self.code, len(self.code)
        ops = [name[c[0]] for c in code]
        succ = []
        for i, op in enumerate(ops):
            s = []
            if op in self.JUMPS:
                s.append(code[i][4].addr)
            if op not in ('j', 'wait'):
                s.append(i + 1)
            succ.append([x for x in s if x is not None and x < n])
        live = [set() for _ in range(n)]
        changed = True
        while changed:
            changed = False
            for i in range(n - 1, -1, -1):
                op = ops[i]
                x = set()
                for t in succ[i]:
                    x |= live[t]
                if op in self.DEF_A: x.discard('A')
                if op in self.DEF_Z: x.discard('Z')
                if op in self.DEF_C: x.discard('C')
                if op in self.USE_A: x.add('A')
                if op in ('jz', 'jnz'): x.add('Z')
                if op in ('jc', 'jnc'): x.add('C')
                if x != live[i]:
                    live[i] = x
                    changed = True
        return ops, succ, live

    def live_after(self, i):
        ops, succ, live = self.live_in()
        x = set()
        for t in succ[i]:
            x |= live[t]
        return x

    def _match(self, ops, i):
        """(replacement, length, flags that must be dead afterwards) for the sequence at i, or None"""
        code = self.code
        o = ops[i:i + 3] + [None] * 3
        c = code[i:i + 3]
        if o[1] == 'st' and o[0] in ('ldi', 'ld', 'ldb'):
            r = c[1][1]
            if o[0] == 'ldi':
                return [OPS['sti'], r, c[0][2], 0, None], 2, {'A', 'Z'}
            if o[0] == 'ld':
                return [OPS['movr'], c[0][1], r << 20, 0, None], 2, {'A', 'Z'}
            return [OPS['movb'], c[0][1], (r << 20) | c[0][2], 0, None], 2, {'A', 'Z'}
        if o[0] == 'ld' and o[1] in ('ori', 'andi') and o[2] == 'st' and c[0][1] == c[2][1] and c[0][1] < 64:
            return [OPS['setb' if o[1] == 'ori' else 'clrb'], c[0][1], c[1][2], 0, None], 3, {'A', 'Z'}
        if o[0] == 'ld' and o[1] == 'cmpi' and o[2] in ('jnz', 'jz') and c[1][2] < 256:
            return [OPS['jne' if o[2] == 'jnz' else 'jeq'], c[0][1], 0, c[1][2], c[2][4]], 3, {'A', 'Z', 'C'}
        return None

    def optimize(self):
        """apply the sequences above until none is left; returns the number of words saved"""
        total = 0
        while True:
            ops, succ, live = self.live_in()
            targets = {lab.addr for lab in self.labels if lab.addr is not None}
            new, where, i = [], {}, 0
            while i < len(self.code):
                where[i] = len(new)
                m = self._match(ops, i)
                if m and not any(i + k in targets for k in range(1, m[1])):
                    after = set()
                    for t in succ[i + m[1] - 1]:
                        after |= live[t]
                    if not (after & m[2]):
                        new.append(m[0])
                        for k in range(1, m[1]):
                            where[i + k] = None
                        i += m[1]
                        continue
                new.append(self.code[i])
                i += 1
            where[len(self.code)] = len(new)
            for lab in self.labels:
                if lab.addr is not None:
                    assert where[lab.addr] is not None
                    lab.addr = where[lab.addr]
            saved = len(self.code) - len(new)
            self.code = new
            total += saved
            if not saved:
                return total

    # ---- output
    def resolve(self):
        for lab in self.labels:
            assert lab.addr is not None or not any(c[4] is lab for c in self.code), 'label not placed: ' + lab.name

    def words(self):
        self.resolve()
        out = []
        for op, r, imm, bit, target in self.code:
            if target is not None:
                assert target.addr is not None, 'label not placed: ' + target.name
                assert target.addr < 2048
                imm = (bit << 12) | target.addr
            out.append((op << 34) | (r << 27) | imm)
        return out

    def vhdl(self, path, entry):
        w = self.words()
        aw = max(9, (len(w) - 1).bit_length())   # ROM depth: a power of two (Quartus infers its M10K from it)
        size = 1 << aw
        assert size <= 2048, 'firmware too big: %d words' % len(w)
        lines = ['-- generated by tools/cdasm.py from rtl/cd_fw/cd_fw.py: do not edit',
                 'library ieee;', 'use ieee.std_logic_1164.all;', '',
                 'package cd_fw_pkg is',
                 '   constant FW_AW    : integer := %d;   -- address bits' % aw,
                 '   constant FW_SIZE  : integer := %d;' % size,
                 '   constant FW_ENTRY : integer := %d;' % entry,
                 '   constant FW_USED  : integer := %d;' % len(w),
                 '   type tfw is array (0 to FW_SIZE - 1) of std_logic_vector(39 downto 0);',
                 '   constant FW : tfw := (']
        for i, x in enumerate(w):
            lines.append('      %d => x"%010X",' % (i, x))
        lines += ['      others => (others => \'0\')', '   );', 'end package;', '']
        open(path, 'w').write('\n'.join(lines))
        return len(w)
