/**
 * Z8 instruction set: 256-entry opcode table plus base cycle counts.
 *
 * The opcode map is a 16x16 matrix. The upper nibble selects the operation,
 * the lower nibble mostly the addressing mode:
 *
 *   x0 R       x1 IR       x2 r,r      x3 r,Ir     x4 R,R (src,dst)  x5 IR,R
 *   x6 R,IM    x7 IR,IM    x8 LD r,R   x9 LD R,r   xA DJNZ r,RA      xB JR cc,RA
 *   xC LD r,IM xD JP cc,DA xE INC r    xF misc (DI, EI, RET, IRET, RCF, SCF, CCF, NOP)
 *
 * Operand notation: r = working register (4 bit, RP-relative), R = 8 bit register
 * address (Ex = working register), IR/Ir = indirect, RR/Irr = register pair, IM = immediate.
 *
 * Each handler returns *extra* cycles (taken branches). Base counts are internal
 * clocks from the Zilog datasheet / MAME z8 core (BSD-3-Clause).
 */
import type { Z8 } from './z8core'
import { ACC_DATA, ACC_PROGRAM, F_C, F_D, F_H, F_S, F_V, F_Z, IMR_ENABLE } from './z8defs'

export type OpHandler = (c: Z8, opcode: number) => number

// ---------------------------------------------------------------------------
// operand decoders
// ---------------------------------------------------------------------------

/** R operand: fetch 8-bit register address. */
const R = (c: Z8) => c.reg(c.fetch())
/** IR operand: register whose content is the address. */
const IR = (c: Z8) => c.readReg(c.reg(c.fetch()))

// ---------------------------------------------------------------------------
// ALU operations: (cpu, dstRegister, srcValue)
// ---------------------------------------------------------------------------

type Alu2 = (c: Z8, dst: number, src: number) => void
type Alu1 = (c: Z8, dst: number) => void

const add: Alu2 = (c, dst, src) => {
  const d = c.readReg(dst)
  const res = d + src
  const r = res & 0xff
  c.flags =
    (c.flags & ~(F_C | F_Z | F_S | F_V | F_D | F_H)) |
    (res & 0x100 ? F_C : 0) |
    (r === 0 ? F_Z : 0) |
    (r & 0x80 ? F_S : 0) |
    ((d & 0x80) === (src & 0x80) && (r & 0x80) !== (src & 0x80) ? F_V : 0) |
    ((d & 0x0f) + (src & 0x0f) >= 0x10 ? F_H : 0)
  c.writeReg(dst, r)
}

const adc: Alu2 = (c, dst, src) => {
  const d = c.readReg(dst)
  const cy = c.flags & F_C ? 1 : 0
  const res = d + src + cy
  const r = res & 0xff
  c.flags =
    (c.flags & ~(F_C | F_Z | F_S | F_V | F_D | F_H)) |
    (res & 0x100 ? F_C : 0) |
    (r === 0 ? F_Z : 0) |
    (r & 0x80 ? F_S : 0) |
    ((d & 0x80) === (src & 0x80) && (r & 0x80) !== (src & 0x80) ? F_V : 0) |
    ((d & 0x0f) + (src & 0x0f) + cy >= 0x10 ? F_H : 0)
  c.writeReg(dst, r)
}

const sub: Alu2 = (c, dst, src) => {
  const d = c.readReg(dst)
  const res = d - src
  const r = res & 0xff
  c.flags =
    (c.flags & ~(F_C | F_Z | F_S | F_V | F_H)) |
    F_D |
    (res & 0x100 ? F_C : 0) |
    (r === 0 ? F_Z : 0) |
    (r & 0x80 ? F_S : 0) |
    ((d & 0x80) !== (src & 0x80) && (r & 0x80) === (src & 0x80) ? F_V : 0) |
    ((d & 0x0f) < (src & 0x0f) ? F_H : 0)
  c.writeReg(dst, r)
}

const sbc: Alu2 = (c, dst, src) => {
  const d = c.readReg(dst)
  const cy = c.flags & F_C ? 1 : 0
  const res = d - src - cy
  const r = res & 0xff
  c.flags =
    (c.flags & ~(F_C | F_Z | F_S | F_V | F_H)) |
    F_D |
    (res & 0x100 ? F_C : 0) |
    (r === 0 ? F_Z : 0) |
    (r & 0x80 ? F_S : 0) |
    ((d & 0x80) !== (src & 0x80) && (r & 0x80) === (src & 0x80) ? F_V : 0) |
    ((d & 0x0f) < (src & 0x0f) + cy ? F_H : 0)
  c.writeReg(dst, r)
}

const cp: Alu2 = (c, dst, src) => {
  const d = c.readReg(dst)
  const res = d - src
  const r = res & 0xff
  c.flags =
    (c.flags & ~(F_C | F_Z | F_S | F_V)) |
    (res & 0x100 ? F_C : 0) |
    (r === 0 ? F_Z : 0) |
    (r & 0x80 ? F_S : 0) |
    ((d & 0x80) !== (src & 0x80) && (r & 0x80) === (src & 0x80) ? F_V : 0)
}

/** Logic ops: Z, S from result, V cleared. */
function logic(c: Z8, r: number): void {
  c.flags = (c.flags & ~(F_Z | F_S | F_V)) | (r === 0 ? F_Z : 0) | (r & 0x80 ? F_S : 0)
}

const and: Alu2 = (c, dst, src) => {
  const r = c.readReg(dst) & src
  c.writeReg(dst, r)
  logic(c, r)
}
const or: Alu2 = (c, dst, src) => {
  const r = c.readReg(dst) | src
  c.writeReg(dst, r)
  logic(c, r)
}
const xor: Alu2 = (c, dst, src) => {
  const r = c.readReg(dst) ^ src
  c.writeReg(dst, r)
  logic(c, r)
}
/** TCM: test complement under mask. */
const tcm: Alu2 = (c, dst, src) => logic(c, ~c.readReg(dst) & src & 0xff)
/** TM: test under mask. */
const tm: Alu2 = (c, dst, src) => logic(c, c.readReg(dst) & src)
const ld: Alu2 = (c, dst, src) => c.writeReg(dst, src)

// single-operand ops

const dec: Alu1 = (c, dst) => {
  const r = (c.readReg(dst) - 1) & 0xff
  c.setZS(r)
  c.setFlag(F_V, r === 0x7f)
  c.writeReg(dst, r)
}
const inc: Alu1 = (c, dst) => {
  const r = (c.readReg(dst) + 1) & 0xff
  c.setZS(r)
  c.setFlag(F_V, r === 0x80)
  c.writeReg(dst, r)
}
const decw: Alu1 = (c, dst) => {
  const r = (c.readPair(dst) - 1) & 0xffff
  c.flags = (c.flags & ~(F_Z | F_S | F_V)) | (r === 0 ? F_Z : 0) | (r & 0x8000 ? F_S : 0) | (r === 0x7fff ? F_V : 0)
  c.writePair(dst, r)
}
const incw: Alu1 = (c, dst) => {
  const r = (c.readPair(dst) + 1) & 0xffff
  c.flags = (c.flags & ~(F_Z | F_S | F_V)) | (r === 0 ? F_Z : 0) | (r & 0x8000 ? F_S : 0) | (r === 0x8000 ? F_V : 0)
  c.writePair(dst, r)
}
const com: Alu1 = (c, dst) => {
  const r = ~c.readReg(dst) & 0xff
  c.writeReg(dst, r)
  logic(c, r)
}
const clr: Alu1 = (c, dst) => c.writeReg(dst, 0)
const da: Alu1 = (c, dst) => {
  const d = c.readReg(dst)
  let r = d
  const sub = c.flags & F_D
  if (c.flags & F_H || (d & 0x0f) > 9) r += sub ? -6 : 6
  if (c.flags & F_C || d > 0x99) r += sub ? -0x60 : 0x60
  c.setFlag(F_C, c.flags & F_C || r & 0x100 || r < 0)
  r &= 0xff
  c.setZS(r)
  c.writeReg(dst, r)
}

/** Rotates/shifts: C from the bit shifted out, V = sign changed. */
function rot(c: Z8, dst: number, d: number, r: number, carry: number): void {
  r &= 0xff
  c.flags =
    (c.flags & ~(F_C | F_Z | F_S | F_V)) |
    (carry ? F_C : 0) |
    (r === 0 ? F_Z : 0) |
    (r & 0x80 ? F_S : 0) |
    ((d & 0x80) !== (r & 0x80) ? F_V : 0)
  c.writeReg(dst, r)
}
const rl: Alu1 = (c, dst) => { const d = c.readReg(dst); rot(c, dst, d, (d << 1) | (d >> 7), d & 0x80) }
const rlc: Alu1 = (c, dst) => { const d = c.readReg(dst); rot(c, dst, d, (d << 1) | (c.flags & F_C ? 1 : 0), d & 0x80) }
const rr: Alu1 = (c, dst) => { const d = c.readReg(dst); rot(c, dst, d, (d >> 1) | ((d & 1) << 7), d & 1) }
const rrc: Alu1 = (c, dst) => { const d = c.readReg(dst); rot(c, dst, d, (d >> 1) | (c.flags & F_C ? 0x80 : 0), d & 1) }
const sra: Alu1 = (c, dst) => {
  const d = c.readReg(dst)
  const r = (d & 0x80) | (d >> 1)
  c.flags = (c.flags & ~(F_C | F_Z | F_S | F_V)) | (d & 1 ? F_C : 0) | (r === 0 ? F_Z : 0) | (r & 0x80 ? F_S : 0)
  c.writeReg(dst, r)
}
const swap: Alu1 = (c, dst) => {
  const d = c.readReg(dst)
  const r = ((d << 4) | (d >> 4)) & 0xff
  c.writeReg(dst, r)
  c.setZS(r)
}
const pop: Alu1 = (c, dst) => c.writeReg(dst, c.popByte())
const push: Alu1 = (c, src) => c.pushByte(c.readReg(src))

// ---------------------------------------------------------------------------
// addressing-mode wrappers
// ---------------------------------------------------------------------------

/** x2: op r1,r2 */
const m_r_r = (f: Alu2): OpHandler => (c) => {
  const b = c.fetch()
  f(c, c.r(b >> 4), c.readReg(c.r(b)))
  return 0
}
/** x3: op r1,@r2 */
const m_r_Ir = (f: Alu2): OpHandler => (c) => {
  const b = c.fetch()
  f(c, c.r(b >> 4), c.readReg(c.readReg(c.r(b))))
  return 0
}
/** x4: op R1,R2 (encoded src, dst) */
const m_R_R = (f: Alu2): OpHandler => (c) => {
  const src = c.readReg(R(c))
  f(c, R(c), src)
  return 0
}
/** x5: op R1,@R2 (encoded src, dst) */
const m_R_IR = (f: Alu2): OpHandler => (c) => {
  const src = c.readReg(c.readReg(R(c)))
  f(c, R(c), src)
  return 0
}
/** x6: op R1,#IM */
const m_R_IM = (f: Alu2): OpHandler => (c) => {
  const dst = R(c)
  f(c, dst, c.fetch())
  return 0
}
/** x7: op @R1,#IM */
const m_IR_IM = (f: Alu2): OpHandler => (c) => {
  const dst = IR(c)
  f(c, dst, c.fetch())
  return 0
}
/** x0: op R1 */
const m_R = (f: Alu1): OpHandler => (c) => {
  f(c, R(c))
  return 0
}
/** x1: op @R1 */
const m_IR = (f: Alu1): OpHandler => (c) => {
  f(c, IR(c))
  return 0
}

/** Rows 0-7 and A, B of columns 2-7 follow the same pattern. */
function aluRow(f: Alu2): OpHandler[] {
  return [m_r_r(f), m_r_Ir(f), m_R_R(f), m_R_IR(f), m_R_IM(f), m_IR_IM(f)]
}

// ---------------------------------------------------------------------------
// branches
// ---------------------------------------------------------------------------

function condition(c: Z8, cc: number): boolean {
  const f = c.flags
  const C = (f & F_C) !== 0
  const Z = (f & F_Z) !== 0
  const S = (f & F_S) !== 0
  const V = (f & F_V) !== 0
  switch (cc) {
    case 0x0: return false // F
    case 0x1: return S !== V // LT
    case 0x2: return Z || S !== V // LE
    case 0x3: return C || Z // ULE
    case 0x4: return V // OV
    case 0x5: return S // MI
    case 0x6: return Z // Z / EQ
    case 0x7: return C // C / ULT
    case 0x8: return true // always
    case 0x9: return S === V // GE
    case 0xa: return !(Z || S !== V) // GT
    case 0xb: return !C && !Z // UGT
    case 0xc: return !V // NOV
    case 0xd: return !S // PL
    case 0xe: return !Z // NZ / NE
    default: return !C // NC / UGE
  }
}

const djnz: OpHandler = (c, op) => {
  const ra = (c.fetch() << 24) >> 24
  const r = c.r(op >> 4)
  const v = (c.readReg(r) - 1) & 0xff
  c.writeReg(r, v)
  if (v !== 0) {
    c.pc = (c.pc + ra) & 0xffff
    return 2
  }
  return 0
}

const jr: OpHandler = (c, op) => {
  const ra = (c.fetch() << 24) >> 24
  if (condition(c, op >> 4)) {
    c.pc = (c.pc + ra) & 0xffff
    return 2
  }
  return 0
}

const jp: OpHandler = (c, op) => {
  const da = c.fetchWord()
  if (condition(c, op >> 4)) {
    c.pc = da
    return 2
  }
  return 0
}

// ---------------------------------------------------------------------------
// LDC / LDE (program / data memory via register pair)
// ---------------------------------------------------------------------------

const ldFromMem = (kind: number): OpHandler => (c) => {
  const b = c.fetch()
  const addr = c.readPair(c.r(b))
  c.writeReg(c.r(b >> 4), c.memRead(addr, kind))
  return 0
}
const ldToMem = (kind: number): OpHandler => (c) => {
  const b = c.fetch()
  const addr = c.readPair(c.r(b))
  c.memWrite(addr, c.readReg(c.r(b >> 4)), kind)
  return 0
}
/** LDCI/LDEI @r1,@rr2: auto-increment both pointers. */
const ldFromMemInc = (kind: number): OpHandler => (c) => {
  const b = c.fetch()
  const dst = c.r(b >> 4)
  const realDst = c.readReg(dst)
  const src = c.r(b)
  const addr = c.readPair(src)
  c.writeReg(realDst, c.memRead(addr, kind))
  c.writeReg(dst, realDst + 1)
  c.writePair(src, addr + 1)
  return 0
}
const ldToMemInc = (kind: number): OpHandler => (c) => {
  const b = c.fetch()
  const src = c.r(b >> 4)
  const realSrc = c.readReg(src)
  const dst = c.r(b)
  const addr = c.readPair(dst)
  c.memWrite(addr, c.readReg(realSrc), kind)
  c.writePair(dst, addr + 1)
  c.writeReg(src, realSrc + 1)
  return 0
}

// ---------------------------------------------------------------------------
// table
// ---------------------------------------------------------------------------

const illegal: OpHandler = (c, op) => {
  c.lastIllegal = op
  return 0
}

export const OPS: OpHandler[] = new Array(256).fill(illegal)
/** Base cycles (internal clocks). Branch handlers add 2 when taken. */
export const CYCLES = new Uint8Array(256).fill(6)

function set(op: number, cycles: number, h: OpHandler): void {
  OPS[op] = h
  CYCLES[op] = cycles
}

// columns 8..E are identical for every row
for (let row = 0; row < 16; row++) {
  const base = row << 4
  set(base | 0x8, 6, (c, op) => { ld(c, c.r(op >> 4), c.readReg(R(c))); return 0 }) // LD r,R
  set(base | 0x9, 6, (c, op) => { ld(c, R(c), c.readReg(c.r(op >> 4))); return 0 }) // LD R,r
  set(base | 0xa, 10, djnz)
  set(base | 0xb, 10, jr)
  set(base | 0xc, 6, (c, op) => { c.writeReg(c.r(op >> 4), c.fetch()); return 0 }) // LD r,IM
  set(base | 0xd, 10, jp)
  set(base | 0xe, 6, (c, op) => { inc(c, c.r(op >> 4)); return 0 }) // INC r
}

// ALU rows: columns 2..7 = r,r  r,Ir  R,R  R,IR  R,IM  IR,IM
const ALU_ROWS: [number, Alu2][] = [
  [0x0, add], [0x1, adc], [0x2, sub], [0x3, sbc],
  [0x4, or], [0x5, and], [0x6, tcm], [0x7, tm],
  [0xa, cp], [0xb, xor],
]
for (const [row, f] of ALU_ROWS) {
  aluRow(f).forEach((h, i) => set((row << 4) | (i + 2), i < 2 ? 6 : 10, h))
}

// columns 0/1: single-operand ops (R / IR)
const UNARY: [number, Alu1, number][] = [
  [0x0, dec, 6], [0x1, rlc, 6], [0x2, inc, 6], [0x4, da, 8],
  [0x5, pop, 10], [0x6, com, 6], [0x8, decw, 10], [0x9, rl, 6],
  [0xa, incw, 10], [0xb, clr, 6], [0xc, rrc, 6], [0xd, sra, 6],
  [0xe, rr, 6], [0xf, swap, 8],
]
for (const [row, f, cy] of UNARY) {
  set(row << 4, cy, m_R(f))
  set((row << 4) | 1, cy, m_IR(f))
}
set(0x70, 10, m_R(push)) // PUSH R
set(0x71, 12, m_IR(push)) // PUSH @R
set(0x30, 8, (c) => { c.pc = c.readPair(c.reg(c.fetch())); return 0 }) // JP @RR
set(0x31, 6, (c) => { c.writeReg(0xfd, c.fetch()); return 0 }) // SRP #IM

// LDE / LDEI / LDC / LDCI
set(0x82, 12, ldFromMem(ACC_DATA)) // LDE r,@rr
set(0x83, 18, ldFromMemInc(ACC_DATA)) // LDEI @r,@rr
set(0x92, 12, ldToMem(ACC_DATA)) // LDE @rr,r
set(0x93, 18, ldToMemInc(ACC_DATA)) // LDEI @rr,@r
set(0xc2, 12, ldFromMem(ACC_PROGRAM)) // LDC r,@rr
set(0xc3, 18, ldFromMemInc(ACC_PROGRAM)) // LDCI @r,@rr
set(0xd2, 12, ldToMem(ACC_PROGRAM)) // LDC @rr,r
set(0xd3, 18, ldToMemInc(ACC_PROGRAM)) // LDCI @rr,@r

// LD variants
set(0xc7, 10, (c) => { // LD r1, X(r2)
  const b = c.fetch()
  const dst = c.r(b >> 4)
  const index = c.readReg(c.r(b))
  c.writeReg(dst, c.readReg((R(c) + index) & 0xff))
  return 0
})
set(0xd7, 10, (c) => { // LD X(r2), r1
  const b = c.fetch()
  const index = c.readReg(c.r(b))
  const dst = (R(c) + index) & 0xff
  c.writeReg(dst, c.readReg(c.r(b >> 4)))
  return 0
})
set(0xe3, 6, m_r_Ir(ld)) // LD r1,@r2
set(0xf3, 6, (c) => { // LD @r1,r2
  const b = c.fetch()
  c.writeReg(c.readReg(c.r(b >> 4)), c.readReg(c.r(b)))
  return 0
})
set(0xe4, 10, m_R_R(ld)) // LD R1,R2
set(0xe5, 10, m_R_IR(ld)) // LD R1,@R2
set(0xe6, 10, m_R_IM(ld)) // LD R1,#IM
set(0xe7, 10, m_IR_IM(ld)) // LD @R1,#IM
set(0xf5, 10, (c) => { // LD @R1,R2 (encoded src, dst)
  const src = c.readReg(R(c))
  c.writeReg(IR(c), src)
  return 0
})

// calls / returns
set(0xd4, 20, (c) => { // CALL @RR
  const dst = c.readPair(c.reg(c.fetch()))
  c.pushWord(c.pc)
  c.pc = dst
  return 0
})
set(0xd6, 20, (c) => { // CALL DA
  const dst = c.fetchWord()
  c.pushWord(c.pc)
  c.pc = dst
  return 0
})
set(0xaf, 14, (c) => { c.pc = c.popWord(); return 0 }) // RET
set(0xbf, 16, (c) => { // IRET
  c.flags = c.popByte()
  c.pc = c.popWord()
  c.imr |= IMR_ENABLE
  return 0
})

// CPU control
set(0x8f, 6, (c) => { c.imr &= ~IMR_ENABLE; return 0 }) // DI
set(0x9f, 6, (c) => { c.imr |= IMR_ENABLE; c.irqInitialized = true; return 0 }) // EI
set(0xcf, 6, (c) => { c.flags &= ~F_C; return 0 }) // RCF
set(0xdf, 6, (c) => { c.flags |= F_C; return 0 }) // SCF
set(0xef, 6, (c) => { c.flags ^= F_C; return 0 }) // CCF
set(0xff, 6, () => 0) // NOP
