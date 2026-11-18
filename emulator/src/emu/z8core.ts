/**
 * Zilog Z8 single-chip MCU core (UB8830 D is the GDR version of the Z8601/Z8681 family).
 *
 * This file holds the CPU state, the register file with its control registers,
 * memory/stack access and the fetch-execute loop. The on-chip peripherals live in
 * separate modules: ports + SIO (z8ports.ts), counter/timers (z8timers.ts),
 * interrupt priority logic (z8irq.ts). Opcode semantics are in z8ops.ts.
 *
 * Behaviour follows the Zilog Z8 datasheet and MAME's z8 core (BSD-3-Clause,
 * Curt Coder / AJR), which was used as the reference for edge cases and timing.
 *
 * Time base: one "cycle" here is one internal clock = XTAL / 2 (4 MHz on the JU+TE).
 */
import { CYCLES, OPS } from './z8ops'
import { Z8Ports } from './z8ports'
import { Z8Timers } from './z8timers'
import { pickInterrupt } from './z8irq'
import { ACC_OPCODE, ACC_OPERAND, ACC_STACK, ACC_VECTOR, F_S, F_Z, IMR_ENABLE, P01M_INTERNAL_STACK } from './z8defs'

export * from './z8defs'

export interface Z8Host {
  /** External memory read (only called for addresses outside the internal ROM). */
  memRead(addr: number, kind: number): number
  /** External memory write (only called for addresses outside the internal ROM). */
  memWrite(addr: number, value: number, kind: number): void
  /** Level of the port pins as seen from outside (bits configured as inputs are used). */
  portRead(port: number): number
  /** Port output latch changed. `mask` = bits that are outputs. */
  portWrite(port: number, value: number, mask: number): void
}

export interface Z8Options {
  /** Size of the mask ROM at 0x0000 (0 for ROM-less parts like Z8681/UB8833). */
  romSize: number
  /** Highest general-purpose register (0x7F on Z8601/UB8830). */
  gprTop?: number
}

export class Z8 {
  // ---- CPU registers -------------------------------------------------------
  pc = 0
  /** Address of the instruction being executed (for debuggers / traces). */
  ppc = 0
  sp = 0
  rp = 0
  flags = 0
  imr = 0
  irq = 0
  ipr = 0
  p01m = 0
  /** IRQ requests are ignored until the first EI after reset (Z8 behaviour). */
  irqInitialized = false

  /** Register file 0x00-0xFF; only 0x04..gprTop is plain RAM. */
  readonly regs = new Uint8Array(256)
  readonly gprTop: number
  readonly rom: Uint8Array
  readonly romSize: number

  readonly ports: Z8Ports
  readonly timers: Z8Timers

  /** Total internal clock cycles since power-on. */
  totalCycles = 0
  /** Called before every instruction (debugger/breakpoints). Return false to stop `run`. */
  onBeforeInstruction: ((cpu: Z8) => boolean) | null = null
  /** Set by an illegal opcode; cleared by the host. */
  lastIllegal = -1

  constructor(
    readonly host: Z8Host,
    rom: Uint8Array,
    opts: Z8Options,
  ) {
    this.romSize = opts.romSize
    this.rom = rom
    this.gprTop = opts.gprTop ?? 0x7f
    this.ports = new Z8Ports(this)
    this.timers = new Z8Timers(this)
  }

  // ---- reset ---------------------------------------------------------------

  /** Power-on: clear the register file, then reset. */
  powerOn(): void {
    this.regs.fill(0)
    this.sp = 0
    this.flags = 0
    this.ipr = 0
    this.imr = 0
    this.totalCycles = 0
    this.ports.powerOn()
    this.timers.powerOn()
    this.reset()
  }

  /** /RESET: program starts at 0x000C, control registers get their reset values. */
  reset(): void {
    this.pc = 0x000c
    this.ppc = 0x000c
    this.rp = 0
    this.irq = 0
    this.imr &= ~IMR_ENABLE
    this.irqInitialized = false
    this.timers.reset()
    // ROM parts start with P0/P1 as inputs and internal stack (P01M = 4Dh),
    // ROM-less ones with the external bus enabled.
    this.p01m = this.romSize === 0 ? 0x55 : 0x4d
    this.ports.reset()
  }

  // ---- register file -------------------------------------------------------

  /** Working register rN -> absolute register address. */
  r(n: number): number {
    return (this.rp & 0xf0) | (n & 0x0f)
  }

  /** 8-bit register operand: Ex addresses mean working registers. */
  reg(addr: number): number {
    return (addr & 0xf0) === 0xe0 ? (this.rp & 0xf0) | (addr & 0x0f) : addr
  }

  readReg(a: number): number {
    if (a < 4) return this.ports.read(a)
    if (a <= this.gprTop) return this.regs[a]
    if (a < 0xf0) return 0xff // not implemented on this part
    switch (a) {
      case 0xf0: return this.ports.sioRead()
      case 0xf1: return this.timers.tmr
      case 0xf2: return this.timers.readCounter(1)
      case 0xf4: return this.timers.readCounter(0)
      case 0xfa: return this.irq
      case 0xfb: return this.imr
      case 0xfc: return this.flags
      case 0xfd: return this.rp
      case 0xfe: return this.sp >> 8
      case 0xff: return this.sp & 0xff
      default: return 0xff // write-only: PRE1, PRE0, P2M, P3M, P01M, IPR
    }
  }

  writeReg(a: number, v: number): void {
    v &= 0xff
    if (a < 4) { this.ports.write(a, v); return }
    if (a <= this.gprTop) { this.regs[a] = v; return }
    if (a < 0xf0) return
    switch (a) {
      case 0xf0: this.ports.sioWrite(v); break
      case 0xf1: this.timers.writeTmr(v); break
      case 0xf2: this.timers.t[1] = v; break
      case 0xf3: this.timers.writePre(1, v); break
      case 0xf4: this.timers.t[0] = v; break
      case 0xf5: this.timers.writePre(0, v); break
      case 0xf6: this.ports.p2m = v; break
      case 0xf7: this.ports.writeP3m(v); break
      case 0xf8: this.p01m = v; break
      case 0xf9: this.ipr = v; break
      case 0xfa: if (this.irqInitialized) this.irq = v & 0x3f; break
      case 0xfb: this.imr = v; break
      case 0xfc: this.flags = v; break
      case 0xfd: this.rp = v; break
      case 0xfe: this.sp = (v << 8) | (this.sp & 0xff); break
      case 0xff: this.sp = (this.sp & 0xff00) | v; break
    }
  }

  readPair(a: number): number {
    return (this.readReg(a) << 8) | this.readReg((a + 1) & 0xff)
  }

  writePair(a: number, v: number): void {
    this.writeReg(a, (v >> 8) & 0xff)
    this.writeReg((a + 1) & 0xff, v & 0xff)
  }

  // ---- memory --------------------------------------------------------------

  /**
   * Port 0 supplies the upper address bits only for the nibbles configured as
   * address lines in P01M; otherwise the pin state appears there.
   */
  maskExternalAddress(addr: number): number {
    switch (this.p01m & 0x82) {
      case 0x00: return (addr & 0x00ff) | (this.ports.read(0) << 8)
      case 0x02: return (addr & 0x0fff) | ((this.ports.read(0) & 0xf0) << 8)
      case 0x80: return (addr & 0xf0ff) | ((this.ports.read(0) & 0x0f) << 8)
      default: return addr
    }
  }

  memRead(addr: number, kind: number): number {
    addr &= 0xffff
    if (addr < this.romSize) return this.rom[addr]
    return this.host.memRead(this.maskExternalAddress(addr), kind) & 0xff
  }

  memWrite(addr: number, v: number, kind: number): void {
    addr &= 0xffff
    if (addr < this.romSize) return // mask ROM
    this.host.memWrite(this.maskExternalAddress(addr), v & 0xff, kind)
  }

  fetch(): number {
    const v = this.memRead(this.pc, ACC_OPERAND)
    this.pc = (this.pc + 1) & 0xffff
    return v
  }

  fetchWord(): number {
    const hi = this.fetch()
    return (hi << 8) | this.fetch()
  }

  // ---- stack ---------------------------------------------------------------

  private get internalStack(): boolean {
    return (this.p01m & P01M_INTERNAL_STACK) !== 0
  }

  pushByte(v: number): void {
    if (this.internalStack) {
      const sp = (this.sp - 1) & 0xff
      this.sp = (this.sp & 0xff00) | sp
      this.writeReg(sp, v)
    } else {
      this.sp = (this.sp - 1) & 0xffff
      this.memWrite(this.sp, v, ACC_STACK)
    }
  }

  pushWord(v: number): void {
    if (this.internalStack) {
      const sp = (this.sp - 2) & 0xff
      this.sp = (this.sp & 0xff00) | sp
      this.writePair(sp, v)
    } else {
      this.sp = (this.sp - 2) & 0xffff
      this.memWrite(this.sp, v >> 8, ACC_STACK)
      this.memWrite(this.sp + 1, v & 0xff, ACC_STACK)
    }
  }

  popByte(): number {
    if (this.internalStack) {
      const sp = this.sp & 0xff
      this.sp = (this.sp & 0xff00) | ((sp + 1) & 0xff)
      return this.readReg(sp)
    }
    const v = this.memRead(this.sp, ACC_STACK)
    this.sp = (this.sp + 1) & 0xffff
    return v
  }

  popWord(): number {
    if (this.internalStack) {
      const sp = this.sp & 0xff
      this.sp = (this.sp & 0xff00) | ((sp + 2) & 0xff)
      return this.readPair(sp)
    }
    const hi = this.memRead(this.sp, ACC_STACK)
    const lo = this.memRead(this.sp + 1, ACC_STACK)
    this.sp = (this.sp + 2) & 0xffff
    return (hi << 8) | lo
  }

  // ---- flags ---------------------------------------------------------------

  setFlag(mask: number, on: boolean | number): void {
    if (on) this.flags |= mask
    else this.flags &= ~mask
  }

  /** Set Z and S from an 8-bit result. */
  setZS(v: number): void {
    this.flags = (this.flags & ~(F_Z | F_S)) | (v === 0 ? F_Z : 0) | (v & 0x80 ? F_S : 0)
  }

  // ---- interrupts ----------------------------------------------------------

  /** Peripheral or pin requests IRQn (0..5). */
  requestInterrupt(n: number): void {
    if (this.irqInitialized) this.irq |= 1 << n
  }

  private takeInterrupt(n: number): void {
    this.imr &= ~IMR_ENABLE
    this.irq &= ~(1 << n)
    this.pushWord(this.pc)
    this.pushByte(this.flags)
    const vector = n * 2
    this.pc = (this.memRead(vector, ACC_VECTOR) << 8) | this.memRead(vector + 1, ACC_VECTOR)
  }

  // ---- execution -----------------------------------------------------------

  /** Execute one instruction (or take one interrupt). Returns the cycles used. */
  step(): number {
    let cycles: number
    const n = this.imr & IMR_ENABLE ? pickInterrupt(this.imr & this.irq & 0x3f, this.ipr) : -1
    if (n >= 0) {
      this.takeInterrupt(n)
      cycles = 27 // interrupt acknowledge sequence (MAME: 58 XTAL clocks ~ 27 internal)
    } else {
      this.ppc = this.pc
      const opcode = this.memRead(this.pc, ACC_OPCODE)
      this.pc = (this.pc + 1) & 0xffff
      cycles = CYCLES[opcode] + OPS[opcode](this, opcode)
    }
    this.totalCycles += cycles
    this.timers.advance(cycles)
    return cycles
  }

  /**
   * Run for at least `budget` cycles. Returns the cycles actually executed
   * (less than budget if `onBeforeInstruction` stopped execution).
   */
  run(budget: number): number {
    let done = 0
    const hook = this.onBeforeInstruction
    while (done < budget) {
      if (hook && !hook(this)) break
      done += this.step()
    }
    return done
  }
}
