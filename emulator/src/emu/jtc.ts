/**
 * The JU+TE computer ("Tiny", Jugend+Technik 7/1987 - 2/1988), minimal configuration:
 *
 *   0000-07FF  UB8830 internal ROM: TINY-MP-BASIC (inside the MCU)
 *   0800-0FFF  2 KByte EPROM U2716: operating system ("Editor"), /CS0
 *   6000-7FFF  keyboard, /CS3: the latched low address bits A0-A3 select one of
 *              16 columns (74154), the four rows come back on D4-D7 (V40098).
 *   E000-FFFF  RAM, /CS7. Minimal 1 KByte (2x U214) is mirrored over the 8 KByte
 *              block because the address is not fully decoded; FE00-FFFF is the
 *              picture memory (64x64 pixels, 1 bit each).
 *
 * The picture is produced in software: the OS's interrupt routine (entered via
 * 080Ch) reads the picture memory byte by byte in sync with the TV lines; a shift
 * register clocked with XTAL/4 serializes each byte. Here we simply render FE00-FFFF
 * and blank the screen when that routine has not run for a while (the OS switches
 * the picture off, e.g. during cassette I/O).
 *
 * P3: P36 = sound / cassette out, P37 = sync, P30 = cassette in, P32 tied low.
 */
import type { Z8Host } from './z8core'
import { ACC_OPCODE } from './z8defs'
import { CPU_HZ, UB8830 } from './ub8830'
import { OS2K_ROM } from './roms/os2k'

export const SCREEN_W = 64
export const SCREEN_H = 64
export const VRAM_BASE = 0xfe00
export const VIDEO_IRQ_ENTRY = 0x080c

/** One external bus cycle, as an Arduino on the real bus would see it. */
export interface BusCycle {
  n: number
  cycle: number
  write: boolean
  /** 0 opcode fetch, 1 operand, 2 LDC, 3 LDE, 4 stack, 5 vector */
  kind: number
  addr: number
  data: number
}

export const RAM_SIZES = [1024, 2048, 8192, 32768] as const
export type RamSize = (typeof RAM_SIZES)[number]

export class JuTe implements Z8Host {
  readonly cpu: UB8830
  readonly os = OS2K_ROM

  ramSize: RamSize
  private ram: Uint8Array
  private ramBase = 0xe000
  private ramMask = 0x3ff

  /** Keyboard matrix: bit n of keyCols[col] = key in row n is pressed. */
  readonly keyCols = new Uint8Array(16)

  /** Cassette input level (P30), unless a tape source is playing. */
  tapeIn = false
  /** Cassette playback: P30 level at a given CPU cycle (see tape.ts). */
  tapeSource: ((cycle: number) => boolean) | null = null
  /** Speaker level derived from P36/P37 and the cycle times of its edges. */
  speaker = false
  readonly speakerEdges: number[] = []
  /** Cassette output level (2K OS: P36/P37 != 1/1) and the cycle times of its edges. */
  tapeOut = false
  readonly tapeOutEdges: number[] = []

  private lastVideoIrq = -Infinity

  /** Bus trace (first N external accesses after reset); null = off. */
  trace: BusCycle[] | null = null
  traceLimit = 2000
  private traceN = 0

  constructor(ramSize: RamSize = 2048) {
    this.ramSize = ramSize
    this.ram = new Uint8Array(ramSize)
    this.setRamSize(ramSize)
    this.cpu = new UB8830(this)
    this.powerOn()
  }

  setRamSize(size: RamSize): void {
    this.ramSize = size
    this.ram = new Uint8Array(size)
    if (size <= 0x2000) {
      this.ramBase = 0xe000 // mirrored inside E000-FFFF
      this.ramMask = size - 1
    } else {
      this.ramBase = 0x10000 - size
      this.ramMask = size - 1
    }
  }

  powerOn(): void {
    this.ram.fill(0)
    this.keyCols.fill(0)
    this.lastVideoIrq = -Infinity
    this.speakerEdges.length = 0
    this.startTrace()
    this.cpu.powerOn()
  }

  /** RESET key: pulls /RESET of the UB8830 low. RAM (and the BASIC program) survives. */
  reset(): void {
    this.keyCols.fill(0)
    this.startTrace()
    this.cpu.reset()
  }

  private startTrace(): void {
    this.traceN = 0
    if (this.trace) this.trace.length = 0
  }

  // ---- memory map ------------------------------------------------------------

  private ramIndex(addr: number): number {
    return addr >= this.ramBase ? addr & this.ramMask : -1
  }

  /** Side-effect free read for debuggers and the screen. */
  peek(addr: number): number {
    addr &= 0xffff
    if (addr < 0x800) return this.cpu.rom[addr]
    if (addr < 0x1000) return this.os[addr - 0x800]
    if (addr >= 0x6000 && addr < 0x8000) return (this.keyCols[addr & 0x0f] << 4) | 0x0f
    const i = this.ramIndex(addr)
    return i >= 0 ? this.ram[i] : 0xff
  }

  poke(addr: number, v: number): void {
    const i = this.ramIndex(addr & 0xffff)
    if (i >= 0) this.ram[i] = v & 0xff
  }

  memRead(addr: number, kind: number): number {
    if (kind === ACC_OPCODE && addr === VIDEO_IRQ_ENTRY) this.lastVideoIrq = this.cpu.totalCycles
    const v = this.peek(addr)
    if (this.trace && this.traceN < this.traceLimit) this.record(false, kind, addr, v)
    return v
  }

  memWrite(addr: number, v: number, kind: number): void {
    const i = this.ramIndex(addr)
    if (i >= 0) this.ram[i] = v
    if (this.trace && this.traceN < this.traceLimit) this.record(true, kind, addr, v)
  }

  private record(write: boolean, kind: number, addr: number, data: number): void {
    this.trace!.push({ n: this.traceN++, cycle: this.cpu.totalCycles, write, kind, addr, data })
  }

  // ---- ports -----------------------------------------------------------------

  portRead(port: number): number {
    if (port !== 3) return 0xff // P0/P1 are the bus, P2 is free (pulled up)
    let v = 0xff & ~0x04 // P32 = 0 (recommended in the J+T errata: no test mode)
    const tape = this.tapeSource ? this.tapeSource(this.cpu.totalCycles) || this.tapeIn : this.tapeIn
    if (!tape) v &= ~0x01
    return v
  }

  portWrite(port: number, value: number): void {
    if (port !== 3) return
    // 2K OS: P36/P37 = 0/1 drive the speaker (JTCEMU: phase = value&C0 != 40)
    const level = (value & 0xc0) !== 0x40
    if (level !== this.speaker) {
      this.speaker = level
      this.speakerEdges.push(this.cpu.totalCycles)
    }
    const tape = (value & 0xc0) !== 0xc0
    if (tape !== this.tapeOut) {
      this.tapeOut = tape
      this.tapeOutEdges.push(this.cpu.totalCycles)
    }
  }

  // ---- video -----------------------------------------------------------------

  /** True while the OS's picture routine runs (otherwise the TV shows black). */
  get screenOn(): boolean {
    return this.cpu.totalCycles - this.lastVideoIrq < CPU_HZ / 10
  }

  /** Pixel (x,y) lit? Bits are MSB first, a 0 bit is a bright pixel. */
  pixel(x: number, y: number): boolean {
    const b = this.peek(VRAM_BASE + y * 8 + (x >> 3))
    return (b & (0x80 >> (x & 7))) === 0
  }

  /** Screen as text (# = lit), handy for tests and debugging. */
  screenAscii(): string {
    const lines: string[] = []
    for (let y = 0; y < SCREEN_H; y++) {
      let s = ''
      for (let x = 0; x < SCREEN_W; x++) s += this.pixel(x, y) ? '#' : '.'
      lines.push(s)
    }
    return lines.join('\n')
  }

  // ---- running ---------------------------------------------------------------

  /** Run for `seconds` of emulated time. */
  runSeconds(seconds: number): number {
    return this.cpu.run(Math.round(seconds * CPU_HZ))
  }
}
