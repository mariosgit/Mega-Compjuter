/**
 * Z8 I/O ports P0..P3 and the serial interface (SIO).
 *
 * P0  (00h): P01M bits 1-0 (low nibble) / 7-6 (high nibble): output, input, or
 *            address A8-A11 / A12-A15 for external memory.
 * P1  (01h): P01M bits 4-3: output, input, multiplexed address/data AD0-AD7, high-Z.
 * P2  (02h): direction per bit in P2M (1 = input). P3M bit 0 = active pull-ups
 *            (otherwise open drain).
 * P3  (03h): P30-P33 are always inputs (IRQ3, IRQ2/TIN, IRQ0, IRQ1 on falling edges,
 *            P30 = serial in). P34-P37 are always outputs; P36 can be TOUT and P37
 *            serial out (P3M bit 6). P34 can be /DM (data memory strobe).
 * SIO (F0h): async 8 bit, 1 start, 2 stop bits; bit clock = T0 end-of-count / 16.
 *
 * Port logic follows MAME's z8 core (BSD-3-Clause), which was used as reference.
 */
import type { Z8 } from './z8core'

const P3M_PULLUPS = 0x01
const P3M_SERIAL = 0x40
const P3M_PARITY = 0x80

export class Z8Ports {
  /** Output latches. */
  readonly out = [0, 0, 0, 0]
  p2m = 0xff
  p3m = 0
  /** Current P34-P37 output levels (after TOUT/SOUT muxing). */
  p3Output = 0
  /** Last seen P30-P33 input levels (for edge detection). */
  private p3In = 0x0f

  // SIO state
  private txShift = 0
  private txCount = 0
  private txParity = false
  private rxBuffer = 0
  private rxShift = 0
  private rxCount = 0
  private rxParity = false
  private rxStarted = false

  constructor(private readonly cpu: Z8) {}

  powerOn(): void {
    this.out.fill(0)
    this.p3Output = 0
    this.p3In = this.cpu.host.portRead(3) & 0x0f
    this.rxBuffer = 0
  }

  reset(): void {
    this.out[3] = 0xf0
    this.p2m = 0xff
    this.writeP3m(0)
  }

  get serialMode(): boolean {
    return (this.p3m & P3M_SERIAL) !== 0
  }

  /** Raw pin level as driven from outside. */
  pinLevel(port: number): number {
    return this.cpu.host.portRead(port) & 0xff
  }

  read(port: number): number {
    const p01m = this.cpu.p01m
    switch (port) {
      case 0: {
        let data = 0
        let mask = 0
        switch (p01m & 0x03) {
          case 0x00: data = this.out[0] & 0x0f; break
          case 0x01: mask = 0x0f; break
          default: data = 0x0f // A8-A11
        }
        switch (p01m & 0xc0) {
          case 0x00: data |= this.out[0] & 0xf0; break
          case 0x40: mask |= 0xf0; break
          default: data |= 0xf0 // A12-A15
        }
        return data | (mask ? this.pinLevel(0) & mask : 0)
      }
      case 1:
        switch (p01m & 0x18) {
          case 0x00: return this.out[1]
          case 0x08: return this.pinLevel(1)
          default: return 0xff // AD0-AD7 / high-Z
        }
      case 2: {
        let mask = this.p2m
        if (!(this.p3m & P3M_PULLUPS)) mask |= this.out[2] // open drain: high outputs readable
        return ((this.pinLevel(2) & mask) | (this.out[2] & ~mask)) & 0xff
      }
      default:
        return (this.pinLevel(3) & 0x0f) | (this.p3Output & 0xf0)
    }
  }

  write(port: number, v: number): void {
    this.out[port] = v
    const p01m = this.cpu.p01m
    switch (port) {
      case 0: {
        let mask = 0
        if ((p01m & 0x03) === 0) mask |= 0x0f
        if ((p01m & 0xc0) === 0) mask |= 0xf0
        if (mask) this.cpu.host.portWrite(0, v & mask, mask)
        break
      }
      case 1:
        if ((p01m & 0x18) === 0) this.cpu.host.portWrite(1, v, 0xff)
        break
      case 2: {
        const mask = ~this.p2m & 0xff
        if (mask) this.cpu.host.portWrite(2, v & mask, mask)
        break
      }
      default:
        this.out[3] = v & 0xf0
        this.updateP3Output()
    }
  }

  writeP3m(v: number): void {
    if (!(v & P3M_SERIAL)) {
      this.txShift = 0
      this.txCount = 0
      this.rxStarted = false
      this.rxCount = 0
    }
    this.p3m = v
    this.updateP3Output()
  }

  /** Recompute P34-P37 (latch + TOUT on P36 + serial out on P37) and notify the host. */
  updateP3Output(): void {
    let output = this.out[3] & 0xf0
    const timers = this.cpu.timers
    if (timers.toutEnabled) output = (output & ~0x40) | (timers.tout ? 0x40 : 0)
    if (this.serialMode) output = (output & ~0x80) | (this.txShift === 0 || this.txShift & 1 ? 0x80 : 0)
    if (output !== this.p3Output) {
      this.p3Output = output
      this.cpu.host.portWrite(3, output, 0xf0)
    }
  }

  /**
   * The board changed P30-P33. Detects falling edges:
   * P32 -> IRQ0, P33 -> IRQ1, P31 -> IRQ2 (+ TIN), P30 -> IRQ3 (unless serial mode).
   */
  inputChanged(): void {
    const now = this.pinLevel(3) & 0x0f
    const fell = this.p3In & ~now
    const rose = ~this.p3In & now
    this.p3In = now
    if (fell & 0x04) this.cpu.requestInterrupt(0)
    if (fell & 0x08) this.cpu.requestInterrupt(1)
    if (fell & 0x02) {
      this.cpu.requestInterrupt(2)
      this.cpu.timers.tinEdge()
    }
    if (rose & 0x02) this.cpu.timers.tinRise()
    if (fell & 0x01 && !this.serialMode) this.cpu.requestInterrupt(3)
  }

  // ---- SIO -----------------------------------------------------------------

  sioRead(): number {
    return this.rxBuffer
  }

  sioWrite(v: number): void {
    // data + 1 start bit + 2 stop bits; keep the current line state in bit 0
    this.txShift = (this.txShift & 1) | (v << 2) | (3 << 10)
    this.txParity = false
    this.txCount = 15
  }

  /** One T0 end-of-count in serial mode = one 16x bit clock. */
  sioClock(): void {
    this.sioReceive()
    this.sioTransmit()
  }

  private serialIn(): boolean {
    return (this.pinLevel(3) & 0x01) !== 0
  }

  private sioReceive(): void {
    if (!this.rxStarted) {
      this.rxShift >>= 1
      if (this.serialIn()) this.rxShift |= 1 << 9
      else if (this.rxShift & (1 << 8)) {
        this.rxStarted = true // high-to-low transition: start bit
        this.rxShift = 0
        this.rxCount = 0
      }
      return
    }
    this.rxCount = (this.rxCount + 1) & 15
    if (this.rxCount !== 8) return // sample in the middle of the bit
    if (this.rxShift === 0) {
      if (!this.serialIn()) {
        this.rxShift |= 1 << 9 // start bit confirmed
        this.rxParity = false
      } else {
        this.rxStarted = false // glitch
        this.rxCount = 0
      }
      return
    }
    this.rxShift >>= 1
    if (this.serialIn()) this.rxShift |= 1 << 9
    if (this.rxShift & 1) {
      this.rxBuffer = (this.rxShift & 0x1fe) >> 1
      this.cpu.requestInterrupt(3)
      this.rxStarted = false
      this.rxCount = 0
    } else {
      if (this.rxShift & (1 << 9)) this.rxParity = !this.rxParity
      if (this.rxShift & 2 && this.p3m & P3M_PARITY) {
        if (this.rxParity) this.rxShift |= 1 << 9
        else this.rxShift &= ~(1 << 9)
      }
    }
  }

  private sioTransmit(): void {
    if (this.txShift === 0) return
    this.txCount = (this.txCount + 1) & 15
    if (this.txCount !== 0) return
    this.txShift >>= 1
    if (this.txShift === 0) {
      this.cpu.requestInterrupt(4) // transmit buffer empty
      return
    }
    if (this.txShift >> 1 === 3 && this.p3m & P3M_PARITY) {
      if (this.txParity) this.txShift |= 1
      else this.txShift &= ~1
    } else if (this.txShift & 1) this.txParity = !this.txParity
    this.updateP3Output()
  }
}
