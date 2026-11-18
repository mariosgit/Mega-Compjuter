/**
 * Z8 counter/timers T0 and T1.
 *
 * Each timer = 6-bit prescaler (PREx bits 7..2, 0 means 64) feeding an 8-bit
 * down counter (Tx, 0 means 256). Bit 0 of PREx selects continuous (modulo-N)
 * versus single-pass mode. The prescalers are clocked with internal clock / 4,
 * i.e. XTAL / 8 = 1 MHz on the JU+TE. T1 can alternatively be clocked, gated or
 * triggered by TIN (P31) when PRE1 bit 1 is 0.
 *
 * TMR (F1h):  bit0 load T0, bit1 enable T0, bit2 load T1, bit3 enable T1,
 *             bits5-4 TIN mode, bits7-6 TOUT mode (off / T0 / T1 / clock) on P36.
 *
 * Instead of ticking every 4 cycles the remaining time to the next end-of-count
 * is tracked in CPU cycles (same approach as MAME, which this follows).
 */
import type { Z8 } from './z8core'

const TMR_LOAD = [0x01, 0x04]
const TMR_ENABLE = [0x02, 0x08]
const PRE_MODULO = 0x01
const PRE1_INTERNAL = 0x02
const TIN_MASK = 0x30
const TIN_EXTERNAL_CLOCK = 0x00
const TIN_GATE = 0x10
const TIN_TRIGGER = 0x20
const TOUT_MASK = 0xc0
const TOUT_OFF = 0x00
const TOUT_T = [0x40, 0x80]

export class Z8Timers {
  tmr = 0
  /** Reload values written to T0/T1. */
  readonly t = [0, 0]
  /** PRE0/PRE1 as written. */
  readonly pre = [0, 0]
  /** Current counter / prescaler values (valid while stopped). */
  readonly count = [0, 0]
  readonly preCount = [0, 0]
  /** Running timers: cycles until end of count. */
  readonly running = [false, false]
  readonly remaining = [0, 0]
  /** TOUT output level (drives P36 when enabled). */
  tout = true

  constructor(private readonly cpu: Z8) {}

  powerOn(): void {
    this.t.fill(0)
    this.pre.fill(0)
    this.count.fill(0)
    this.preCount.fill(0)
    this.tout = true
  }

  reset(): void {
    this.pre[0] &= ~PRE_MODULO
    this.pre[1] &= ~(PRE_MODULO | PRE1_INTERNAL)
    this.tmr = 0
    this.stop(0)
    this.stop(1)
  }

  private prescaler(n: number): number {
    return this.pre[n] >> 2 || 64
  }

  private start(n: number): void {
    const p = this.prescaler(n)
    const full = (this.count[n] ? this.count[n] - 1 : 255) * p + (this.preCount[n] || 64)
    this.remaining[n] = 4 * full
    this.running[n] = true
  }

  private stop(n: number): void {
    if (!this.running[n]) return
    const p = this.prescaler(n)
    const rem = Math.max(0, Math.floor(this.remaining[n] / 4))
    this.count[n] = (Math.floor(rem / p) + 1) & 0xff
    this.preCount[n] = ((rem % p) + 1) & 0x3f
    this.running[n] = false
  }

  /** End of count: toggle TOUT, raise the interrupt (or clock the SIO), reload. */
  private end(n: number): void {
    if ((this.tmr & TOUT_MASK) === TOUT_T[n]) this.toggleTout()
    const ports = this.cpu.ports
    if (n === 0 && ports.serialMode) ports.sioClock()
    else this.cpu.requestInterrupt(4 + n)
    this.preCount[n] = this.pre[n] >> 2
    if (this.pre[n] & PRE_MODULO) this.count[n] = this.t[n]
    else this.tmr &= ~TMR_ENABLE[n]
  }

  /** Called by the core after each instruction. */
  advance(cycles: number): void {
    for (let n = 0; n < 2; n++) {
      if (!this.running[n]) continue
      this.remaining[n] -= cycles
      while (this.running[n] && this.remaining[n] <= 0) {
        const overshoot = this.remaining[n]
        this.end(n)
        if (this.pre[n] & PRE_MODULO) {
          this.start(n)
          this.remaining[n] += overshoot
        } else {
          this.count[n] = 0
          this.running[n] = false
        }
      }
    }
  }

  readCounter(n: number): number {
    if (!this.running[n]) return this.count[n]
    const rem = Math.max(0, Math.floor(this.remaining[n] / 4))
    return (Math.floor(rem / this.prescaler(n)) + 1) & 0xff
  }

  writePre(n: number, v: number): void {
    if (n === 0) {
      if (this.running[0]) {
        this.stop(0)
        this.pre[0] = v
        this.start(0)
      } else this.pre[0] = v
      return
    }
    const wasRunning = this.running[1]
    if (wasRunning) this.stop(1)
    this.pre[1] = v
    const restart = v & PRE1_INTERNAL
      ? (this.tmr & TMR_ENABLE[1]) !== 0
      : wasRunning && (this.tmr & TIN_MASK) !== TIN_EXTERNAL_CLOCK
    if (restart) this.start(1)
  }

  writeTmr(v: number): void {
    this.tmr = v & ~(TMR_LOAD[0] | TMR_LOAD[1]) // load bits are self-clearing
    let t1Internal = (this.pre[1] & PRE1_INTERNAL) !== 0
    let t1Enable = (v & TMR_ENABLE[1]) !== 0
    if (!t1Internal && (v & TIN_MASK) === TIN_GATE) {
      if (this.cpu.ports.pinLevel(3) & 0x02) t1Internal = true // gate open (P31 high)
      else t1Enable = false
    }
    for (let n = 0; n < 2; n++) {
      const load = (v & TMR_LOAD[n]) !== 0
      const enable = n === 0 ? (v & TMR_ENABLE[0]) !== 0 : t1Enable
      const internal = n === 0 || t1Internal
      if (load) {
        this.count[n] = this.t[n]
        this.preCount[n] = this.pre[n] >> 2
        if ((v & TOUT_MASK) === TOUT_T[n]) this.initTout()
      }
      if (enable) {
        if (internal && (load || !this.running[n])) this.start(n)
      } else this.stop(n)
    }
  }

  /** P31 (TIN) falling edge while T1 uses the external input. */
  tinEdge(): void {
    if (this.pre[1] & PRE1_INTERNAL) return
    switch (this.tmr & TIN_MASK) {
      case TIN_EXTERNAL_CLOCK:
        this.preCount[1] = (this.preCount[1] - 1) & 0x3f
        if (this.preCount[1] === 0) {
          this.preCount[1] = this.pre[1] >> 2
          if (this.tmr & TMR_ENABLE[1]) {
            this.count[1] = (this.count[1] - 1) & 0xff
            if (this.count[1] === 0) this.end(1)
          }
        }
        break
      case TIN_GATE:
        this.stop(1)
        break
      default: // trigger (ignored while running) / retrigger
        if ((this.tmr & TIN_MASK) === TIN_TRIGGER && this.running[1]) break
        if (this.tmr & TMR_ENABLE[1]) {
          this.count[1] = this.t[1]
          this.preCount[1] = this.pre[1] >> 2
          this.start(1)
        }
    }
  }

  /** P31 rising edge: gate mode resumes counting. */
  tinRise(): void {
    if (!(this.pre[1] & PRE1_INTERNAL) && (this.tmr & TIN_MASK) === TIN_GATE && this.tmr & TMR_ENABLE[1]) {
      this.start(1)
    }
  }

  get toutEnabled(): boolean {
    return (this.tmr & TOUT_MASK) !== TOUT_OFF
  }

  private initTout(): void {
    this.tout = true
    this.cpu.ports.updateP3Output()
  }

  private toggleTout(): void {
    this.tout = !this.tout
    this.cpu.ports.updateP3Output()
  }
}
