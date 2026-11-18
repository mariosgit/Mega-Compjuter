/**
 * Real-time driver: runs the machine in step with the browser's animation
 * frames, feeds queued keystrokes, the cassette and the speaker.
 * Pure TS (no React) so it can also be driven from tests.
 */
import { JuTe, type RamSize } from './jtc'
import { KeyMapper, KeyPlayer, type Tap } from './keyboard'
import { TapeDeck, type Tape } from './tape'
import { Beeper } from './audio'
import { CPU_HZ } from './ub8830'
import { listingToKeys } from './basic'

/** 1, 2, 4 = times real speed; 0 = as fast as the browser allows. */
export type Speed = 0 | 1 | 2 | 4

const SLICE = 0.004 // seconds of emulated time between keyboard/tape updates
const TYPING_SPEEDUP = 10 // queued keystrokes play this much faster than real time

export class Emulator {
  readonly machine: JuTe
  readonly keys: KeyMapper
  readonly player: KeyPlayer
  readonly deck: TapeDeck
  readonly beeper = new Beeper()

  speed: Speed = 1
  paused = false
  /** Measured emulation speed relative to the real machine. */
  actualSpeed = 1
  onTapeRecorded: ((t: Tape) => void) | null = null

  private listeners = new Set<() => void>()
  private speakerLevel = false

  constructor(ramSize: RamSize = 2048) {
    this.machine = new JuTe(ramSize)
    this.keys = new KeyMapper((a) => this.machine.peek(a))
    this.player = new KeyPlayer(this.machine)
    this.deck = new TapeDeck(this.machine)
    this.deck.onRecorded = (t) => this.onTapeRecorded?.(t)
    this.machine.trace = []
  }

  subscribe(fn: () => void): () => void {
    this.listeners.add(fn)
    return () => this.listeners.delete(fn)
  }

  notify(): void {
    for (const fn of this.listeners) fn()
  }

  // ---- controls ---------------------------------------------------------------

  powerOn(): void {
    this.player.clear()
    this.deck.stop()
    this.machine.powerOn()
    this.notify()
  }

  /** The RESET key: restarts the OS, the BASIC program in RAM stays. */
  reset(): void {
    this.player.clear()
    this.machine.reset()
    this.notify()
  }

  setRamSize(size: RamSize): void {
    this.machine.setRamSize(size)
    this.powerOn()
  }

  /** Single instruction (debugger). */
  step(): void {
    this.machine.cpu.step()
    this.notify()
  }

  tap(...taps: Tap[]): void {
    this.player.push(...taps)
  }

  typeText(text: string): void {
    for (const ch of text) this.player.push(...this.keys.tapsForChar(ch))
  }

  /** Type a BASIC listing into the editor (NEW first if requested). */
  typeListing(listing: string, clearFirst: boolean): string[] {
    const { strokes, errors } = listingToKeys(listing)
    if (clearFirst) this.player.push(...this.keys.tapsForCode(0x08).map((t) => ({ ...t, pause: 0.3 }))) // NEW
    for (const s of strokes) this.player.push(...this.keys.tapsForChar(s))
    return errors
  }

  get typing(): boolean {
    return this.player.pending > 0
  }

  // ---- main loop ----------------------------------------------------------------

  /** Advance by `dtMs` of real time (call from requestAnimationFrame). */
  tick(dtMs: number): void {
    if (this.paused) return
    const dt = Math.min(dtMs, 50) / 1000
    const m = this.machine
    const startCycle = m.cpu.totalCycles
    const wallStart = performance.now()

    const fast = this.speed === 0 || this.typing
    if (fast) {
      // as fast as possible within ~70% of the frame, typing capped at TYPING_SPEEDUP
      const budgetMs = Math.max(4, dtMs * 0.7)
      const cap = this.speed === 0 ? Infinity : dt * Math.max(this.speed, TYPING_SPEEDUP) * CPU_HZ
      while (performance.now() - wallStart < budgetMs && m.cpu.totalCycles - startCycle < cap) {
        this.slice()
        if (this.speed !== 0 && !this.typing) break
      }
    } else {
      const target = startCycle + dt * this.speed * CPU_HZ
      while (m.cpu.totalCycles < target) this.slice()
    }

    const ran = m.cpu.totalCycles - startCycle
    this.actualSpeed = this.actualSpeed * 0.9 + (ran / CPU_HZ / dt) * 0.1

    // sound only makes sense at real speed
    const edges = m.speakerEdges
    if (!fast && this.speed === 1) this.beeper.render(edges, this.speakerLevel, startCycle, m.cpu.totalCycles)
    this.speakerLevel = m.speaker
    edges.length = 0

    this.notify()
  }

  private slice(): void {
    this.player.update()
    this.machine.cpu.run(SLICE * CPU_HZ)
    this.deck.update()
  }
}
