/**
 * Speaker: turns the cycle-stamped P36 edges of each emulated slice into a
 * square wave and queues it on a WebAudio context.
 */
import { CPU_HZ } from './ub8830'

export class Beeper {
  private ctx: AudioContext | null = null
  private nextTime = 0
  private hp = 0 // high-pass state (removes the DC level of an idle speaker)
  private last = 0
  enabled = true
  volume = 0.12

  /** Must be called from a user gesture (browser autoplay rules). */
  resume(): void {
    try {
      if (!this.ctx) this.ctx = new AudioContext()
      if (this.ctx.state === 'suspended') void this.ctx.resume()
    } catch {
      this.ctx = null
    }
  }

  /**
   * Render one slice of emulated time.
   * @param edges   cycle times of level changes within [start, end)
   * @param level   speaker level at `start`
   */
  render(edges: number[], level: boolean, start: number, end: number): void {
    const ctx = this.ctx
    if (!ctx || !this.enabled || ctx.state !== 'running' || end <= start) return
    const rate = ctx.sampleRate
    const n = Math.max(1, Math.round(((end - start) / CPU_HZ) * rate))
    const buf = ctx.createBuffer(1, n, rate)
    const out = buf.getChannelData(0)
    let e = 0
    let lv = level
    const cyclesPerSample = (end - start) / n
    for (let i = 0; i < n; i++) {
      const t = start + i * cyclesPerSample
      while (e < edges.length && edges[e] <= t) {
        lv = !lv
        e++
      }
      const x = lv ? this.volume : -this.volume
      // one-pole high-pass ~20 Hz
      this.hp = 0.997 * (this.hp + x - this.last)
      this.last = x
      out[i] = this.hp
    }
    const src = ctx.createBufferSource()
    src.buffer = buf
    src.connect(ctx.destination)
    const now = ctx.currentTime
    if (this.nextTime < now + 0.02 || this.nextTime > now + 0.25) this.nextTime = now + 0.05
    src.start(this.nextTime)
    this.nextTime += n / rate
  }
}
