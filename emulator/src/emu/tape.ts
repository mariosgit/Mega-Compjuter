/**
 * Virtual cassette recorder for the 2K OS.
 *
 * SAVE writes a ~5 kHz tone on P36 (with P37 low) and keys it on and off; the
 * recorder's output is rectified so that P30 = 1 means "tone present" (on-off
 * keying, see J+T 11/1987 "Magnetband-Anschluss"). The OS starts with a 5 s leader.
 *
 * Recording: tone envelope from the edges of the cassette output, stored as
 * alternating durations in CPU cycles [on, off, on, off, ...].
 * Playback: P30 follows that envelope, evaluated at the exact cycle the CPU reads it.
 */
import type { JuTe } from './jtc'
import { CPU_HZ } from './ub8830'

/** Edges closer than this belong to the same tone burst (tone half period ~400). */
const TONE_GAP = 1200
/** Silence that ends an automatic recording. */
const END_SILENCE = CPU_HZ * 1.5

export interface Tape {
  id: string
  name: string
  created: number
  /** Alternating on/off durations in cycles, starting with "on". */
  segments: number[]
}

/**
 * Make a raw envelope tape-like: start at the leader (drops key clicks before it)
 * and give every burst its last half wave back (the envelope is measured from
 * edge to edge). Returns null if there is no leader (not a SAVE).
 */
function cleanUp(raw: number[], halfWave: number): number[] | null {
  let lead = -1
  for (let i = 0; i < raw.length; i += 2) {
    if (raw[i] > CPU_HZ) { lead = i; break }
  }
  if (lead < 0) return null
  const out = raw.slice(lead)
  const hw = Math.round(halfWave)
  for (let i = 0; i < out.length; i += 2) {
    out[i] += hw
    if (i + 1 < out.length) out[i + 1] = Math.max(1, out[i + 1] - hw)
  }
  return out
}

export class TapeDeck {
  /** Called when an automatic recording is complete. */
  onRecorded: ((tape: Tape) => void) | null = null
  /** Called when playback ends. */
  onPlaybackEnd: (() => void) | null = null

  private edgeIndex = 0
  private rec: { segments: number[]; burstStart: number; lastEdge: number; gapSum: number; gapN: number } | null = null

  private playing: { segments: number[]; start: number; idx: number; segEnd: number; on: boolean } | null = null

  constructor(private readonly machine: JuTe) {
    machine.tapeSource = (cycle) => this.levelAt(cycle)
  }

  /** True once a SAVE leader tone is being recorded (key clicks alone don't count). */
  get isRecording(): boolean {
    const r = this.rec
    if (!r) return false
    const leader = CPU_HZ / 2
    if (r.lastEdge - r.burstStart > leader) return true
    for (let i = 0; i < r.segments.length; i += 2) if (r.segments[i] > leader) return true
    return false
  }

  get isPlaying(): boolean {
    return this.playing !== null
  }

  /** Playback progress 0..1. */
  get progress(): number {
    const p = this.playing
    if (!p) return 0
    const total = p.segments.reduce((a, b) => a + b, 0)
    return Math.min(1, (this.machine.cpu.totalCycles - p.start) / total)
  }

  play(tape: Tape): void {
    const now = this.machine.cpu.totalCycles
    this.playing = { segments: tape.segments, start: now, idx: 0, segEnd: now + (tape.segments[0] ?? 0), on: true }
  }

  stop(): void {
    this.playing = null
    this.machine.tapeIn = false
  }

  private levelAt(cycle: number): boolean {
    const p = this.playing
    if (!p) return false
    while (cycle >= p.segEnd) {
      p.idx++
      if (p.idx >= p.segments.length) {
        this.playing = null
        this.onPlaybackEnd?.()
        return false
      }
      p.on = !p.on
      p.segEnd += p.segments[p.idx]
    }
    return p.on
  }

  /** Call regularly: turns new cassette-output edges into a recording. */
  update(): void {
    const m = this.machine
    const edges = m.tapeOutEdges
    for (; this.edgeIndex < edges.length; this.edgeIndex++) {
      const t = edges[this.edgeIndex]
      const r = this.rec
      if (!r) {
        this.rec = { segments: [], burstStart: t, lastEdge: t, gapSum: 0, gapN: 0 }
        continue
      }
      const gap = t - r.lastEdge
      if (gap > TONE_GAP) {
        // previous burst ended at lastEdge (+ its last half wave); silence until t
        r.segments.push(r.lastEdge - r.burstStart, gap)
        r.burstStart = t
      } else {
        r.gapSum += gap
        r.gapN++
      }
      r.lastEdge = t
    }
    // don't let the edge list grow forever
    if (this.edgeIndex > 100_000) {
      edges.splice(0, this.edgeIndex)
      this.edgeIndex = 0
    }
    const r = this.rec
    if (r && m.cpu.totalCycles - r.lastEdge > END_SILENCE) {
      r.segments.push(r.lastEdge - r.burstStart)
      this.rec = null
      const segments = cleanUp(r.segments, r.gapN ? r.gapSum / r.gapN : 0)
      if (segments) {
        const created = Date.now()
        this.onRecorded?.({
          id: created.toString(36),
          name: 'Tape ' + new Date(created).toLocaleString(),
          created,
          segments,
        })
      }
    }
  }
}
