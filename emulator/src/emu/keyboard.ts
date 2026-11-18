/**
 * JU+TE keyboard: a 4 x 15 contact matrix. The OS writes the column number to the
 * low address lines (A0-A3 -> 74154 decoder) and reads the rows S0-S3 on D4-D7 at
 * 6000h-7FFFh. The OS translates (column,row) into a code with the table at
 * 0F00h-0F3Fh of the EPROM (row-major, 16 entries per row), so the key layout is
 * taken from the ROM itself.
 *
 * SHIFT (code 15h) is a prefix key: the OS remembers it and applies it to the next
 * key, so a shifted character is typed as two taps.
 */
import type { JuTe } from './jtc'
import { CPU_HZ } from './ub8830'

export interface KeyPos {
  col: number
  row: number
}

export const KEY_TABLE_ADDR = 0x0f00
/** After ENTER the editor sorts the line into the program; keys typed meanwhile get lost. */
export const ENTER_PAUSE = 0.3

export const CODE_SHIFT = 0x15
export const CODE_ENTER = 0x0d
export const CODE_CLR = 0x08
export const CODE_SPACE = 0x20
export const CODE_LEFT = 0x0b
export const CODE_DOWN = 0x0a

/** Read the 4x16 key table from the OS ROM: code -> matrix position. */
export function readKeyTable(peek: (a: number) => number): Map<number, KeyPos> {
  const map = new Map<number, KeyPos>()
  for (let row = 0; row < 4; row++) {
    for (let col = 1; col < 16; col++) {
      const code = peek(KEY_TABLE_ADDR + row * 16 + col)
      if (code && !map.has(code)) map.set(code, { col, row })
    }
  }
  return map
}

/** A tap = keys held together for a moment, then released. */
export interface Tap {
  keys: KeyPos[]
  /** Optional label for UIs. */
  label?: string
  /** Extra pause after release, in seconds (e.g. the editor sorting in a line after ENTER). */
  pause?: number
}

/**
 * Plays taps into the keyboard matrix in emulated time. Hold and gap are long
 * enough for the OS's scan, debounce and key-click beep (the OS beeps for ~80 ms
 * after each key and misses keys pressed meanwhile).
 */
export class KeyPlayer {
  private queue: Tap[] = []
  private phase: 'idle' | 'down' | 'up' = 'idle'
  private until = 0
  /** Measured: ~0.22 s per keystroke is reliable; shorter taps get lost now and then. */
  holdSeconds = 0.1
  gapSeconds = 0.12
  private currentPause = 0

  constructor(private readonly machine: JuTe) {}

  get pending(): number {
    return this.queue.length + (this.phase === 'idle' ? 0 : 1)
  }

  push(...taps: Tap[]): void {
    this.queue.push(...taps)
  }

  clear(): void {
    this.queue = []
    this.phase = 'idle'
    this.machine.keyCols.fill(0)
  }

  /** Call regularly (e.g. every few ms of emulated time). */
  update(): void {
    const now = this.machine.cpu.totalCycles
    const cols = this.machine.keyCols
    if (this.phase === 'down' && now >= this.until) {
      cols.fill(0)
      this.phase = 'up'
      this.until = now + (this.gapSeconds + this.currentPause) * CPU_HZ
    }
    if (this.phase === 'up' && now >= this.until) this.phase = 'idle'
    if (this.phase === 'idle' && this.queue.length) {
      const tap = this.queue.shift()!
      this.currentPause = tap.pause ?? 0
      cols.fill(0)
      for (const k of tap.keys) cols[k.col] |= 1 << k.row
      this.phase = 'down'
      this.until = now + this.holdSeconds * CPU_HZ
    }
  }

  /** Run the machine for `seconds`, feeding queued taps on the way. */
  run(seconds: number, slice = 0.005): void {
    const end = this.machine.cpu.totalCycles + seconds * CPU_HZ
    while (this.machine.cpu.totalCycles < end) {
      this.update()
      this.machine.cpu.run(slice * CPU_HZ)
    }
  }
}

/**
 * Character -> taps. Uses the unshifted key table from the ROM and, for the
 * second level, the shift layer of the 2K system (J+T 1987, as in JTCEMU).
 */
const SHIFT_LAYER: Record<string, string> = {
  '!': '1', '"': '2', '#': '3', '$': '4', '%': '5', '&': '6', "'": '7', '(': '8', ')': '9',
  '\\': 'N', ']': 'M', '<': ',', '>': '.', '?': '/',
  '[': 'K', ':': '*', '=': '-',
  '@': 'P', ';': '+', '_': 'O',
}

export class KeyMapper {
  readonly table: Map<number, KeyPos>
  readonly shift: KeyPos

  constructor(peek: (a: number) => number) {
    this.table = readKeyTable(peek)
    this.shift = this.table.get(CODE_SHIFT) ?? { col: 1, row: 1 }
  }

  pos(code: number): KeyPos | undefined {
    return this.table.get(code)
  }

  /** Taps for a code (plain key), optionally preceded by SHIFT. */
  tapsForCode(code: number, shifted = false): Tap[] {
    const p = this.table.get(code)
    if (!p) return []
    return shifted ? [{ keys: [this.shift], label: 'SHIFT' }, { keys: [p] }] : [{ keys: [p] }]
  }

  /** Taps to produce a character as typed on a PC keyboard. */
  tapsForChar(ch: string): Tap[] {
    if (ch === '\n' || ch === '\r') return this.tapsForCode(CODE_ENTER).map((t) => ({ ...t, pause: ENTER_PAUSE }))
    const up = ch.toUpperCase()
    const code = up.charCodeAt(0)
    if (this.table.has(code)) return this.tapsForCode(code)
    const base = SHIFT_LAYER[ch]
    if (base) return this.tapsForCode(base.charCodeAt(0), true)
    return []
  }
}
