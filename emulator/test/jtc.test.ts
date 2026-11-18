import { describe, expect, test } from 'vitest'
import { JuTe } from '../src/emu/jtc'
import { KeyMapper, KeyPlayer } from '../src/emu/keyboard'
import { listingToBytes, listingToKeys, programToText } from '../src/emu/basic'
import { Emulator } from '../src/emu/runner'
import { TapeDeck, type Tape } from '../src/emu/tape'
import { EXAMPLES } from '../src/examples'

/** Glyphs of the 2K OS font (4x5 in a 5x8 cell), read from the ROM's output. */
function screenRows(m: JuTe): string[] {
  return m.screenAscii().split('\n')
}

function boot(ram: 1024 | 2048 = 2048) {
  const m = new JuTe(ram)
  const km = new KeyMapper((a) => m.peek(a))
  const kp = new KeyPlayer(m)
  kp.run(1)
  const type = (s: string) => { for (const ch of s) kp.push(...km.tapsForChar(ch)) }
  const finish = (extra = 1) => { while (kp.pending) kp.run(0.5); kp.run(extra) }
  return { m, km, kp, type, finish }
}

describe('JU+TE machine', () => {
  test('boots to the editor with the picture on', () => {
    const { m } = boot()
    expect(m.screenOn).toBe(true)
    expect(m.cpu.lastIllegal).toBe(-1)
    // the "COMP JU+TE R" banner occupies text line 1 (pixel rows 7-11)
    const banner = screenRows(m).slice(7, 12).join('\n')
    expect(banner).toContain('.###..##..#..#.###')
    expect(m.cpu.pc).toBeGreaterThanOrEqual(0x0800)
  })

  test('bus trace: RAM probe at 0812h, then JP to the OS', () => {
    const m = new JuTe(2048)
    m.trace = []
    m.powerOn()
    m.runSeconds(0.01)
    const t = m.trace.slice(0, 7).map((b) => `${b.write ? 'W' : 'R'}${b.addr.toString(16)}=${b.data.toString(16)}`)
    expect(t).toEqual(['R812=8d', 'W812=72', 'R812=8d', 'W812=8d', 'R812=8d', 'R813=8', 'R814=ff'])
  })

  test('typed program runs (PRINT, END)', () => {
    const { m, km, kp, type, finish } = boot()
    kp.push(...km.tapsForCode(0x08)) // NEW
    for (const s of listingToKeys('10 PRINT "HALLO";END').strokes) type(s)
    finish()
    expect(programToText((a) => m.peek(a))).toBe('10 PRINT "HALLO";END')
    kp.push(...km.tapsForCode(0x2b)) // RUN
    finish(2)
    const H = '#..#.' // first glyph column pattern of H
    expect(screenRows(m).some((r) => r.includes(H))).toBe(true)
  })

  test('Mondlandung listing types in exactly', () => {
    const { m, km, kp, type, finish } = boot()
    kp.push(...km.tapsForCode(0x08))
    const ex = EXAMPLES.find((e) => e.name === 'Mondlandung')!
    const { strokes, errors } = listingToKeys(ex.listing)
    expect(errors).toEqual([])
    for (const s of strokes) type(s)
    finish()
    const norm = (s: string | null) => (s ?? '').replace(/\s+/g, '')
    expect(norm(programToText((a) => m.peek(a)))).toBe(norm(ex.listing))
  })

  test('SAVE to the virtual tape and LOAD it back', () => {
    const { m, km, kp, type } = boot()
    const deck = new TapeDeck(m)
    let tape: Tape | null = null
    deck.onRecorded = (t) => { tape = t }
    const run = (s: number) => { for (let i = 0; i < s / 0.01; i++) { kp.run(0.01); deck.update() } }
    type('10\nP"HI";E\n')
    run(6)
    const before = Array.from({ length: 12 }, (_, i) => m.peek(0xe000 + i))
    kp.push(...km.tapsForCode(0x50, true)) // SAVE
    run(10)
    expect(tape).not.toBeNull()
    kp.push(...km.tapsForCode(0x08)) // NEW
    run(2)
    expect(m.peek(0xe000)).toBe(0)
    kp.push(...km.tapsForCode(0x4f, true)) // LOAD
    run(1)
    deck.play(tape!)
    run(10)
    expect(Array.from({ length: 12 }, (_, i) => m.peek(0xe000 + i))).toEqual(before)
  })

  test('Turm von Hanoi: loads into RAM (switching to 8 KByte), runs, moves a stone', () => {
    const emu = new Emulator(2048)
    emu.machine.runSeconds(1)
    const ex = EXAMPLES.find((e) => e.name === 'Turm von Hanoi')!
    const r = emu.loadListing(ex.listing)
    expect(r.errors).toEqual([])
    expect(r.switchedRam).toBe(true)
    expect(emu.machine.ramSize).toBe(8192)
    const norm = (s: string | null) => (s ?? '').replace(/\s+/g, '')
    expect(norm(programToText((a) => emu.machine.peek(a)))).toBe(norm(ex.listing))
    expect(listingToBytes(ex.listing).bytes.length).toBe(r.bytes)

    const kp = new KeyPlayer(emu.machine)
    kp.push(...emu.keys.tapsForCode(0x2b)) // RUN
    kp.run(6)
    // smallest stone (3 px wide, at x 6-8) sits on top of the left tower, right stand empty
    const rows = () => emu.machine.screenAscii().split('\n')
    expect(rows()[43].slice(0, 12)).toBe('......###...')
    expect(rows()[59].slice(36, 44)).toBe('........')
    // the BASIC program polls the key with GTC, so give it time between the two inputs
    kp.push(...emu.keys.tapsForChar('1'))
    kp.run(1)
    kp.push(...emu.keys.tapsForChar('3'))
    kp.run(3)
    expect(rows()[43].slice(0, 12)).toBe('............')
    expect(rows()[59].slice(36, 44)).toBe('..###...')
  })
})
