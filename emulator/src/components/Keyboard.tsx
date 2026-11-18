import type { Emulator } from '../emu/runner'
import { CODE_SHIFT } from '../emu/keyboard'

/**
 * On-screen keyboard laid out like the 4-row JU+TE keyboard (Abb. 16, J+T 11/1987):
 * main character, shifted character, BASIC keyword (A cursor) and command (K cursor).
 */
interface KeyDef {
  code: number
  label: string
  shift?: string
  /** keyword entered with this key when the editor shows the A cursor */
  kw?: string
  /** command when the editor shows the K cursor */
  cmd?: string
  wide?: boolean
}

const ROWS: KeyDef[][] = [
  [
    { code: 0x0a, label: '↓', shift: '↑' },
    { code: 0x31, label: '1', shift: '!', kw: 'TRAP⇧' },
    { code: 0x32, label: '2', shift: '"' },
    { code: 0x33, label: '3', shift: '#' },
    { code: 0x34, label: '4', shift: '¤' },
    { code: 0x35, label: '5', shift: '%' },
    { code: 0x36, label: '6', shift: '&' },
    { code: 0x37, label: '7', shift: "'" },
    { code: 0x38, label: '8', shift: '(' },
    { code: 0x39, label: '9', shift: ')' },
    { code: 0x30, label: '0', shift: '␣' },
    { code: 0x08, label: 'CLR', cmd: 'NEW' },
  ],
  [
    { code: 0x0b, label: '←', shift: '→' },
    { code: 0x51, label: 'Q' },
    { code: 0x57, label: 'W', kw: 'WAIT' },
    { code: 0x45, label: 'E', kw: 'END' },
    { code: 0x52, label: 'R', kw: 'RETURN' },
    { code: 0x54, label: 'T', kw: 'STOP' },
    { code: 0x5a, label: 'Z' },
    { code: 0x55, label: 'U' },
    { code: 0x49, label: 'I', kw: 'INPUT' },
    { code: 0x4f, label: 'O', shift: '_', kw: 'PROC', cmd: 'LOAD⇧' },
    { code: 0x50, label: 'P', shift: '@', kw: 'PRINT', cmd: 'SAVE⇧' },
    { code: 0x2b, label: '+', shift: ';', cmd: 'RUN' },
  ],
  [
    { code: CODE_SHIFT, label: 'SHIFT' },
    { code: 0x41, label: 'A' },
    { code: 0x53, label: 'S', kw: 'GOSUB' },
    { code: 0x44, label: 'D' },
    { code: 0x46, label: 'F', kw: 'IF' },
    { code: 0x47, label: 'G', kw: 'GOTO' },
    { code: 0x48, label: 'H', kw: 'PTH' },
    { code: 0x4a, label: 'J' },
    { code: 0x4b, label: 'K', shift: '[' },
    { code: 0x4c, label: 'L', kw: 'LET' },
    { code: 0x2a, label: '*', shift: ':', cmd: 'CONT' },
    { code: 0x2d, label: '−', shift: '=', cmd: 'LIST' },
  ],
  [
    { code: 0x20, label: 'SPACE' },
    { code: 0x59, label: 'Y' },
    { code: 0x58, label: 'X' },
    { code: 0x43, label: 'C', kw: 'CALL' },
    { code: 0x56, label: 'V' },
    { code: 0x42, label: 'B' },
    { code: 0x4e, label: 'N', shift: '\\' },
    { code: 0x4d, label: 'M', shift: ']', kw: 'REM' },
    { code: 0x2c, label: ',', shift: '<', kw: 'THEN' },
    { code: 0x2e, label: '.', shift: '>', kw: 'ELSE⇧' },
    { code: 0x2f, label: '/', shift: '?', kw: 'TOFF' },
    { code: 0x0d, label: 'ENTER', shift: 'OFF' },
  ],
]

export function Keyboard({ emu }: { emu: Emulator }) {
  const press = (k: KeyDef) => {
    emu.beeper.resume()
    emu.tap(...emu.keys.tapsForCode(k.code))
  }
  return (
    <div className="keyboard" role="group" aria-label="JU+TE keyboard">
      {ROWS.map((row, r) => (
        <div className={`kb-row kb-row-${r}`} key={r}>
          {row.map((k) => (
            <button
              type="button"
              key={k.code}
              className={`key${k.label.length > 2 ? ' key-word' : ''}${k.code === CODE_SHIFT ? ' key-shift' : ''}`}
              onPointerDown={(e) => {
                e.preventDefault()
                press(k)
              }}
              onKeyDown={(e) => {
                if (e.key === ' ' || e.key === 'Enter') {
                  e.preventDefault()
                  e.stopPropagation()
                  press(k)
                }
              }}
              title={[k.cmd && `K: ${k.cmd}`, k.kw && `A: ${k.kw}`, k.shift && `SHIFT: ${k.shift}`].filter(Boolean).join(' · ')}
            >
              <span className="key-top">
                <span className="key-shift-ch">{k.shift ?? ''}</span>
                {k.cmd && <span className="key-cmd">{k.cmd}</span>}
              </span>
              <span className="key-main">{k.label}</span>
              <span className="key-kw">{k.kw ?? ''}</span>
            </button>
          ))}
        </div>
      ))}
    </div>
  )
}
