import type { Emulator, Speed } from '../emu/runner'
import { RAM_SIZES, type RamSize } from '../emu/jtc'
import { useEmuRefresh } from '../hooks'

const SPEEDS: { v: Speed; label: string }[] = [
  { v: 1, label: '1×' },
  { v: 2, label: '2×' },
  { v: 4, label: '4×' },
  { v: 0, label: 'Max' },
]

/** Power, RESET, speed, memory size and sound. */
export function MachinePanel({ emu }: { emu: Emulator }) {
  useEmuRefresh(emu, 4)
  const m = emu.machine

  return (
    <section className="panel" aria-labelledby="machine-h">
      <h2 id="machine-h">Machine</h2>
      <div className="row">
        <button type="button" className="btn" onClick={() => emu.reset()} title="RESET key: pulls /RESET (pin 6) low. The BASIC program stays in RAM.">
          RESET
        </button>
        <button type="button" className="btn" onClick={() => emu.powerOn()} title="Switch off and on: RAM is cleared.">
          Power cycle
        </button>
        <button type="button" className="btn" onClick={() => { emu.paused = !emu.paused; emu.notify() }} aria-pressed={emu.paused}>
          {emu.paused ? 'Resume' : 'Pause'}
        </button>
      </div>

      <div className="field">
        <span className="field-label" id="speed-l">Speed</span>
        <div className="segmented" role="radiogroup" aria-labelledby="speed-l">
          {SPEEDS.map((s) => (
            <button
              type="button"
              key={s.v}
              role="radio"
              aria-checked={emu.speed === s.v}
              className={emu.speed === s.v ? 'on' : ''}
              onClick={() => { emu.speed = s.v; emu.notify() }}
            >
              {s.label}
            </button>
          ))}
        </div>
        <span className="field-value">{emu.paused ? 'paused' : `${emu.actualSpeed.toFixed(1)}× real`}</span>
      </div>

      <div className="field">
        <label className="field-label" htmlFor="ram">RAM</label>
        <select id="ram" value={m.ramSize} onChange={(e) => emu.setRamSize(Number(e.target.value) as RamSize)}>
          {RAM_SIZES.map((s) => (
            <option key={s} value={s}>
              {s / 1024} KByte {s === 1024 ? '(minimum, 2× U214)' : s <= 8192 ? 'at E000h' : 'at 8000h'}
            </option>
          ))}
        </select>
      </div>

      <div className="field">
        <label className="check">
          <input
            id="sound"
            type="checkbox"
            checked={emu.beeper.enabled}
            onChange={(e) => {
              emu.beeper.enabled = e.target.checked
              if (e.target.checked) emu.beeper.resume()
              emu.notify()
            }}
          />
          Sound (P36, 1× only)
        </label>
      </div>

      <dl className="facts">
        <div><dt>CPU</dt><dd>UB 8830 D, 8 MHz crystal</dd></div>
        <div><dt>Picture</dt><dd>{m.screenOn ? 'on' : 'off (OS busy)'}</dd></div>
        <div><dt>Run time</dt><dd>{(m.cpu.totalCycles / 4e6).toFixed(1)} s</dd></div>
      </dl>
    </section>
  )
}
