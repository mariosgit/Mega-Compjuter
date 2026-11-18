import { useState } from 'react'
import type { Emulator } from '../emu/runner'
import { programToText } from '../emu/basic'
import { EXAMPLES } from '../examples'
import { useEmuRefresh } from '../hooks'

/** Type BASIC listings into the editor and read the program back from RAM. */
export function ProgramPanel({ emu }: { emu: Emulator }) {
  useEmuRefresh(emu, 4)
  const [text, setText] = useState(EXAMPLES[4].listing)
  const [clearFirst, setClearFirst] = useState(true)
  const [message, setMessage] = useState<string | null>(null)

  const typing = emu.typing
  const remaining = emu.player.pending

  const type = () => {
    emu.beeper.resume()
    const errors = emu.typeListing(text, clearFirst)
    setMessage(errors.length ? `Typed with warnings: ${errors.join('; ')}` : null)
  }

  const load = () => {
    const r = emu.loadListing(text)
    const parts = [`Loaded ${r.bytes} bytes at E000h. Press + (RUN).`]
    if (r.switchedRam) parts.push('RAM switched to 8 KByte, the program did not fit.')
    if (r.errors.length) parts.push(`Warnings: ${r.errors.join('; ')}`)
    setMessage(parts.join(' '))
  }

  const readBack = () => {
    const listing = programToText((a) => emu.machine.peek(a))
    if (listing) {
      setText(listing)
      setMessage('Listing read from RAM at E000h.')
    } else setMessage('No BASIC program in RAM yet.')
  }

  return (
    <section className="panel" aria-labelledby="prog-h">
      <h2 id="prog-h">Programs</h2>
      <p className="hint">
        <i>Type</i> enters the listing key by key, as on the real keyboard (line number, ENTER, one key per
        keyword). <i>Load</i> puts it straight into RAM, as from tape. Then press <kbd>+</kbd> (RUN).
      </p>
      <div className="chips" role="list" aria-label="Examples from Jugend+Technik 1987–1989">
        {EXAMPLES.map((ex) => (
          <button
            type="button"
            key={ex.name}
            role="listitem"
            className="chip"
            title={ex.note}
            onClick={() => {
              setText(ex.listing)
              setMessage(ex.note)
            }}
          >
            {ex.name}
          </button>
        ))}
      </div>
      <label className="sr-only" htmlFor="listing">
        BASIC listing
      </label>
      <textarea
        id="listing"
        className="listing"
        spellCheck={false}
        value={text}
        onChange={(e) => setText(e.target.value)}
        rows={10}
      />
      <div className="row">
        {typing ? (
          <button type="button" className="btn" onClick={() => emu.player.clear()}>
            Stop typing ({remaining} keys left)
          </button>
        ) : (
          <button type="button" className="btn btn-primary" onClick={type}>
            Type
          </button>
        )}
        <button type="button" className="btn" onClick={load} disabled={typing}>
          Load into RAM
        </button>
        <button type="button" className="btn" onClick={readBack} disabled={typing}>
          Read from RAM
        </button>
        <label className="check">
          <input id="clear-first" type="checkbox" checked={clearFirst} onChange={(e) => setClearFirst(e.target.checked)} />
          NEW first
        </label>
      </div>
      {message && <p className="note">{message}</p>}
    </section>
  )
}
