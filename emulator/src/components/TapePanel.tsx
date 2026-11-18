import { useEffect } from 'react'
import type { Emulator } from '../emu/runner'
import type { Tape } from '../emu/tape'
import { CPU_HZ } from '../emu/ub8830'
import { useEmuRefresh, useStoredState } from '../hooks'

/**
 * Virtual cassette recorder. Every SAVE is recorded automatically; to LOAD, press
 * SHIFT+O on the Tiny, then Play on a tape.
 */
export function TapePanel({ emu }: { emu: Emulator }) {
  useEmuRefresh(emu, 4)
  const [tapes, setTapes] = useStoredState<Tape[]>('jute-tiny.tapes', [])

  useEffect(() => {
    emu.onTapeRecorded = (t) => setTapes((old) => [t, ...old].slice(0, 20))
    return () => {
      emu.onTapeRecorded = null
    }
  }, [emu, setTapes])

  const deck = emu.deck
  const status = deck.isPlaying
    ? `Playing… ${Math.round(deck.progress * 100)} %`
    : deck.isRecording
      ? 'Recording…'
      : 'Stopped'

  const seconds = (t: Tape) => Math.round(t.segments.reduce((a, b) => a + b, 0) / CPU_HZ)

  return (
    <section className="panel" aria-labelledby="tape-h">
      <h2 id="tape-h">Cassette</h2>
      <p className="hint">
        <kbd>SHIFT</kbd> <kbd>P</kbd> = SAVE records a tape here (5 s leader, picture off meanwhile).
        To load: <kbd>SHIFT</kbd> <kbd>O</kbd> = LOAD, then press Play.
      </p>
      <div className="tape-status">
        <span className={`dot ${deck.isPlaying ? 'dot-play' : deck.isRecording ? 'dot-rec' : ''}`} aria-hidden="true" />
        {status}
        {deck.isPlaying && (
          <button type="button" className="btn btn-small" onClick={() => deck.stop()}>
            Stop
          </button>
        )}
      </div>
      {tapes.length === 0 ? (
        <p className="empty">No tapes yet. Type a program, then SAVE it.</p>
      ) : (
        <ul className="tapes">
          {tapes.map((t) => (
            <li key={t.id}>
              <input
                id={`tape-${t.id}`}
                className="tape-name"
                aria-label="Tape name"
                value={t.name}
                onChange={(e) => setTapes((old) => old.map((x) => (x.id === t.id ? { ...x, name: e.target.value } : x)))}
              />
              <span className="tape-len">{seconds(t)} s</span>
              <button type="button" className="btn btn-small" onClick={() => deck.play(t)} disabled={deck.isPlaying}>
                Play
              </button>
              <button
                type="button"
                className="btn btn-small btn-quiet"
                aria-label={`Delete ${t.name}`}
                onClick={() => setTapes((old) => old.filter((x) => x.id !== t.id))}
              >
                ✕
              </button>
            </li>
          ))}
        </ul>
      )}
    </section>
  )
}
