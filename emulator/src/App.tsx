import { useEffect, useState } from 'react'
import { Emulator } from './emu/runner'
import { CODE_CLR, CODE_DOWN, CODE_ENTER, CODE_LEFT } from './emu/keyboard'
import { Screen } from './components/Screen'
import { Keyboard } from './components/Keyboard'
import { ProgramPanel } from './components/ProgramPanel'
import { TapePanel } from './components/TapePanel'
import { MachinePanel } from './components/MachinePanel'
import { DebugPanel } from './components/DebugPanel'
import { DocViewer } from './components/DocViewer'
import { useEmuRefresh } from './hooks'
import { DOCS, DOCS_LOCAL, docUrl, type Doc } from './docs'

export default function App() {
  const [emu] = useState(() => new Emulator(2048))
  const [doc, setDoc] = useState<Doc | null>(null)

  // emulation loop, paced by the display
  useEffect(() => {
    let raf = 0
    let last = performance.now()
    const loop = (now: number) => {
      emu.tick(now - last)
      last = now
      raf = requestAnimationFrame(loop)
    }
    raf = requestAnimationFrame(loop)
    return () => cancelAnimationFrame(raf)
  }, [emu])

  // PC keyboard -> JU+TE keyboard
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      const t = e.target as HTMLElement | null
      if (t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.tagName === 'SELECT')) return
      if (document.querySelector('dialog[open]')) return // reading a document
      if (e.ctrlKey || e.metaKey || e.altKey) return
      const k = emu.keys
      let taps = null
      switch (e.key) {
        case 'Enter': taps = k.tapsForCode(CODE_ENTER, e.shiftKey); break // SHIFT+ENTER = OFF
        case 'Backspace': taps = k.tapsForCode(CODE_CLR); break
        case 'ArrowLeft': taps = k.tapsForCode(CODE_LEFT); break
        case 'ArrowRight': taps = k.tapsForCode(CODE_LEFT, true); break
        case 'ArrowDown': taps = k.tapsForCode(CODE_DOWN); break
        case 'ArrowUp': taps = k.tapsForCode(CODE_DOWN, true); break
        default:
          if (e.key.length === 1) taps = k.tapsForChar(e.key)
      }
      if (taps && taps.length) {
        e.preventDefault()
        emu.beeper.resume()
        emu.tap(...taps)
      }
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [emu])

  return (
    <div className="page">
      <header className="masthead">
        <div className="brand">
          <h1>
            JU<span className="plus">+</span>TE Tiny
          </h1>
          <p className="sub">
            The 1987 Jugend+Technik build-it-yourself computer: UB 8830 D, 2 KByte OS, TINY-MP-BASIC.
          </p>
        </div>
        <StatusBar emu={emu} />
      </header>

      <main className="layout">
        <div className="console">
          <Screen emu={emu} />
          <Keyboard emu={emu} />
          <div className="legend" aria-label="How the editor works">
            <p>
              <b className="cur-k">K</b> after the cursor: commands. <kbd>CLR</kbd> NEW · <kbd>−</kbd> LIST ·{' '}
              <kbd>+</kbd> RUN · <kbd>*</kbd> CONT · <kbd>SHIFT</kbd><kbd>P</kbd> SAVE · <kbd>SHIFT</kbd><kbd>O</kbd> LOAD.
              A letter shows that variable.
            </p>
            <p>
              Type a line number and <kbd>ENTER</kbd>: <b className="cur-a">A</b> after the cursor wants a keyword.
              One key each, green on the keys (P = PRINT, G = GOTO, F = IF, <kbd>,</kbd> = THEN). Close the line with{' '}
              <kbd>ENTER</kbd>. <kbd>SHIFT</kbd> works on the next key; ⇧ marks shifted keywords.
            </p>
            <p className="pc">
              Your PC keyboard types too: letters, digits, <kbd>Enter</kbd>, <kbd>Backspace</kbd> = CLR, arrows,{' '}
              <kbd>Shift</kbd>+<kbd>Enter</kbd> = OFF. Symbols like <kbd>"</kbd> and <kbd>;</kbd> are sent as SHIFT + key.
            </p>
          </div>
        </div>

        <aside className="side">
          <ProgramPanel emu={emu} />
          <TapePanel emu={emu} />
          <MachinePanel emu={emu} />
        </aside>
      </main>

      <DebugPanel emu={emu} />
      <Originals onOpen={setDoc} />
      <FurtherReading />
      <DocViewer doc={doc} onClose={() => setDoc(null)} />

      <footer className="credits">
        <p>
          Hardware and 2K system: Dr. Helmut Hoyer, <i>JU+TE Computer selbst gebaut</i>, Jugend+Technik 7/1987 – 2/1988.
          TINY-MP-BASIC: VEB Mikroelektronik Erfurt (UB 8830 D mask ROM, image via JTCEMU by Jens Müller).
          Z8 core modelled on the Zilog datasheet and MAME&apos;s z8 emulation.
        </p>
      </footer>
    </div>
  )
}

const READING: { title: string; href: string; what: string }[] = [
  { title: 'Original articles and documentation', href: 'https://hc-ddr.hucki.net/wiki/doku.php/tiny/dokumentation', what: 'J+T build series 1987–1991, ES 2.3 / ES 4.0, ABC Einchip-Mikrorechner (hc-ddr.hucki.net)' },
  { title: 'tiny-computer.de', href: 'https://www.tiny-computer.de/', what: 'Information and software for the Tiny' },
  { title: 'JU-TE-Computer on GitHub', href: 'https://github.com/boert/JU-TE-Computer', what: 'Schematics and firmware of the 2K and 6K Tiny' },
  { title: 'JTCEMU', href: 'https://hc-ddr.hucki.net/wiki/doku.php/tiny/jtcemu', what: 'The Java emulator by Jens Müller, source of the BASIC ROM image' },
  { title: 'Z8681 data sheet', href: 'http://bitsavers.informatik.uni-stuttgart.de/components/zilog/z8/Z8681_Data_Sheet_Mar83.pdf', what: 'Zilog, March 1983 (UB 88xx = Z8 family), bitsavers.org' },
  { title: 'Robotrontechnik forum thread', href: 'https://www.robotrontechnik.de/html/forum/thwb/showtopic.php?threadid=16374', what: 'Discussion about the JU+TE computer' },
  { title: 'RetroShield', href: 'http://8bitforce.com/projects/retroshield/', what: 'Arduino Mega shields that run real retro CPUs' },
  { title: 'A Z80 board with very few parts', href: 'https://hackaday.com/2020/07/17/a-z80-board-with-very-few-parts/', what: 'Hackaday: a real CPU driven by an Arduino' },
  { title: 'Mega-Compjuter', href: 'https://github.com/mariosgit/Mega-Compjuter', what: 'This project: an Arduino Mega shield for the UB 8830/8833, and this emulator' },
]

function Originals({ onOpen }: { onOpen: (d: Doc) => void }) {
  return (
    <section className="reading originals" aria-labelledby="orig-h">
      <h2 id="orig-h">The originals</h2>
      <p className="hint">
        The articles and booklets the Tiny was built from, as PDF (in German).
        {DOCS_LOCAL ? '' : ' They open on hc-ddr.hucki.net.'}
      </p>
      <ul>
        {DOCS.map((d) => (
          <li key={d.file}>
            {DOCS_LOCAL ? (
              <button type="button" className="doc-link" onClick={() => onOpen(d)}>
                {d.title}
              </button>
            ) : (
              <a href={docUrl(d)} target="_blank" rel="noopener noreferrer">
                {d.title}
              </a>
            )}
            <span className="doc-src">
              {d.source} · {d.pages} pages
            </span>
            <span>{d.summary}</span>
          </li>
        ))}
      </ul>
    </section>
  )
}

function FurtherReading() {
  return (
    <nav className="reading" aria-labelledby="reading-h">
      <h2 id="reading-h">Further reading</h2>
      <ul>
        {READING.map((r) => (
          <li key={r.href}>
            <a href={r.href} target="_blank" rel="noopener noreferrer">
              {r.title}
            </a>
            <span>{r.what}</span>
          </li>
        ))}
      </ul>
    </nav>
  )
}

function StatusBar({ emu }: { emu: Emulator }) {
  useEmuRefresh(emu, 4)
  const m = emu.machine
  const items: [string, string, string][] = [
    ['state', emu.paused ? 'Paused' : emu.typing ? `Typing · ${emu.player.pending}` : 'Running', emu.paused ? 'warn' : emu.typing ? 'busy' : 'ok'],
    ['picture', m.screenOn ? 'Picture on' : 'Picture off', m.screenOn ? 'ok' : 'off'],
    ['tape', emu.deck.isPlaying ? 'Tape playing' : emu.deck.isRecording ? 'Tape recording' : 'Tape idle', emu.deck.isPlaying || emu.deck.isRecording ? 'busy' : 'off'],
    ['ram', `${m.ramSize / 1024} KByte RAM`, 'off'],
  ]
  return (
    <ul className="status" aria-label="Machine status">
      {items.map(([k, label, tone]) => (
        <li key={k} className={`pill pill-${tone}`}>
          {label}
        </li>
      ))}
    </ul>
  )
}
