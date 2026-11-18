import { useState } from 'react'
import type { Emulator } from '../emu/runner'
import { disassemble } from '../emu/z8disasm'
import { ACC_NAMES, F_C, F_D, F_H, F_S, F_V, F_Z } from '../emu/z8defs'
import { hex2, hex4, useEmuRefresh } from '../hooks'

const FLAG_BITS: [number, string][] = [[F_C, 'C'], [F_Z, 'Z'], [F_S, 'S'], [F_V, 'V'], [F_D, 'D'], [F_H, 'H']]

/**
 * CPU state, disassembly at PC and the external bus trace after the last reset.
 * The trace lists every /AS-/DS cycle the way the Arduino shield sees them on the
 * real chip, so it can be compared with hardware logs.
 */
export function DebugPanel({ emu }: { emu: Emulator }) {
  useEmuRefresh(emu, emu.paused ? 30 : 4)
  const [traceRows, setTraceRows] = useState(64)
  const [copied, setCopied] = useState(false)
  const cpu = emu.machine.cpu
  const peek = (a: number) => emu.machine.peek(a)

  const regs: [string, string][] = [
    ['PC', hex4(cpu.pc)],
    ['SP', hex4(cpu.sp)],
    ['RP', hex2(cpu.rp)],
    ['FLAGS', FLAG_BITS.map(([b, n]) => (cpu.flags & b ? n : '·')).join('')],
    ['IMR', hex2(cpu.imr)],
    ['IRQ', hex2(cpu.irq)],
    ['IPR', hex2(cpu.ipr)],
    ['P01M', hex2(cpu.p01m)],
    ['P2M', hex2(cpu.ports.p2m)],
    ['P3M', hex2(cpu.ports.p3m)],
    ['P3 out', hex2(cpu.ports.p3Output)],
    ['TMR', hex2(cpu.timers.tmr)],
    ['T0/PRE0', `${hex2(cpu.timers.t[0])}/${hex2(cpu.timers.pre[0])}`],
    ['T1/PRE1', `${hex2(cpu.timers.t[1])}/${hex2(cpu.timers.pre[1])}`],
  ]

  const work = Array.from({ length: 16 }, (_, i) => cpu.readReg((cpu.rp & 0xf0) | i))

  const lines = []
  let a = cpu.pc
  for (let i = 0; i < 8; i++) {
    const d = disassemble(a, peek)
    lines.push(d)
    a = (a + d.bytes.length) & 0xffff
  }

  const trace = emu.machine.trace ?? []
  const traceText = (rows: number) =>
    trace
      .slice(0, rows)
      .map((b) => `${String(b.n).padStart(4)}  ${b.write ? 'W' : 'R'}  ${hex4(b.addr)}  ${hex2(b.data)}  ${ACC_NAMES[b.kind]}`)
      .join('\n')

  const copy = async () => {
    try {
      await navigator.clipboard.writeText(traceText(trace.length))
      setCopied(true)
      setTimeout(() => setCopied(false), 1500)
    } catch {
      setCopied(false)
    }
  }

  return (
    <details className="panel debug">
      <summary>
        <h2>Debugger &amp; bus trace</h2>
      </summary>
      <div className="debug-body">
      <div className="debug-cpu">
      <div className="row">
        <button type="button" className="btn" onClick={() => { emu.paused = !emu.paused; emu.notify() }} aria-pressed={emu.paused}>
          {emu.paused ? 'Resume' : 'Pause'}
        </button>
        <button type="button" className="btn" onClick={() => emu.step()} disabled={!emu.paused}>
          Step
        </button>
        {cpu.lastIllegal >= 0 && <span className="warn">Illegal opcode {hex2(cpu.lastIllegal)}h</span>}
      </div>

      <dl className="regs">
        {regs.map(([k, v]) => (
          <div key={k}>
            <dt>{k}</dt>
            <dd>{v}</dd>
          </div>
        ))}
      </dl>
      <div className="workregs" aria-label="Working registers r0 to r15">
        {work.map((v, i) => (
          <span key={i}>
            <b>r{i}</b>
            {hex2(v)}
          </span>
        ))}
      </div>

      <pre className="code disasm">
        {lines.map((d, i) => (
          <span key={d.addr} className={i === 0 ? 'cur' : ''}>
            {hex4(d.addr)}  {d.bytes.map(hex2).join(' ').padEnd(9)} {d.text}
            {'\n'}
          </span>
        ))}
      </pre>
      </div>

      <div className="debug-bus">
      <h3>External bus after reset</h3>
      <p className="hint">
        Each line is one bus cycle: /AS latches the address, /DS moves the data (R/W). The internal ROM checks
        for RAM at 0812h (read, write, read, write back) before it jumps there.
      </p>
      <pre className="code trace">{`   #  R/W addr  data kind\n${traceText(traceRows)}`}</pre>
      <div className="row">
        <button type="button" className="btn btn-small" onClick={() => setTraceRows((n) => Math.min(trace.length, n + 256))} disabled={traceRows >= trace.length}>
          Show more ({trace.length} recorded)
        </button>
        <button type="button" className="btn btn-small" onClick={copy}>
          {copied ? 'Copied' : 'Copy trace'}
        </button>
      </div>
      </div>
      </div>
    </details>
  )
}
