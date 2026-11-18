/**
 * Z8 disassembler (Zilog syntax, hex with 'h' suffix like the MAME z8 disassembler).
 */

const CTRL_NAMES: Record<number, string> = {
  0x00: 'P0', 0x01: 'P1', 0x02: 'P2', 0x03: 'P3',
  0xf0: 'SIO', 0xf1: 'TMR', 0xf2: 'T1', 0xf3: 'PRE1', 0xf4: 'T0', 0xf5: 'PRE0',
  0xf6: 'P2M', 0xf7: 'P3M', 0xf8: 'P01M', 0xf9: 'IPR', 0xfa: 'IRQ', 0xfb: 'IMR',
  0xfc: 'FLAGS', 0xfd: 'RP', 0xfe: 'SPH', 0xff: 'SPL',
}

const CC = ['F', 'LT', 'LE', 'ULE', 'OV', 'MI', 'Z', 'C', '', 'GE', 'GT', 'UGT', 'NOV', 'PL', 'NZ', 'NC']

const ALU = ['ADD', 'ADC', 'SUB', 'SBC', 'OR', 'AND', 'TCM', 'TM', '', '', 'CP', 'XOR']
const UNARY: Record<number, string> = {
  0x0: 'DEC', 0x1: 'RLC', 0x2: 'INC', 0x4: 'DA', 0x5: 'POP', 0x6: 'COM', 0x7: 'PUSH',
  0x8: 'DECW', 0x9: 'RL', 0xa: 'INCW', 0xb: 'CLR', 0xc: 'RRC', 0xd: 'SRA', 0xe: 'RR', 0xf: 'SWAP',
}

const h2 = (v: number) => v.toString(16).toUpperCase().padStart(2, '0') + 'h'
const h4 = (v: number) => v.toString(16).toUpperCase().padStart(4, '0') + 'h'

const r = (n: number) => `R${n & 15}`
const Ir = (n: number) => `@R${n & 15}`
const Irr = (n: number) => `@RR${n & 15}`
const regR = (v: number) => ((v & 0xf0) === 0xe0 ? r(v) : CTRL_NAMES[v] ?? h2(v))
const regRR = (v: number) => ((v & 0xf0) === 0xe0 ? `RR${v & 15}` : h2(v))
const regIR = (v: number) => ((v & 0xf0) === 0xe0 ? Ir(v) : '@' + h2(v))
const IM = (v: number) => '#' + h2(v)

export interface DisasmLine {
  addr: number
  bytes: number[]
  text: string
}

/** Disassemble one instruction at `addr`; `read` returns program memory bytes. */
export function disassemble(addr: number, read: (a: number) => number): DisasmLine {
  const op = read(addr)
  const b0 = read((addr + 1) & 0xffff)
  const b1 = read((addr + 2) & 0xffff)
  const hi = op >> 4
  const lo = op & 15
  let len = 1
  let text = 'Illegal'

  const ra = () => h4((addr + 2 + ((b0 << 24) >> 24)) & 0xffff)
  const da = () => h4((b0 << 8) | b1)
  const cc = () => (CC[hi] ? CC[hi] + ', ' : '')

  if (addr < 0x0c && addr % 2 === 0) {
    return { addr, bytes: [op, b0], text: `IRQ${addr / 2} vector ${h4((op << 8) | b0)}` }
  }

  switch (lo) {
    case 0x8: len = 2; text = `LD ${r(hi)}, ${regR(b0)}`; break
    case 0x9: len = 2; text = `LD ${regR(b0)}, ${r(hi)}`; break
    case 0xa: len = 2; text = `DJNZ ${r(hi)}, ${ra()}`; break
    case 0xb: len = 2; text = `JR ${cc()}${ra()}`; break
    case 0xc: len = 2; text = `LD ${r(hi)}, ${IM(b0)}`; break
    case 0xd: len = 3; text = `JP ${cc()}${da()}`; break
    case 0xe: text = `INC ${r(hi)}`; break
    case 0xf:
      text = ({ 8: 'DI', 9: 'EI', 0xa: 'RET', 0xb: 'IRET', 0xc: 'RCF', 0xd: 'SCF', 0xe: 'CCF', 0xf: 'NOP' } as Record<number, string>)[hi] ?? 'Illegal'
      break
    case 0x0:
    case 0x1: {
      const m = UNARY[hi]
      if (op === 0x30) { len = 2; text = `JP @${regRR(b0).replace(/^RR/, 'RR')}`; break }
      if (op === 0x31) { len = 2; text = `SRP ${IM(b0)}`; break }
      len = 2
      const arg = lo === 1 ? regIR(b0) : hi === 0x8 || hi === 0xa ? regRR(b0) : regR(b0)
      text = `${m} ${arg}`
      break
    }
    default: {
      if (ALU[hi] && lo >= 2 && lo <= 7) {
        const m = ALU[hi]
        switch (lo) {
          case 2: len = 2; text = `${m} ${r(b0 >> 4)}, ${r(b0)}`; break
          case 3: len = 2; text = `${m} ${r(b0 >> 4)}, ${Ir(b0)}`; break
          case 4: len = 3; text = `${m} ${regR(b1)}, ${regR(b0)}`; break
          case 5: len = 3; text = `${m} ${regR(b1)}, ${regIR(b0)}`; break
          case 6: len = 3; text = `${m} ${regR(b0)}, ${IM(b1)}`; break
          case 7: len = 3; text = `${m} ${regIR(b0)}, ${IM(b1)}`; break
        }
        break
      }
      switch (op) {
        case 0x82: len = 2; text = `LDE ${r(b0 >> 4)}, ${Irr(b0)}`; break
        case 0x83: len = 2; text = `LDEI ${Ir(b0 >> 4)}, ${Irr(b0)}`; break
        case 0x92: len = 2; text = `LDE ${Irr(b0)}, ${r(b0 >> 4)}`; break
        case 0x93: len = 2; text = `LDEI ${Irr(b0)}, ${Ir(b0 >> 4)}`; break
        case 0xc2: len = 2; text = `LDC ${r(b0 >> 4)}, ${Irr(b0)}`; break
        case 0xc3: len = 2; text = `LDCI ${Ir(b0 >> 4)}, ${Irr(b0)}`; break
        case 0xd2: len = 2; text = `LDC ${Irr(b0)}, ${r(b0 >> 4)}`; break
        case 0xd3: len = 2; text = `LDCI ${Irr(b0)}, ${Ir(b0 >> 4)}`; break
        case 0xc7: len = 3; text = `LD ${r(b0 >> 4)}, ${h2(b1)}(${r(b0)})`; break
        case 0xd7: len = 3; text = `LD ${h2(b1)}(${r(b0)}), ${r(b0 >> 4)}`; break
        case 0xd4: len = 2; text = `CALL ${regIR(b0).replace('@R', '@RR')}`; break
        case 0xd6: len = 3; text = `CALL ${da()}`; break
        case 0xe3: len = 2; text = `LD ${r(b0 >> 4)}, ${Ir(b0)}`; break
        case 0xf3: len = 2; text = `LD ${Ir(b0 >> 4)}, ${r(b0)}`; break
        case 0xe4: len = 3; text = `LD ${regR(b1)}, ${regR(b0)}`; break
        case 0xe5: len = 3; text = `LD ${regR(b1)}, ${regIR(b0)}`; break
        case 0xe6: len = 3; text = `LD ${regR(b0)}, ${IM(b1)}`; break
        case 0xe7: len = 3; text = `LD ${regIR(b0)}, ${IM(b1)}`; break
        case 0xf5: len = 3; text = `LD ${regIR(b1)}, ${regR(b0)}`; break
      }
    }
  }
  const bytes = [op, b0, b1].slice(0, len)
  return { addr, bytes, text }
}
