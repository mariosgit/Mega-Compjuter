import { describe, expect, test } from 'vitest'
import { Z8, type Z8Host } from '../src/emu/z8core'
import { F_C, F_H, F_S, F_V, F_Z } from '../src/emu/z8defs'
import { pickInterrupt } from '../src/emu/z8irq'

/** A bare Z8 with 64K of external RAM; programs are loaded at 1000h. */
function cpuWith(program: number[]) {
  const mem = new Uint8Array(0x10000)
  const host: Z8Host = {
    memRead: (a) => mem[a],
    memWrite: (a, v) => { mem[a] = v },
    portRead: () => 0xff,
    portWrite: () => {},
  }
  const cpu = new Z8(host, new Uint8Array(0), { romSize: 0 })
  cpu.powerOn()
  cpu.p01m = 0x92 // P0 = A8-A15, P1 = AD0-AD7, external stack
  mem.set(program, 0x1000)
  cpu.pc = 0x1000
  cpu.sp = 0xff00
  return { cpu, mem }
}
const run = (cpu: Z8, n: number) => { for (let i = 0; i < n; i++) cpu.step() }

describe('Z8 core', () => {
  test('LD and ADD set result and flags', () => {
    // SRP #10h; LD r0,#7Fh; LD r1,#01h; ADD r0,r1
    const { cpu } = cpuWith([0x31, 0x10, 0x0c, 0x7f, 0x1c, 0x01, 0x02, 0x01])
    run(cpu, 4)
    expect(cpu.regs[0x10]).toBe(0x80)
    expect(cpu.flags & (F_V | F_S | F_H)).toBe(F_V | F_S | F_H)
    expect(cpu.flags & (F_C | F_Z)).toBe(0)
  })

  test('ADC with carry and DA give a BCD result', () => {
    // SRP #20h; LD r0,#19h; SCF; ADC r0,#... via R1,IM form: ADC 20h,#22h; DA 20h
    const { cpu } = cpuWith([0x31, 0x20, 0x0c, 0x19, 0xdf, 0x16, 0x20, 0x22, 0x40, 0x20])
    run(cpu, 5)
    expect(cpu.regs[0x20]).toBe(0x42) // 19 + 22 + 1 = 42 (BCD)
  })

  test('working registers follow RP and Ex addressing', () => {
    // SRP #30h; LD E5h,#AAh  (E5 = r5 -> register 35h)
    const { cpu } = cpuWith([0x31, 0x30, 0xe6, 0xe5, 0xaa])
    run(cpu, 2)
    expect(cpu.regs[0x35]).toBe(0xaa)
  })

  test('DJNZ loops until zero', () => {
    // SRP #10h; LD r2,#05h; CLR 11h; loop: INC 11h; DJNZ r2,loop
    const { cpu } = cpuWith([0x31, 0x10, 0x2c, 0x05, 0xb0, 0x11, 0x20, 0x11, 0x2a, 0xfc])
    run(cpu, 3 + 5 * 2)
    expect(cpu.regs[0x11]).toBe(5)
    expect(cpu.regs[0x12]).toBe(0)
    expect(cpu.pc).toBe(0x100a)
  })

  test('CALL/RET use the external stack', () => {
    // CALL 1010h; NOP ... at 1010h: RET
    const { cpu, mem } = cpuWith([0xd6, 0x10, 0x10])
    mem[0x1010] = 0xaf
    cpu.step()
    expect(cpu.pc).toBe(0x1010)
    expect(cpu.sp).toBe(0xfefe)
    expect((mem[0xfefe] << 8) | mem[0xfeff]).toBe(0x1003)
    cpu.step()
    expect(cpu.pc).toBe(0x1003)
    expect(cpu.sp).toBe(0xff00)
  })

  test('LDE / LDEI move bytes through register pairs', () => {
    // SRP #10h; LD r2,#20h; LD r3,#00h (rr2 = 2000h); LD r4,#40h; LDEI @r4,@rr2 x2
    const { cpu, mem } = cpuWith([0x31, 0x10, 0x2c, 0x20, 0x3c, 0x00, 0x4c, 0x40, 0x83, 0x42, 0x83, 0x42])
    mem[0x2000] = 0x11
    mem[0x2001] = 0x22
    run(cpu, 6)
    expect([cpu.regs[0x40], cpu.regs[0x41]]).toEqual([0x11, 0x22])
    expect(cpu.readPair(0x12)).toBe(0x2002)
  })

  test('T0 interrupt fires and IRET returns', () => {
    // vector table is in external memory for a ROM-less part: IRQ4 -> 1100h
    const { cpu, mem } = cpuWith([
      0xe6, 0xf4, 0x02, // LD T0,#2
      0xe6, 0xf5, 0x05, // LD PRE0,#(1<<2)|1  continuous, prescale 1
      0xe6, 0xfb, 0x10, // LD IMR,#10h
      0x9f, //             EI
      0xe6, 0xf1, 0x03, // LD TMR,#03h (load + enable T0)
      0x8b, 0xfe, //       JR $
    ])
    mem[0x08] = 0x11
    mem[0x09] = 0x00
    mem[0x1100] = 0x20 // INC 40h
    mem[0x1101] = 0x40
    mem[0x1102] = 0xbf // IRET
    run(cpu, 200)
    expect(cpu.regs[0x40]).toBeGreaterThan(3)
  })

  test('interrupt priority groups', () => {
    // IPR 0 behaves like C > A > B: IRQ1 first
    expect(pickInterrupt(0b111111, 0)).toBe(1)
    // A > B > C (IPR = 08h): IRQ5 first
    expect(pickInterrupt(0b111111, 0x08)).toBe(5)
    expect(pickInterrupt(0, 0x08)).toBe(-1)
  })
})
