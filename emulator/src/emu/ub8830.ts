/**
 * UB8830 D: the Z8 single-chip MCU from VEB Mikroelektronik Erfurt used in the
 * JU+TE computer. It corresponds to the Zilog Z8601 with a 2 KByte mask ROM at
 * 0000h-07FFh containing the TINY-MP-BASIC interpreter, 124 general registers
 * (04h-7Fh), four ports, two counter/timers, a UART and six interrupt sources.
 *
 * Its ROM-less sibling UB8833 (Z8681-like, as on the Arduino shield in this repo)
 * would be `new Z8(host, new Uint8Array(0), { romSize: 0 })`.
 *
 * Reset vector: 000Ch. The mask ROM's start-up code sets P01M for external
 * memory (P0 = A8-A15, P1 = AD0-AD7), configures P3 and jumps to 0812h in the
 * external EPROM, unless P32 is tied to P35 (test mode). The IRQ vectors at
 * 0000h-000Bh point to 0800h, 0803h, ... 080Fh in external memory.
 */
import { Z8, type Z8Host } from './z8core'
import { UB8830_ROM } from './roms/ub8830'

export const XTAL_HZ = 8_000_000
/** Internal clock = XTAL / 2; all cycle counts are in this unit. */
export const CPU_HZ = XTAL_HZ / 2

export class UB8830 extends Z8 {
  constructor(host: Z8Host) {
    super(host, UB8830_ROM, { romSize: 0x800, gprTop: 0x7f })
  }
}
