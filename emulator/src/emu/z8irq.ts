/**
 * Z8 interrupt priority logic.
 *
 * Six sources IRQ0..IRQ5:
 *   IRQ0 = P32 falling edge   IRQ1 = P33 falling edge   IRQ2 = P31 falling edge (also TIN)
 *   IRQ3 = P30 falling edge or serial receive           IRQ4 = T0 or serial transmit
 *   IRQ5 = T1
 * IRQ (FAh) latches requests, IMR (FBh) masks them (bit 7 = global enable),
 * IPR (F9h) sets the priority between three groups A {5,3}, B {2,0}, C {1,4}.
 * Vectors are the 16-bit words at 0000h (IRQ0) .. 000Ah (IRQ5) in program memory.
 */

/**
 * Returns the IRQ number to service, or -1.
 * @param pending IMR & IRQ & 3Fh
 * @param ipr     interrupt priority register
 */
export function pickInterrupt(pending: number, ipr: number): number {
  if (pending === 0) return -1

  const a = ipr & 0x20 ? [3, 5] : [5, 3]
  const b = ipr & 0x04 ? [0, 2] : [2, 0]
  const c = ipr & 0x02 ? [4, 1] : [1, 4]

  let order: number[][]
  switch (((ipr & 0x18) >> 2) | (ipr & 0x01)) {
    case 0: // reserved by Zilog; behaves like C > A > B
    case 1: order = [c, a, b]; break
    case 2: order = [a, b, c]; break
    case 3: order = [a, c, b]; break
    case 4: order = [b, c, a]; break
    case 5: order = [c, b, a]; break
    default: order = [b, a, c]; break // 6, and reserved 7
  }
  for (const group of order) {
    for (const n of group) {
      if (pending & (1 << n)) return n
    }
  }
  return -1
}
