/** Shared Z8 constants (kept separate from z8core.ts to avoid import cycles). */

/** Kind of an external bus access, reported to the host (useful for traces). */
export const ACC_OPCODE = 0 // opcode fetch
export const ACC_OPERAND = 1 // operand fetch
export const ACC_PROGRAM = 2 // LDC / LDCI
export const ACC_DATA = 3 // LDE / LDEI
export const ACC_STACK = 4 // push/pop, call/ret, interrupts
export const ACC_VECTOR = 5 // interrupt vector read

export const ACC_NAMES = ['fetch', 'operand', 'LDC', 'LDE', 'stack', 'vector'] as const

/** FLAGS register (FCh) bits. */
export const F_C = 0x80
export const F_Z = 0x40
export const F_S = 0x20
export const F_V = 0x10
export const F_D = 0x08
export const F_H = 0x04

export const IMR_ENABLE = 0x80
export const P01M_INTERNAL_STACK = 0x04
