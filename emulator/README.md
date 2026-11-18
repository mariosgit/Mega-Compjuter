# JU+TE Tiny emulator

A browser emulator of the original **JU+TE computer** ("Tiny") from *Jugend+Technik* 7/1987 – 2/1988:
UB 8830 D single-chip computer at 8 MHz, 2 KByte operating system (the same image as
[`../src/rom.cpp`](../src/rom.cpp)), TINY-MP-BASIC in the chip's mask ROM, a 64 × 64 pixel software-generated
TV picture and the 4-row matrix keyboard.

It doubles as a reference for the Arduino shield in this repo. The debugger shows every external bus
cycle after reset, in the same terms (/AS address, /DS data, R/W) as the hardware logs in the main README.

**Online:** <https://mariosgit.github.io/Mega-Compjuter/>, built and deployed by
[`../.github/workflows/pages.yml`](../.github/workflows/pages.yml) on every push to `main` that touches
`emulator/` or `src/rom.cpp` (or manually via *Actions → Run workflow*).

## Run

```sh
npm install
npm run dev            # http://localhost:5173
npm test               # CPU unit tests + machine tests (boot, typing, RUN, SAVE/LOAD)
npm run build          # static site in dist/ (relative paths, works on any static host)
npm run build:artifact # single self-contained HTML in dist-artifact/
```

`npm run roms` (run automatically before dev/build/test) generates `src/emu/roms/*.ts` from
`../src/rom.cpp` and `roms/ub8830.bin`.

## Using it

- **K** after the cursor: command mode. `CLR` = NEW, `−` = LIST, `+` = RUN, `*` = CONT,
  `SHIFT P` = SAVE, `SHIFT O` = LOAD. A letter key shows that variable.
- Line number + `ENTER` gives an **A** cursor: one key per keyword (P = PRINT, G = GOTO, F = IF, `,` = THEN, …),
  then the rest of the statement, `ENTER`.
- The PC keyboard types too. *Programs* types a listing in key by key (at 10× speed).
- *Cassette*: every SAVE is recorded as a tape (kept in the browser). To load, press `SHIFT O`, then Play.

## Code map

| File | What |
|---|---|
| `src/emu/z8core.ts` | Z8 CPU core: register file, control registers, stack, interrupts, fetch/execute |
| `src/emu/z8ops.ts` | Opcode table (16 × 16 matrix), addressing modes, ALU, cycle counts |
| `src/emu/z8ports.ts` | Ports P0–P3 (P01M/P2M/P3M modes, P3 IRQ edges) and the UART |
| `src/emu/z8timers.ts` | Counter/timers T0/T1 with prescalers, TOUT |
| `src/emu/z8irq.ts` | IRQ0–5 priority logic (IPR groups) |
| `src/emu/z8disasm.ts` | Disassembler for the debugger |
| `src/emu/ub8830.ts` | The UB 8830 D: Z8 + 2K mask ROM |
| `src/emu/jtc.ts` | The board: memory map, keyboard matrix at 6000h, RAM mirroring, picture at FE00h, P3 sound/tape, bus trace |
| `src/emu/keyboard.ts` | Key table from the OS ROM (0F00h), timed key taps |
| `src/emu/tape.ts` | Virtual cassette (tone envelope on P36 → P30) |
| `src/emu/basic.ts` | Listing → keystrokes, RAM → listing |
| `src/emu/runner.ts` | Real-time loop, speed control, sound |
| `src/components/*` | React UI |

## Hardware facts the emulation relies on

- Memory map (J+T 7/1987): 0000–07FF internal ROM, 0800–0FFF EPROM, 6000–7FFF keyboard (/CS3),
  E000–FFFF RAM (/CS7, 1 KByte mirrored when only 2 × U214 are fitted), picture memory FE00–FFFF.
- Keyboard: latched A0–A3 → 74154 column select, rows S0–S3 read on D4–D7 (V 40098).
- Picture: the OS's interrupt routine (via 080Ch) reads the picture memory in step with the TV lines, and
  2 × D195 shift each byte out at 2 MHz. A 0 bit is a lit pixel.
- After reset the mask ROM probes for RAM at 0812h (read, write 72h, read, write back) and then jumps there.
- Key timing: the OS beeps ~80 ms per key and needs ~0.2 s per keystroke; faster taps get lost.
- Cassette (2K OS): on-off keyed ~5 kHz tone, ~1.66 ms units, 5 s leader; P30 = tone present.

## Credits

- Hardware and 2K system: Dr. Helmut Hoyer, *JU+TE Computer selbst gebaut*, Jugend+Technik 1987/88
  (`../ref/jutecomp1.pdf`).
- TINY-MP-BASIC mask ROM: VEB Mikroelektronik Erfurt. Image from JTCEMU (Jens Müller), see `roms/SOURCE.md`.
  The ROM contents are not under an open-source license.
- Z8 behaviour and timing: Zilog Z8 datasheet, cross-checked against MAME's z8 core (BSD-3-Clause).
- Example programs: J+T 12/1987.
