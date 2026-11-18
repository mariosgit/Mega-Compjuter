# ROM images

| File | Content | Origin |
|---|---|---|
| `ub8830.bin` | UB8830 D internal mask ROM, 0x0000–0x07FF: TINY-MP-BASIC interpreter + reset/test code | Extracted from `rom/u883rom.bin` in JTCEMU 2.1c (Jens Müller), downloaded from <https://hc-ddr.hucki.net/wiki/doku.php/tiny/jtcemu> |
| `../../src/rom.cpp` | 2K operating system ("Editor", 0x0800–0x0FFF) | Already in this repo; byte-identical to JTCEMU's `os2k_0800.bin` |

SHA-1 `ub8830.bin`: `816f5d08f8064b69b1779eb6661fde091aa58ba8`

Copyright of the ROM contents (per JTCEMU's notes, not covered by any open-source license):
- TINY-MP-BASIC interpreter: VEB Mikroelektronik Erfurt
- 2K system / ES 1988: Dr. Helmut Hoyer
