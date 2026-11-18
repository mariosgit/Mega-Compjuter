import { useEffect, useRef } from 'react'
import type { Emulator } from '../emu/runner'
import { SCREEN_H, SCREEN_W, VRAM_BASE } from '../emu/jtc'

/**
 * The TV picture: 64 x 64 pixels from FE00h-FFFFh. The real image is 13 characters
 * of 5 pixels by 8 text lines; a B/W TV shows it as bluish white on dark grey.
 */
export function Screen({ emu }: { emu: Emulator }) {
  const canvas = useRef<HTMLCanvasElement>(null)

  useEffect(() => {
    const c = canvas.current
    if (!c) return
    const ctx = c.getContext('2d')
    if (!ctx) return
    const img = ctx.createImageData(SCREEN_W, SCREEN_H)
    const style = getComputedStyle(c)
    const on = parse(style.getPropertyValue('--phosphor')) ?? [226, 234, 255]
    let lastSig = ''

    const draw = () => {
      const m = emu.machine
      const lit = m.screenOn
      // cheap change detection: skip redraw if VRAM and power state are unchanged
      let sig = lit ? '1' : '0'
      if (lit) for (let a = 0; a < 512; a += 1) sig += String.fromCharCode(m.peek(VRAM_BASE + a))
      if (sig === lastSig) return
      lastSig = sig
      const d = img.data
      for (let y = 0; y < SCREEN_H; y++) {
        for (let x = 0; x < SCREEN_W; x++) {
          // unlit pixels stay transparent so only the phosphor glows
          const i = (y * SCREEN_W + x) * 4
          d[i] = on[0]
          d[i + 1] = on[1]
          d[i + 2] = on[2]
          d[i + 3] = lit && m.pixel(x, y) ? 255 : 0
        }
      }
      ctx.putImageData(img, 0, 0)
    }
    draw()
    return emu.subscribe(draw)
  }, [emu])

  return (
    <div className="tv" aria-label="TV picture of the JU+TE computer">
      <div className="tv-glass">
        <canvas ref={canvas} width={SCREEN_W} height={SCREEN_H} className="tv-canvas" />
      </div>
    </div>
  )
}

function parse(css: string): [number, number, number] | null {
  const m = /#([0-9a-f]{6})/i.exec(css.trim())
  if (!m) return null
  const v = parseInt(m[1], 16)
  return [(v >> 16) & 255, (v >> 8) & 255, v & 255]
}
