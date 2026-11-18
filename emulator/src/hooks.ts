import { useEffect, useState } from 'react'
import type { Emulator } from './emu/runner'

/** Re-render the calling component at most `hz` times per second while the emulator runs. */
export function useEmuRefresh(emu: Emulator, hz = 8): number {
  const [n, setN] = useState(0)
  useEffect(() => {
    let last = 0
    return emu.subscribe(() => {
      const now = performance.now()
      if (now - last >= 1000 / hz) {
        last = now
        setN((x) => x + 1)
      }
    })
  }, [emu, hz])
  return n
}

/** localStorage-backed state; falls back to memory when storage is unavailable. */
export function useStoredState<T>(key: string, initial: T): [T, (v: T | ((old: T) => T)) => void] {
  const [value, setValue] = useState<T>(() => {
    try {
      const raw = localStorage.getItem(key)
      return raw ? (JSON.parse(raw) as T) : initial
    } catch {
      return initial
    }
  })
  useEffect(() => {
    try {
      localStorage.setItem(key, JSON.stringify(value))
    } catch {
      /* storage blocked or full: keep it in memory */
    }
  }, [key, value])
  return [value, setValue]
}

export const hex2 = (v: number) => v.toString(16).toUpperCase().padStart(2, '0')
export const hex4 = (v: number) => v.toString(16).toUpperCase().padStart(4, '0')
