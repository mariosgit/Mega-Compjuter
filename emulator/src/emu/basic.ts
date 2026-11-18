/**
 * TINY-MP-BASIC helpers for the 2K OS.
 *
 * Program format in RAM (starting at E000h):
 *   line   := (80h | number >> 8) (number & FFh) statement { ';' statement } 0Dh
 *   stmt   := keyword-letter text...
 *   end    := 00h
 * Every keyword is stored as the single character of its key: P = PRINT,
 * G = GOTO, F = IF ... The editor works the same way: after a line number it
 * waits for a keyword key (A cursor), then takes the rest as text.
 *
 *   listingToKeys()  turns a text listing into the keystrokes a user would type.
 *   programToText()  reads the program from RAM and returns a listing.
 */

/** Keyword -> key character (the character stored in RAM). */
export const KEYWORDS: [string, string][] = [
  ['RETURN', 'R'], ['GOSUB', 'S'], ['PRINT', 'P'], ['INPUT', 'I'], ['GOTO', 'G'],
  ['PROC', 'O'], ['STOP', 'T'], ['WAIT', 'W'], ['CALL', 'C'], ['TRAP', '!'],
  ['TOFF', '/'], ['ELSE', '>'], ['LET', 'L'], ['REM', 'M'], ['END', 'E'],
  ['PTH', 'H'], ['IF', 'F'],
]
const KEY_TO_WORD = new Map(KEYWORDS.map(([w, k]) => [k, w]))

/** Characters of the DDR charset that differ from ASCII. */
const CHAR_ALIASES: Record<string, string> = { '¤': '$' }

/** One keystroke to type: a character (as on a PC keyboard) or ENTER. */
export type Stroke = string

/** Split a statement list at ';' outside of string literals. */
function splitStatements(s: string): string[] {
  const out: string[] = []
  let cur = ''
  let quoted = false
  for (const ch of s) {
    if (ch === '"') quoted = !quoted
    if (ch === ';' && !quoted) {
      out.push(cur)
      cur = ''
    } else cur += ch
  }
  out.push(cur)
  return out.map((x) => x.trim()).filter((x) => x.length > 0)
}

/** Replace a word outside of string literals (used for THEN / TO). */
function replaceOutsideQuotes(s: string, re: RegExp, by: string): string {
  return s
    .split('"')
    .map((part, i) => (i % 2 === 0 ? part.replace(re, by) : part))
    .join('"')
}

/**
 * Convert a listing like
 *   10 LET A=5
 *   20 IF A>4 THEN PRINT "GROSS"
 * into keystrokes for the editor.
 */
export function listingToKeys(listing: string): { strokes: Stroke[]; errors: string[] } {
  const strokes: Stroke[] = []
  const errors: string[] = []
  const typeText = (t: string) => {
    for (const ch of t) strokes.push(CHAR_ALIASES[ch] ?? ch)
  }

  const typeStatement = (stmt: string, lineNo: string) => {
    const upper = stmt.toUpperCase()
    const kw = KEYWORDS.find(([w]) => upper.startsWith(w))
    if (!kw) {
      errors.push(`${lineNo}: no keyword at "${stmt}"`)
      typeText(stmt)
      return
    }
    const [word, key] = kw
    strokes.push(key)
    let rest = stmt.slice(word.length).trim()
    if (word === 'IF') {
      // IF cond THEN stmt  ->  F cond , stmt   (THEN is the ',' key)
      const m = /^(.*?)\s*\bTHEN\b\s*(.*)$/i.exec(replaceOutsideQuotes(rest, /\bTHEN\b/gi, 'THEN'))
      if (m) {
        typeText(m[1].trim())
        strokes.push(',')
        if (m[2].trim()) typeStatement(m[2].trim(), lineNo)
        return
      }
    }
    if (word === 'TRAP') rest = replaceOutsideQuotes(rest, /\s*\bTO\b\s*/gi, ',')
    if (word === 'REM') {
      typeText(stmt.slice(word.length).replace(/^ /, ''))
      return
    }
    typeText(rest)
  }

  for (const raw of listing.split(/\r?\n/)) {
    const line = raw.trim()
    if (!line) continue
    const m = /^(\d+)\s*(.*)$/.exec(line)
    if (!m) {
      errors.push(`missing line number: "${line}"`)
      continue
    }
    const [, num, body] = m
    if (+num < 1 || +num > 32767) errors.push(`${num}: line number out of range`)
    typeText(num)
    strokes.push('\n')
    splitStatements(body).forEach((stmt, i) => {
      if (i > 0) strokes.push(';')
      typeStatement(stmt, num)
    })
    strokes.push('\n')
  }
  return { strokes, errors }
}

/** Read the BASIC program from memory and return a listing (null if none). */
export function programToText(peek: (addr: number) => number, start = 0xe000): string | null {
  let a = start
  const lines: string[] = []
  for (let guard = 0; guard < 2000; guard++) {
    const hi = peek(a)
    if (!(hi & 0x80)) break // 00h = end of program
    const num = ((hi & 0x7f) << 8) | peek(a + 1)
    if (num === 0 || num > 32767) break
    a += 2
    const stmts: string[] = []
    let afterIf = false
    while (a < 0x10000) {
      const key = peek(a++)
      const word = KEY_TO_WORD.get(String.fromCharCode(key))
      let text = ''
      let ch = peek(a)
      while (ch !== 0x0d && ch !== 0x00 && ch !== 0x3b && a < 0x10000) {
        text += String.fromCharCode(ch)
        ch = peek(++a)
      }
      if (word === 'TRAP') text = text.replace(',', ' TO ')
      const stmt = word ? (text ? `${word} ${text}` : word) : String.fromCharCode(key) + text
      if (afterIf) stmts[stmts.length - 1] += ' THEN ' + stmt
      else stmts.push(stmt)
      afterIf = word === 'IF' && ch === 0x3b
      a++ // skip ';' or CR
      if (ch !== 0x3b) break
    }
    lines.push(`${num} ${stmts.join(';')}`)
  }
  return lines.length ? lines.join('\n') : null
}
