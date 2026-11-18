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

/** Remove blanks outside of string literals (the editor stores what is typed; blanks there are noise). */
function stripBlanks(s: string): string {
  return s
    .split('"')
    .map((part, i) => (i % 2 === 0 ? part.replace(/\s+/g, '') : part))
    .join('"')
}

interface Statement {
  /** keyword key, e.g. 'P' */
  key: string
  text: string
  /** IF: the next statement is its THEN part */
  isIf: boolean
}
interface Line {
  num: number
  stmts: Statement[]
}

/** Parse a text listing into lines of keyword statements. */
export function parseListing(listing: string): { lines: Line[]; errors: string[] } {
  const lines: Line[] = []
  const errors: string[] = []

  const parseStatement = (stmt: string, lineNo: number, out: Statement[]) => {
    const upper = stmt.toUpperCase()
    const kw = KEYWORDS.find(([w]) => upper.startsWith(w))
    if (!kw) {
      errors.push(`${lineNo}: no keyword at "${stmt}"`)
      return
    }
    const [word, key] = kw
    let rest = stmt.slice(word.length).trim()
    if (word === 'REM') {
      out.push({ key, text: stmt.slice(word.length).replace(/^ /, ''), isIf: false })
      return
    }
    if (word === 'IF') {
      // IF cond THEN stmt: the THEN part is stored as the next statement
      const m = /^(.*?)\s*\bTHEN\b\s*(.*)$/i.exec(replaceOutsideQuotes(rest, /\bTHEN\b/gi, 'THEN'))
      if (m) {
        out.push({ key, text: stripBlanks(m[1]), isIf: true })
        if (m[2].trim()) parseStatement(m[2].trim(), lineNo, out)
        else errors.push(`${lineNo}: nothing after THEN`)
        return
      }
      errors.push(`${lineNo}: IF without THEN`)
    }
    if (word === 'TRAP') rest = replaceOutsideQuotes(rest, /\s*\bTO\b\s*/gi, ',')
    out.push({ key, text: stripBlanks(rest), isIf: false })
  }

  for (const raw of listing.split(/\r?\n/)) {
    const line = raw.trim()
    if (!line) continue
    const m = /^(\d+)\s*(.*)$/.exec(line)
    if (!m) {
      errors.push(`missing line number: "${line}"`)
      continue
    }
    const num = +m[1]
    if (num < 1 || num > 32767) errors.push(`${num}: line number out of range`)
    const stmts: Statement[] = []
    for (const stmt of splitStatements(m[2])) parseStatement(stmt, num, stmts)
    lines.push({ num, stmts })
  }
  return { lines, errors }
}

const charsOf = (t: string) => Array.from(t, (ch) => CHAR_ALIASES[ch] ?? ch)

/**
 * Convert a listing like
 *   10 LET A=5
 *   20 IF A>4 THEN PRINT "GROSS"
 * into keystrokes for the editor: number, ENTER, keyword key + text per
 * statement (';' between statements, ',' = THEN after IF), ENTER.
 */
export function listingToKeys(listing: string): { strokes: Stroke[]; errors: string[] } {
  const { lines, errors } = parseListing(listing)
  const strokes: Stroke[] = []
  for (const line of lines) {
    strokes.push(...String(line.num), '\n')
    line.stmts.forEach((st, i) => {
      if (i > 0) strokes.push(line.stmts[i - 1].isIf ? ',' : ';')
      strokes.push(st.key, ...charsOf(st.text))
    })
    strokes.push('\n')
  }
  return { strokes, errors }
}

/**
 * Convert a listing straight into the program format in RAM (what LOAD from tape
 * produces): lines sorted by number, terminated by 00h.
 */
export function listingToBytes(listing: string): { bytes: Uint8Array; errors: string[] } {
  const { lines, errors } = parseListing(listing)
  const byNum = new Map<number, Line>()
  for (const l of lines) byNum.set(l.num, l) // a later line replaces an earlier one, like in the editor
  const out: number[] = []
  for (const l of [...byNum.values()].sort((a, b) => a.num - b.num)) {
    if (!l.stmts.length) continue
    out.push(0x80 | (l.num >> 8), l.num & 0xff)
    l.stmts.forEach((st, i) => {
      if (i > 0) out.push(0x3b) // ';'
      for (const ch of [st.key, ...charsOf(st.text)]) out.push(ch.charCodeAt(0) & 0xff)
    })
    out.push(0x0d)
  }
  out.push(0x00)
  return { bytes: Uint8Array.from(out), errors }
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
