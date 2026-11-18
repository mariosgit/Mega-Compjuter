/**
 * The original publications, served from public/ref/ (GitHub Pages, dev server).
 * Scans/PDF versions by Volker Pohlers and others, from hc-ddr.hucki.net.
 */
export interface Doc {
  file: string
  title: string
  source: string
  summary: string
  pages: number
}

export const DOCS: Doc[] = [
  {
    file: 'jutecomp1.pdf',
    title: 'JU+TE Computer selbst gebaut, Teil I',
    source: 'Jugend+Technik 7/1987 – 2/1988',
    summary: 'The build series: processor board, memory module, TV and keyboard, TINY-MP-BASIC, Mondlandung.',
    pages: 31,
  },
  {
    file: 'jutecomp2.pdf',
    title: 'JU+TE Computer selbst gebaut, Teil II',
    source: 'Jugend+Technik 3/1988 – 7/1989',
    summary: 'Graphics, reaction games, tips, the 6516 memory module and the EPROM programmer.',
    pages: 54,
  },
  {
    file: 'jutecomp3.pdf',
    title: 'JU+TE Computer selbst gebaut, Teil III',
    source: 'Jugend+Technik 8/1989 – 2/1991',
    summary: 'Turm von Hanoi, OS variants, serial interfaces, A/D converter, FORTH-83, higher resolution.',
    pages: 56,
  },
  {
    file: 'abc.pdf',
    title: 'ABC Einchip-Mikrorechner',
    source: 'Jugend+Technik series',
    summary: 'How the U 881 / UB 8830 family works: architecture, instruction set, programming in machine code.',
    pages: 36,
  },
  {
    file: 'buch.pdf',
    title: 'Der kleine Selbstbau-Computer',
    source: 'Book, Verlag Junge Welt, Berlin',
    summary: 'From TINY-BASIC to a development system: the complete JU+TE computer in one volume.',
    pages: 67,
  },
  {
    file: 'es_2_3.pdf',
    title: 'ES 2.3 – das andere Betriebssystem',
    source: '1989',
    summary: 'Full-screen editor, 128 × 128 pixels, fast cassette interface and machine monitor.',
    pages: 14,
  },
  {
    file: '6k_es_40.pdf',
    title: 'Betriebssystem ES 4.0',
    source: 'Sonderdruck zu Jugend+Technik 6/1990, H. Scheutzow',
    summary: '"Die Krönung": full graphics and colour for the JU+TE computer, with the display controller.',
    pages: 31,
  },
]

/** Where the original files live online (they cannot be framed from other sites). */
export const DOC_ORIGIN = 'https://hc-ddr.hucki.net/wiki/lib/exe/fetch.php/tiny/'

/**
 * The single-file artifact build ships without the 27 MB of PDFs and its sandbox
 * can't show embedded PDFs, so there the documents open from the original site.
 */
export const DOCS_LOCAL = import.meta.env.MODE !== 'artifact'

export const docUrl = (d: Doc) => (DOCS_LOCAL ? `ref/${d.file}` : DOC_ORIGIN + d.file)
