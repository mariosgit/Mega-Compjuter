// Turns the single-file build (dist-artifact/index.html, from `vite build --mode artifact`)
// into page content for a claude.ai artifact: the host adds its own <html>/<head>/<body>
// skeleton, so we keep only title, meta, font links, styles, the root element and the script.
import { readFileSync, writeFileSync } from 'node:fs'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'

const root = join(dirname(fileURLToPath(import.meta.url)), '..')
const html = readFileSync(join(root, 'dist-artifact/index.html'), 'utf8')

const pick = (re) => [...html.matchAll(re)].map((m) => m[0])
const title = pick(/<title>[\s\S]*?<\/title>/g)
const metas = pick(/<meta\s+name="(?:claude:viewport-fit-sides|description)"[^>]*>/g)
const links = pick(/<link\s+rel="(?:stylesheet|preconnect)"[^>]*>/g)
const styles = pick(/<style[^>]*>[\s\S]*?<\/style>/g)
const scripts = pick(/<script[^>]*>[\s\S]*?<\/script>/g)

if (!title.length || !scripts.length) throw new Error('unexpected build output (no title or script)')

const out = [...title, ...metas, ...links, ...styles, '<div id="root"></div>', ...scripts].join('\n')
writeFileSync(join(root, 'dist-artifact/artifact.html'), out + '\n')
console.log(`artifact page: dist-artifact/artifact.html (${(out.length / 1024).toFixed(0)} KB)`)
