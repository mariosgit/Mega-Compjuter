/// <reference types="vitest/config" />
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import { viteSingleFile } from 'vite-plugin-singlefile'

// `vite build --mode artifact` inlines everything into one HTML file
// (scripts/build-artifact.mjs then turns it into the hosted artifact page).
export default defineConfig(({ mode }) => ({
  base: './',
  // public/ref holds the 27 MB of original PDFs: deployed with the site, left out of the single-file artifact
  publicDir: mode === 'artifact' ? false : 'public',
  plugins: [react(), ...(mode === 'artifact' ? [viteSingleFile()] : [])],
  build: {
    outDir: mode === 'artifact' ? 'dist-artifact' : 'dist',
  },
  test: {
    environment: 'node',
    testTimeout: 120_000,
  },
}))
