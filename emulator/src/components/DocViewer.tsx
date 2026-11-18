import { useEffect, useRef } from 'react'
import { docUrl, type Doc } from '../docs'

/** Full-size popup with the browser's own PDF viewer. Esc or the close button closes it. */
export function DocViewer({ doc, onClose }: { doc: Doc | null; onClose: () => void }) {
  const dialog = useRef<HTMLDialogElement>(null)

  useEffect(() => {
    const d = dialog.current
    if (!d) return
    if (doc && !d.open) d.showModal()
    if (!doc && d.open) d.close()
  }, [doc])

  return (
    <dialog
      ref={dialog}
      className="docviewer"
      aria-labelledby="doc-title"
      onClose={onClose}
      onClick={(e) => {
        if (e.target === dialog.current) onClose() // click on the backdrop
      }}
    >
      {doc && (
        <div className="docviewer-frame">
          <header className="docviewer-bar">
            <div>
              <h2 id="doc-title">{doc.title}</h2>
              <p>
                {doc.source} · {doc.pages} pages
              </p>
            </div>
            <div className="row">
              <a className="btn btn-small" href={docUrl(doc)} target="_blank" rel="noopener noreferrer">
                Open in new tab
              </a>
              <button type="button" className="btn btn-small" onClick={onClose} autoFocus>
                Close
              </button>
            </div>
          </header>
          <iframe className="docviewer-pdf" src={`${docUrl(doc)}#view=FitH`} title={doc.title} />
        </div>
      )}
    </dialog>
  )
}
