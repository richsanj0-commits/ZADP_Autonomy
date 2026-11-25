# vendor

Libraries bundled so the web app (`zadp_game.html`) works offline.

| Path | Library | Version (from the file header) |
|---|---|---|
| `clipper.js` | Clipper (Angus Johnson), polygon offsetting and clipping, JavaScript port | 6.4.2 (`ClipperLib.version` 6.4.2.2), dated 27 February 2017 |
| `tesseract/tesseract.min.js`, `tesseract/worker.min.js` | Tesseract.js, text reader for the Round 1 dimensions | not stated in the files; the online fallback in `zadp_game.html` names `tesseract.js@5.1.1` |
| `tesseract/core/` | Tesseract WebAssembly cores (`tesseract-core`, `-simd`, `-lstm`, `-simd-lstm`) | not stated in the files |
| `tesseract/lang/eng.traineddata.gz` | English language data | not stated in the file |

The page loads `vendor/` over http only (not `file://`); see the root `README.md`.
