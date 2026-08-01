"""Render generated supplier PDFs to PNG for mandatory visual QA."""

from pathlib import Path

import pypdfium2 as pdfium
from pypdf import PdfReader


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "output" / "pdf"
TARGET = ROOT / "tmp" / "pdfs"


def main() -> int:
    TARGET.mkdir(parents=True, exist_ok=True)
    for old in TARGET.glob("custom-*.png"):
        old.unlink()
    for path in sorted(SOURCE.glob("*.pdf")):
        reader = PdfReader(str(path))
        document = pdfium.PdfDocument(str(path))
        if len(reader.pages) != len(document):
            raise RuntimeError(f"page-count mismatch for {path.name}")
        for index, page in enumerate(document):
            image = page.render(scale=2.25).to_pil()
            output = TARGET / f"custom-{path.stem}-p{index + 1}.png"
            image.save(output)
            print(f"{path.name}: page {index + 1} -> {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
