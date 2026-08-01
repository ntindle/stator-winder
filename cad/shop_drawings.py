"""Generate dimensioned PDFs for local/factory shop artifacts."""

from __future__ import annotations

from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.pagesizes import landscape, letter
from reportlab.pdfbase.pdfmetrics import stringWidth
from reportlab.pdfgen import canvas

from shop_artifacts import EXTRUSION_CUTS, PDF_OUT, extrusion_total_mm


PAGE = landscape(letter)
W, H = PAGE
DATE = "2026-07-10"
REV = "A"
NAVY = colors.HexColor("#18324A")
BLUE = colors.HexColor("#2B6F9F")
PALE = colors.HexColor("#EAF2F7")
ORANGE = colors.HexColor("#D97706")
INK = colors.HexColor("#17212A")
GRAY = colors.HexColor("#5E6B75")


def _page_background(c: canvas.Canvas):
    """Paint an explicit opaque page before adding drawing content."""
    c.setFillColor(colors.white)
    c.rect(0, 0, W, H, fill=1, stroke=0)


def _header(c: canvas.Canvas, title: str, drawing: str):
    c.setFillColor(NAVY)
    c.rect(0, H - 50, W, 50, fill=1, stroke=0)
    c.setFillColor(colors.white)
    c.setFont("Helvetica-Bold", 16)
    c.drawString(28, H - 31, title)
    c.setFont("Helvetica", 8)
    c.drawRightString(W - 28, H - 22, f"DRAWING {drawing}  REV {REV}")
    c.drawRightString(W - 28, H - 35, f"{DATE}  PAGE 1/1")
    c.setFillColor(INK)


def _footer(c: canvas.Canvas, authority: str):
    c.setStrokeColor(colors.HexColor("#AAB6BE"))
    c.line(28, 28, W - 28, 28)
    c.setFillColor(GRAY)
    c.setFont("Helvetica", 7.5)
    c.drawString(28, 16, "UNITS: mm  |  UNLESS NOTED: remove burrs and loose fibres")
    c.drawRightString(W - 28, 16, f"AUTHORITY: {authority}")


def _table(c: canvas.Canvas, x: float, y: float, widths: list[float],
           headers: list[str], rows: list[list[str]], row_h: float = 21):
    total = sum(widths)
    c.setFillColor(NAVY)
    c.rect(x, y - row_h, total, row_h, fill=1, stroke=0)
    xx = x
    c.setFillColor(colors.white)
    c.setFont("Helvetica-Bold", 8)
    for header, width in zip(headers, widths):
        c.drawCentredString(xx + width / 2, y - row_h + 6, header)
        xx += width
    for row_index, row in enumerate(rows):
        top = y - row_h * (row_index + 1)
        c.setFillColor(PALE if row_index % 2 == 0 else colors.white)
        c.rect(x, top - row_h, total, row_h, fill=1, stroke=0)
        c.setFillColor(INK)
        c.setFont("Helvetica", 7.8)
        xx = x
        for value, width in zip(row, widths):
            shown = str(value)
            while stringWidth(shown, "Helvetica", 7.8) > width - 8 and len(shown) > 5:
                shown = shown[:-2] + "..."
            c.drawString(xx + 4, top - row_h + 6, shown)
            xx += width
    c.setStrokeColor(colors.HexColor("#A9B6BE"))
    c.rect(
        x,
        y - row_h * (len(rows) + 1),
        total,
        row_h * (len(rows) + 1),
        fill=0,
        stroke=1,
    )


def _notes(c: canvas.Canvas, x: float, y: float, width: float,
           title: str, lines: list[str]):
    wrapped: list[str] = []
    usable = width - 28
    for index, line in enumerate(lines, 1):
        current = f"{index}. "
        for word in line.split():
            candidate = current + ("" if current.endswith(" ") else " ") + word
            if stringWidth(candidate, "Helvetica", 8.5) <= usable:
                current = candidate
            else:
                wrapped.append(current)
                current = "   " + word
        wrapped.append(current)
    height = 25 + 15 * len(wrapped)
    c.setFillColor(PALE)
    c.roundRect(x, y - height + 7, width, height, 5, fill=1, stroke=0)
    c.setFillColor(NAVY)
    c.setFont("Helvetica-Bold", 10)
    c.drawString(x + 10, y, title)
    c.setFillColor(INK)
    c.setFont("Helvetica", 8.5)
    yy = y - 17
    for line in wrapped:
        c.drawString(x + 12, yy, line)
        yy -= 15


def _local_banner(c: canvas.Canvas, text: str):
    c.setFillColor(colors.HexColor("#FFF3E0"))
    c.setStrokeColor(ORANGE)
    c.roundRect(45, H - 88, W - 90, 24, 4, fill=1, stroke=1)
    c.setFillColor(ORANGE)
    c.setFont("Helvetica-Bold", 9)
    c.drawCentredString(W / 2, H - 80, text)


def felt_pdf(path: Path):
    c = canvas.Canvas(str(path), pagesize=PAGE)
    _page_background(c)
    _header(c, "FELT TENSIONER BACKING DISCS AND DRAG PADS", "M3-LOCAL-001")
    _local_banner(
        c,
        "LOCAL FABRICATION - OD20 PARTS ARE BELOW SENDCUTSEND MINIMUM LENGTH",
    )

    for cx, label, fill in (
        (165, "304 BACKING DISC", colors.HexColor("#D7DEE3")),
        (420, "F5 WOOL FELT PAD", colors.HexColor("#F3E6C8")),
    ):
        cy = 360
        c.setFillColor(fill)
        c.setStrokeColor(BLUE)
        c.setLineWidth(2)
        c.circle(cx, cy, 90, fill=1, stroke=1)
        c.setFillColor(colors.white)
        c.circle(cx, cy, 20.25, fill=1, stroke=1)
        c.setStrokeColor(GRAY)
        c.setDash(4, 3)
        c.line(cx - 103, cy, cx + 103, cy)
        c.line(cx, cy - 103, cx, cy + 103)
        c.setDash()
        c.setFillColor(INK)
        c.setFont("Helvetica-Bold", 9)
        c.drawCentredString(cx, 238, label)
        c.setFont("Helvetica", 8)
        c.drawCentredString(cx, 225, "FRONT VIEW - 5:1")

    _table(c, 545, 470, [90, 130], ["FEATURE", "REQUIREMENT"], [
        ["OD", "20.00 +/-0.10"],
        ["Bore", "4.50 +/-0.10"],
        ["Backing", "304 SS, 1.00 +/-0.05"],
        ["Pad stock", "McMaster 8341K31"],
        ["Pad grade", "F5 plain, 95% wool"],
        ["Pad thickness", "3.00 +/-0.30"],
        ["Quantity", "2 each"],
    ], 20)
    _notes(c, 45, 175, 710, "CUTTING AND USE NOTES", [
        "DXFs are 1:1 local-cut templates with two closed circular CUT entities.",
        "Backing discs: local laser, waterjet, punch, or lathe; deburr both faces completely.",
        "Pads: punch or knife-cut from plain-back F5 felt; no adhesive, oil, grit, or loose fibres.",
        "Twenty millimetres is 0.787 inch, below the published 1.5 inch SendCutSend minimum length.",
        "Set drag with the spring/wingnut after assembly; the drawing does not certify enamel wear.",
    ])
    _footer(c, "out/custom/shop/dxf/local_felt_*.dxf")
    c.save()


def sleeve_pdf(path: Path):
    c = canvas.Canvas(str(path), pagesize=PAGE)
    _page_background(c)
    _header(c, "DANCER HARD-STOP AND SPRING STAND-OFF SLEEVES", "M3-LOCAL-002")
    _local_banner(c, "LOCAL TURNED PARTS - NOT SHEET-CUT OR 3D-PRINTED COMPONENTS")

    for cx, outer, inner, length, label in (
        (115, 5.0, 3.2, 4.0, "STOP - QTY 2"),
        (270, 4.0, 2.2, 1.5, "FIXED ANCHOR - QTY 1"),
        (425, 4.0, 2.2, 4.0, "MOVING ANCHOR - QTY 1"),
    ):
        scale = 25
        c.setStrokeColor(BLUE)
        c.setLineWidth(2)
        c.rect(cx - length * scale / 2, 330 - outer * scale / 2,
               length * scale, outer * scale, fill=0, stroke=1)
        c.rect(cx - length * scale / 2, 330 - inner * scale / 2,
               length * scale, inner * scale, fill=0, stroke=1)
        c.setFillColor(INK)
        c.setFont("Helvetica-Bold", 8.5)
        c.drawCentredString(cx, 235, label)
        c.setFont("Helvetica", 7.5)
        c.drawCentredString(cx, 223, "SECTION VIEW - 25:1")

    _table(c, 520, 455, [105, 45, 45, 55],
           ["PART", "OD", "ID", "LENGTH"], [
        ["Hard stop", "5.00", "3.20", "4.00"],
        ["Fixed anchor", "4.00", "2.20", "1.50"],
        ["Moving anchor", "4.00", "2.20", "4.00"],
    ], 24)
    _notes(c, 45, 175, 710, "COMMON TURNING NOTES", [
        "Material 303 stainless steel; diameters +/-0.05 and lengths +/-0.05 unless noted.",
        "Face parallelism <=0.03; all spring-loop and arm contact surfaces Ra <=1.6 um.",
        "Break every edge 0.10 max and remove all wire edges, slivers, chips, and burrs.",
        "Supply the quantity shown; do not substitute printed plastic or coiled spring envelopes.",
    ])
    _footer(c, "out/custom/shop/step/dancer_*_sleeve*.step")
    c.save()


def extrusion_pdf(path: Path):
    c = canvas.Canvas(str(path), pagesize=PAGE)
    _page_background(c)
    _header(c, "2020 SLOT-6 EXTRUSION CUT LIST", "FRAME-LOCAL-001")
    _local_banner(c, "FACTORY CUT-TO-LENGTH OR LOCAL SQUARE-SAW PROCESS - NO END TAPPING")

    rows = [
        [row.line_id, row.description, str(row.quantity), f"{row.length_mm:.2f}",
         f"{row.quantity * row.length_mm:.2f}"]
        for row in EXTRUSION_CUTS
    ]
    rows.append(["TOTAL", "10 members", "10", "-", f"{extrusion_total_mm():.2f}"])
    _table(c, 55, 475, [115, 220, 70, 105, 115],
           ["LINE", "DESCRIPTION", "QTY", "LENGTH", "LINE TOTAL"], rows, 27)
    _notes(c, 55, 215, 680, "ORDER AND INSPECTION NOTES", [
        "Profile is MISUMI HFS5-2020 or an exact B-type 20 x 20 slot-6 equivalent.",
        "Both ends square cut, deburred, and clean; length tolerance +/-0.50 mm.",
        "No end drilling or tapping: every connection uses slot nuts and external brackets.",
        "Do not substitute an unverified V-slot profile; the selected MISUMI nuts must engage.",
        "Total ordered length is 2775 mm, correcting the former approximately 2.5 m BOM note.",
    ])
    _footer(c, "out/custom/shop/csv/extrusion_cut_list.csv")
    c.save()


def main() -> int:
    PDF_OUT.mkdir(parents=True, exist_ok=True)
    for filename, generator in (
        ("felt_tensioner_consumables.pdf", felt_pdf),
        ("dancer_standoff_sleeves.pdf", sleeve_pdf),
        ("extrusion_cut_list.pdf", extrusion_pdf),
    ):
        path = PDF_OUT / filename
        generator(path)
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
