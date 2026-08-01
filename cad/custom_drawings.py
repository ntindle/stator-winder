"""Generate dimensioned supplier PDFs for the custom release STEP packet.

The drawings are intentionally conservative RFQ/inspection documents.  They
define every functional interface and finish, while the paired STEP remains
the shape authority.  Run ``custom_parts.py`` first, then render these PDFs
with Poppler for visual QA per the repository PDF workflow.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.pagesizes import landscape, letter
from reportlab.pdfbase.pdfmetrics import stringWidth
from reportlab.pdfgen import canvas


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "output" / "pdf"
PAGE = landscape(letter)
W, H = PAGE
REV = "A"
DATE = "2026-07-10"

NAVY = colors.HexColor("#18324A")
BLUE = colors.HexColor("#2B6F9F")
PALE = colors.HexColor("#EAF2F7")
INK = colors.HexColor("#17212A")
GRAY = colors.HexColor("#5E6B75")


def _header(c: canvas.Canvas, title: str, drawing: str, page: int = 1,
            pages: int = 1, rev: str = REV, date: str = DATE):
    c.setFillColor(NAVY)
    c.rect(0, H - 50, W, 50, fill=1, stroke=0)
    c.setFillColor(colors.white)
    c.setFont("Helvetica-Bold", 16)
    c.drawString(28, H - 31, title)
    c.setFont("Helvetica", 8)
    c.drawRightString(W - 28, H - 22, f"DRAWING {drawing}  REV {rev}")
    c.drawRightString(W - 28, H - 35, f"{date}  PAGE {page}/{pages}")
    c.setFillColor(INK)


def _footer(c: canvas.Canvas, step_name: str, *, authority_dir: str = "out/custom/step"):
    c.setStrokeColor(colors.HexColor("#AAB6BE"))
    c.line(28, 28, W - 28, 28)
    c.setFillColor(GRAY)
    c.setFont("Helvetica", 7.5)
    c.drawString(28, 16, "UNITS: mm  |  UNLESS NOTED: remove burrs; break non-contact edges 0.2 max")
    c.drawRightString(W - 28, 16, f"SHAPE AUTHORITY: {authority_dir}/{step_name}.step")


def _notes(c: canvas.Canvas, x: float, y: float, width: float,
           title: str, lines: list[str]):
    wrapped = []
    usable = width - 28
    for index, line in enumerate(lines, 1):
        words = line.split()
        prefix = f"{index}. "
        current = prefix
        for word in words:
            candidate = current + ("" if current.endswith(" ") else " ") + word
            if stringWidth(candidate, "Helvetica", 8.5) <= usable:
                current = candidate
            else:
                wrapped.append(current)
                current = "   " + word
        wrapped.append(current)
    c.setFillColor(PALE)
    c.roundRect(x, y - 18 - 15 * len(wrapped), width,
                25 + 15 * len(wrapped),
                5, fill=1, stroke=0)
    c.setFillColor(NAVY)
    c.setFont("Helvetica-Bold", 10)
    c.drawString(x + 10, y, title)
    c.setFillColor(INK)
    c.setFont("Helvetica", 8.5)
    yy = y - 17
    for line in wrapped:
        c.drawString(x + 12, yy, line)
        yy -= 15


def _table(c: canvas.Canvas, x: float, y: float, widths: list[float],
           headers: list[str], rows: list[list[str]], row_h: float = 18):
    total = sum(widths)
    c.setFillColor(NAVY)
    c.rect(x, y - row_h, total, row_h, fill=1, stroke=0)
    xx = x
    c.setFont("Helvetica-Bold", 8)
    c.setFillColor(colors.white)
    for header, width in zip(headers, widths):
        c.drawCentredString(xx + width / 2, y - row_h + 5, header)
        xx += width
    for row_i, row in enumerate(rows):
        top = y - row_h * (row_i + 1)
        c.setFillColor(PALE if row_i % 2 == 0 else colors.white)
        c.rect(x, top - row_h, total, row_h, fill=1, stroke=0)
        xx = x
        c.setFillColor(INK)
        c.setFont("Helvetica", 7.8)
        for value, width in zip(row, widths):
            shown = str(value)
            while stringWidth(shown, "Helvetica", 7.8) > width - 8 and len(shown) > 5:
                shown = shown[:-2] + "..."
            c.drawString(xx + 4, top - row_h + 5, shown)
            xx += width
    c.setStrokeColor(colors.HexColor("#A9B6BE"))
    c.rect(x, y - row_h * (len(rows) + 1), total,
           row_h * (len(rows) + 1), fill=0, stroke=1)


def torus_pdf(path: Path):
    c = canvas.Canvas(str(path), pagesize=PAGE)
    _header(c, "POLISHED CERAMIC FLYER TORUS", "WG-001")
    cx, cy = 225, 330
    scale = 18
    c.setStrokeColor(BLUE)
    c.setLineWidth(2)
    c.circle(cx, cy, 9.5 * scale, fill=0, stroke=1)
    c.circle(cx, cy, 3.5 * scale, fill=0, stroke=1)
    c.setStrokeColor(GRAY)
    c.setDash(4, 3)
    c.line(cx - 190, cy, cx + 190, cy)
    c.line(cx, cy - 190, cx, cy + 190)
    c.setDash()
    c.setFillColor(INK)
    c.setFont("Helvetica-Bold", 10)
    c.drawString(70, 118, "FRONT VIEW - TORUS AXIS NORMAL TO PAGE")
    _table(c, 470, 495, [145, 120], ["FEATURE", "REQUIREMENT"], [
        ["Major radius", "R6.50 +/-0.05"],
        ["Circular section", "R3.00 +/-0.05"],
        ["Overall diameter", "19.00 REF"],
        ["Clear bore", "7.00 REF"],
        ["Axial thickness", "6.00 REF"],
        ["Material", "99.8% Al2O3"],
        ["Wire surface", "Ra <=0.2 um"],
    ])
    _notes(c, 470, 290, 285, "RFQ AND INSPECTION NOTES", [
        "Diamond-polish the complete toroidal exterior; no molding seam on contact surface.",
        "OD-to-ID concentricity <=0.05; no chips, pits, cracks, glaze runs, or sharp defects.",
        "Epoxy only the rear half into the printed R3.15 cradle; working meridian stays exposed.",
        "Quote quantity 2: one production part plus one fragile-part spare.",
    ])
    _footer(c, "tip_toroid_guide")
    c.save()


def fixed_eyelet_pdf(path: Path):
    c = canvas.Canvas(str(path), pagesize=PAGE)
    _header(c, "POLISHED FIXED CERAMIC WIRE GUIDE", "WG-003")

    # Front view and a deliberately enlarged axial section make the two
    # polished bore blends unambiguous to a ceramic supplier.
    cx, cy, scale = 190, 345, 29
    c.setStrokeColor(BLUE)
    c.setLineWidth(2)
    c.circle(cx, cy, 4.5 * scale, fill=0, stroke=1)
    c.circle(cx, cy, 2.0 * scale, fill=0, stroke=1)
    c.setStrokeColor(GRAY)
    c.setDash(4, 3)
    c.line(cx - 155, cy, cx + 155, cy)
    c.line(cx, cy - 155, cx, cy + 155)
    c.setDash()
    c.setFillColor(INK)
    c.setFont("Helvetica-Bold", 10)
    c.drawCentredString(cx, 175, "FRONT VIEW - 8:1")

    sx0, sy0, sscale = 365, 110, 34
    outer_r, bore_r, thick, blend = 4.5, 2.0, 3.0, 0.75
    # Section outline. Curved bore lips are represented by paired arcs; the
    # STEP remains the shape authority for the exact filleted solid.
    c.setStrokeColor(BLUE)
    c.setLineWidth(2)
    c.rect(sx0, sy0, thick * sscale, (outer_r - bore_r) * sscale,
           fill=0, stroke=1)
    c.rect(sx0, sy0 + (outer_r + bore_r) * sscale,
           thick * sscale, (outer_r - bore_r) * sscale, fill=0, stroke=1)
    c.arc(sx0, sy0 + (outer_r - bore_r - 2 * blend) * sscale,
          sx0 + 2 * blend * sscale,
          sy0 + (outer_r - bore_r) * sscale, 270, 90)
    c.arc(sx0 + (thick - 2 * blend) * sscale,
          sy0 + (outer_r - bore_r - 2 * blend) * sscale,
          sx0 + thick * sscale,
          sy0 + (outer_r - bore_r) * sscale, 90, 90)
    c.arc(sx0, sy0 + (outer_r + bore_r) * sscale,
          sx0 + 2 * blend * sscale,
          sy0 + (outer_r + bore_r + 2 * blend) * sscale, 180, 90)
    c.arc(sx0 + (thick - 2 * blend) * sscale,
          sy0 + (outer_r + bore_r) * sscale,
          sx0 + thick * sscale,
          sy0 + (outer_r + bore_r + 2 * blend) * sscale, 270, 90)
    c.setFillColor(INK)
    c.setFont("Helvetica-Bold", 10)
    c.drawString(sx0 - 10, 82, "AXIAL SECTION - 12:1; BOTH BORE RIMS R0.75")

    _table(c, 520, 505, [135, 120], ["FEATURE", "REQUIREMENT"], [
        ["Outside diameter", "9.00 +0/-0.03"],
        ["Clear bore waist", "4.00 +0.05/0"],
        ["Axial thickness", "3.00 +/-0.05"],
        ["Both bore rims", "R0.75 +/-0.05"],
        ["Material", "99.8% Al2O3"],
        ["Wire surfaces", "Ra <=0.2 um"],
        ["OD concentricity", "<=0.03 to bore"],
    ])
    _notes(c, 500, 285, 260, "RFQ AND INSPECTION NOTES", [
        "Diamond-polish the full bore and both R0.75 blends; no molding seam or glaze ridge may cross a wire-contact surface.",
        "No chips, cracks, pits, sharp defects, abrasive residue, or polishing compound; ultrasonically clean before packing.",
        "OD is the bonded light-interference seat in a nominal ID8.90 PETG cradle; do not increase OD above 9.00.",
        "Quote quantity 2: one production entry guide plus one fragile-part spare; provide dimensional inspection with shipment.",
    ])
    _footer(c, "fixed_eyelet_id4_od9_t3")
    c.save()


def sleeve_pdf(path: Path):
    c = canvas.Canvas(str(path), pagesize=PAGE)
    _header(c, "PHASE-LEAD SHAFT WRAP SLEEVE FAMILY", "WG-002")
    c.setStrokeColor(BLUE)
    c.setLineWidth(2)
    c.rect(70, 315, 270, 120, fill=0, stroke=1)
    c.rect(70, 350, 270, 50, fill=0, stroke=1)
    c.setFillColor(INK)
    c.setFont("Helvetica-Bold", 10)
    c.drawString(70, 285, "SECTION VIEW - AXIS HORIZONTAL")
    rows = []
    for shaft in range(3, 9):
        rows.append([f"{shaft:.2f}", f"{shaft + 0.05:.2f} +0.03/0",
                     f"{max(8.0, shaft + 2.0):.2f} +/-0.05", "6.00 +/-0.05"])
    _table(c, 410, 500, [70, 110, 100, 95],
           ["SHAFT OD", "BORE E", "OUTER OD B", "LENGTH C"], rows)
    _notes(c, 410, 300, 345, "FUNCTIONAL NOTES", [
        "99.8% alumina, seamless one-piece sleeve; no slit, seam, or exposed fastener.",
        "Both outside rims R0.75 minimum; OD and rims diamond-polished Ra <=0.2 um.",
        "Bore Ra <=0.8 um; bore-to-OD concentricity <=0.03; ID entry chamfer C0.2 max.",
        "Generate the job sleeve from measured shaft OD, then qualify bond gap and full cure.",
    ])
    _footer(c, "shaft_wrap_sleeve_d4")
    c.save()


def t8_pdf(path: Path):
    c = canvas.Canvas(str(path), pagesize=PAGE)
    _header(c, "CUSTOM TR8x8(P2) LEAD SCREW - NATIVE JOURNAL", "M0-001")
    x0, y0, total = 70, 360, 620
    thread_len = total * 158 / 188
    c.setStrokeColor(BLUE)
    c.setLineWidth(2)
    c.rect(x0, y0 - 18, thread_len, 36, fill=0, stroke=1)
    c.rect(x0 + thread_len, y0 - 18, total - thread_len, 36, fill=0, stroke=1)
    c.setStrokeColor(GRAY)
    for x in range(int(x0 + 8), int(x0 + thread_len - 5), 12):
        c.line(x, y0 - 18, x + 12, y0 + 18)
    c.setFillColor(INK)
    c.setFont("Helvetica-Bold", 9)
    c.drawCentredString(x0 + thread_len / 2, y0 + 32,
                        "TR8x8(P2), 4 START, RH - 158.00")
    c.drawCentredString(x0 + thread_len + (total - thread_len) / 2,
                        y0 + 32, "NATIVE OD8 h9 JOURNAL - 30.00")
    _table(c, 70, 270, [175, 190, 250], ["FEATURE", "NOMINAL", "CONTROL"], [
        ["Overall length", "188.00", "+/-0.10"],
        ["Thread", "Tr8x8(P2), DIN 103 7e", "4-start RH; gauge to production nut"],
        ["Journal", "OD8 h9 x 30.00", "Ra <=0.8; axis runout <=0.03"],
        ["Straightness", "188 span", "<=0.10 total"],
        ["Material", "AISI 304", "Passivated; clean and burr-free"],
        ["Ends", "C0.5 max", "Keep runout outside collar/bearing seat"],
    ])
    _notes(c, 470, 135, 285, "DO NOT SUBSTITUTE", [
        "The journal is retained from the original OD8 blank before threading.",
        "Do not cut fully threaded OD8 stock and claim the result is an OD8 journal.",
        "Supply material cert, inspection dimensions, and mating-nut identification with quote.",
    ])
    _footer(c, "t8x8_leadscrew_188_journal30")
    c.save()


def flyer_tube_pdf(path: Path):
    c = canvas.Canvas(str(path), pagesize=PAGE)
    _header(
        c,
        "FLYER HOLLOW SHAFT - STOCK D10 NECK",
        "M2-001",
        rev="D",
        date="2026-07-11",
    )
    x0, x1, y = 70, 700, 375
    c.setStrokeColor(BLUE)
    c.setLineWidth(2)
    span = x1 - x0
    total = 79.0
    neck = span * 18.50 / total
    rear_to_arm = span * 64.75 / total
    c.rect(x0, y - 35, span, 70, fill=0, stroke=1)
    c.setFillColor(colors.HexColor("#F7B267"))
    c.rect(x0, y - 28, neck, 56, fill=1, stroke=0)
    c.rect(
        x0 + rear_to_arm - span * 2.5 / total,
        y - 35,
        span * 5.0 / total,
        5,
        fill=1,
        stroke=0,
    )
    c.setFillColor(INK)
    c.setFont("Helvetica-Bold", 9)
    c.drawString(x0, y + 52, "REAR DATUM A")
    c.drawRightString(x1, y + 52, "FRONT")
    c.setFont("Helvetica", 8)
    c.drawString(x0 + 8, y - 54, "ROUND D10 h6 SEAT L18.50")
    c.drawCentredString(x0 + rear_to_arm, y - 54, "ARM FLATS @ 64.75 FROM A")
    _table(c, 70, 305, [125, 115, 90, 240],
           ["FEATURE", "FROM DATUM A", "NORMAL", "CONTROL"], [
        ["D10 clamp seat", "0.00..18.50", "axis", "OD9.991-10.000 h6; round, no pulley flats; ID6.000-6.030"],
        ["ID transition", "18.50..21.50", "axis", "coaxial ID6-to-ID9 cone; polish"],
        ["Arm flat 1", "64.75 +/-0.05", "-Y", "depth 0.30 +/-0.03; axial L5.00 +/-0.10"],
        ["Arm flat 2", "64.75 +/-0.05", "+X", "90 deg +/-0.5 deg from arm flat 1"],
        ["Shaft", "L79.00 +/-0.05", "axis", "main OD11.980-12.000; main ID9.000-9.050"],
    ])
    _notes(c, 70, 145, 685, "MACHINING, FIT, AND WIRE-SURFACE NOTES", [
        "6061-T6 machined hollow bar; total indicated runout <=0.05 over the bearing span. The shoulder at 18.50 is the stock-pulley axial datum.",
        "The full 18.50 mm pulley through-bore must remain OD10 h6 and round. NBK also permits h7, but Rev D selects h6; record three-axis micrometer readings before clamp assembly.",
        "The ID6 neck leaves 2.00 mm nominal and 1.980 mm minimum radial wall at drawing limits. Blend coaxially to the controlled ID9 over 3.00 mm; the wire centerline stays straight and has no forced bend.",
        "Round and polish the rear ID6 and front ID9 mouths to R0.5 minimum and Ra <=0.4 micrometre; polish the transition, then clean all chips and abrasive.",
        "Use only stock NBK P30-3GT-BLP-6C-10 with its supplied M2 split-clamp bolt at the manufacturer torque. No M3 pulley flats or set screws are permitted.",
    ])
    _footer(c, "flyer_shaft_d10_id6_to_id9_l79")
    c.save()


def shaft8_holder_pdf(path: Path):
    c = canvas.Canvas(str(path), pagesize=PAGE)
    _header(c, "DEDICATED 8 MM STATOR SHAFT SOCKET HOLDER", "M1-002")

    # Longitudinal section.  The top/work datum is at the left; the shared
    # bearing/coupling shank extends to the right.  This orientation keeps all
    # axial station dimensions readable on one page.
    x0, axis_y, scale = 55.0, 390.0, 4.05
    body_l = 16.0 * scale
    shank_l = 100.0 * scale
    body_r = 8.0 * scale
    shank_r = 4.0 * scale
    socket_l = 14.0 * scale
    socket_r = 4.05 * scale

    c.setStrokeColor(BLUE)
    c.setLineWidth(2)
    c.rect(x0, axis_y - body_r, body_l, 2 * body_r,
           fill=0, stroke=1)
    c.rect(x0 + body_l, axis_y - shank_r, shank_l, 2 * shank_r,
           fill=0, stroke=1)
    c.setFillColor(colors.white)
    c.rect(x0 - 1, axis_y - socket_r, socket_l + 1, 2 * socket_r,
           fill=1, stroke=0)
    c.setStrokeColor(BLUE)
    c.line(x0, axis_y - 4.30 * scale,
           x0 + scale, axis_y - socket_r)
    c.line(x0, axis_y + 4.30 * scale,
           x0 + scale, axis_y + socket_r)
    c.line(x0 + scale, axis_y - socket_r,
           x0 + socket_l, axis_y - socket_r)
    c.line(x0 + scale, axis_y + socket_r,
           x0 + socket_l, axis_y + socket_r)
    c.line(x0 + socket_l, axis_y - socket_r,
           x0 + socket_l, axis_y + socket_r)
    c.setStrokeColor(GRAY)
    c.setDash(4, 3)
    c.line(x0 - 10, axis_y, x0 + body_l + shank_l + 8, axis_y)
    c.setDash()

    # Radial M4 port stations.  One is shown in-section and the orthogonal
    # port is identified by its station/clocking callout.
    for depth, label in ((5.0, "+X PORT"), (10.0, "+Y PORT, 90 DEG")):
        xx = x0 + depth * scale
        c.setStrokeColor(colors.HexColor("#F29E4C"))
        c.setLineWidth(3)
        c.line(xx, axis_y + socket_r, xx, axis_y + body_r + 9)
        c.setFillColor(INK)
        c.setFont("Helvetica-Bold", 7.5)
        if depth == 5.0:
            c.drawRightString(xx - 5, axis_y + body_r + 14,
                              "+X PORT @ 5")
        else:
            c.drawString(xx + 5, axis_y + body_r + 28,
                         "+Y PORT @ 10, 90 DEG")

    c.setFillColor(INK)
    c.setFont("Helvetica-Bold", 9)
    c.drawString(x0, axis_y + body_r + 35, "TOP DATUM A")
    c.drawString(x0 + body_l + 145, axis_y + shank_r + 18,
                 "OD8 BEARING / COUPLING SHANK - 100.00")
    c.drawString(x0 + 8, axis_y - body_r - 22,
                 "OD16 CLAMP BODY - 16.00")
    c.setFont("Helvetica", 8)
    c.drawString(x0 + 4, axis_y + 5, "ID8.10 SOCKET, DEPTH 14.00")
    c.drawRightString(x0 + body_l + shank_l, axis_y - shank_r - 18,
                      "OVERALL 116.00")

    # End view makes the two orthogonal ports and their relationship to the
    # socket unambiguous.
    ex, ey, es = 670.0, 390.0, 7.0
    c.setStrokeColor(BLUE)
    c.setLineWidth(2)
    c.circle(ex, ey, 8.0 * es, fill=0, stroke=1)
    c.circle(ex, ey, 4.05 * es, fill=0, stroke=1)
    c.setStrokeColor(colors.HexColor("#F29E4C"))
    c.setLineWidth(4)
    c.line(ex + 4.05 * es, ey, ex + 8.0 * es, ey)
    c.line(ex, ey + 4.05 * es, ex, ey + 8.0 * es)
    c.setFillColor(INK)
    c.setFont("Helvetica-Bold", 9)
    c.drawCentredString(ex, ey - 78, "TOP VIEW AT DATUM A")
    c.setFont("Helvetica", 7.5)
    c.drawCentredString(ex, ey - 90, "PORTS CLOCKED 90 DEG +/-1 DEG")

    _table(c, 45, 255, [105, 85, 125, 165],
           ["FEATURE", "NOMINAL", "TOLERANCE", "CONTROL"], [
        ["Overall length", "116.00", "+/-0.10", "datum A to shank end"],
        ["Clamp body", "OD16 x 16", "OD +0/-0.03; L +/-0.05", "break OD edges C0.2"],
        ["Shared shank", "OD8 x 100", "-0.005/-0.015; L +/-0.05", "Ra <=0.8; straight <=0.05"],
        ["Shaft socket", "ID8.10 x 14", "+0.03/0; depth +0/-0.05", "Ra <=1.6; bottom flat"],
        ["Mouth lead-in", "ID8.60 x 1", "reference", "smooth blend; no sharp lip"],
        ["Clamp ports", "2x M4x0.7-6H", "stations +/-0.05", "5 and 10 from datum A"],
        ["Port clocking", "+X / +Y", "90 deg +/-1 deg", "tap through wall to socket"],
        ["Material", "4140 prehard", "28-32 HRC", "machine from one piece"],
    ], 18)
    _notes(c, 540, 245, 215, "MACHINING AND USE NOTES", [
        "Custom part; no supplier SKU. Quote the paired STEP and drawing.",
        "Use two M4x5 cup-point screws; matching shaft flats are recommended.",
        "Deburr both tap breakthroughs and polish the socket mouth.",
        "Socket-to-shank total indicated runout <=0.03 after final finish.",
        "Qualify screw torque and shaft marking on scrap before production.",
    ])
    _footer(c, "shaft8_socket_holder")
    c.save()


def spacer_pdf(path: Path):
    c = canvas.Canvas(str(path), pagesize=PAGE)
    _header(c, "BEARING RETENTION SPACERS AND COLLARS", "RET-001")
    rows = [
        ["M0 fixed collar", "18 body x7 + 11.8 pilot x2", "8.05", "9.00", "303 SS; radial M3"],
        ["M0 inner shim", "12.00", "8.10", "1.00", "301/304 SS"],
        ["M1 outer spacer", "21.80", "18.20", "16.00", "6061-T6"],
        ["M1 inner spacer", "17.80", "8.05", "16.00", "6061-T6"],
        ["M1 lower spacer", "17.80", "8.05", "12.00", "6061-T6"],
        ["M1 upper collar", "16.00", "8.05", "9.00", "303 SS; radial M3"],
        ["M2 outer spacer", "27.80", "22.00 +0.05/0", "11.00", "6061-T6"],
        ["M2 rear shim", "18.00", "12.05", "0.50", "301/304 SS"],
        ["M2 center spacer", "17.80 +0/-0.05", "12.05", "11.00", "6061-T6"],
        ["M2 front spacer", "18.00", "12.05", "4.00", "6061-T6"],
    ]
    _table(c, 45, 510, [150, 155, 120, 80, 190],
           ["PART", "OD / PILOT", "ID", "LENGTH", "MATERIAL / NOTE"], rows, 20)
    _notes(c, 45, 255, 710, "COMMON TURNING AND INSPECTION NOTES", [
        "Unless individually toleranced: diameters +/-0.05; lengths +/-0.03; face parallelism <=0.02.",
        "Bearing-abutment faces Ra <=1.6; remove burrs without rolling an edge onto the face.",
        "M2 unilateral diameters preserve the validated 2.10 nominal radial clearance.",
        "ID12.05 parts require measured flyer tube OD <=12.02; otherwise ream as a matched set.",
        "DIN 472 retaining rings are catalog hardware and are not included in this custom packet.",
    ])
    _footer(c, "m0_fixed_collar (packet contains individual STEP files)")
    c.save()


def successor_custom_parts_pdf(path: Path):
    """Three-page fail-closed RFQ index for the frozen successor STEP set."""

    import integrated_release_candidate as candidate
    import retained_flyer_peek_guide_successor as flyer

    solution = candidate.integrated_balance_solution()
    rear = list(map(float, solution["rear_slug_lengths_mm"]))
    front = float(solution["front_trim_common_thickness_mm"])
    pages = 3
    c = canvas.Canvas(str(path), pagesize=PAGE)

    _header(c, "SUCCESSOR PEEK WIRE-CONTACT PARTS", "WG-010", 1, pages,
            rev="A", date="2026-07-11")
    _table(c, 35, 500, [205, 70, 155, 315],
           ["STEP PART", "QTY", "MATERIAL", "CONTROLLED FUNCTION"], [
        ["flyer_peek_guide_bell", "1", "Natural unfilled PEEK",
         "One-piece ID0.60 polished bore, R3.25 root elbow and integral exit bell"],
        ["stator_short_leadin_cap_front", "1", "Natural unfilled PEEK",
         "Front 24-sector cap with open short lead-ins; paired M2 retention"],
        ["stator_short_leadin_cap_rear", "1", "Natural unfilled PEEK",
         "Rear 24-sector cap with open short lead-ins; paired M2 retention"],
        ["active_sector_peek_guide_front", "1", "Natural unfilled PEEK",
         "M0-following front capture shoe with open bowls and keyed pads"],
        ["active_sector_peek_guide_rear", "1", "Natural unfilled PEEK",
         "M0-following rear capture shoe with open bowls and keyed pads"],
    ], 25)
    _notes(c, 35, 290, 720, "RFQ AND RELEASE BLOCKERS", [
        "The paired STEP is the exact shape authority. Do not close, bridge, or simplify any polished open wire channel.",
        "Quote only natural unfilled PEEK. State exact resin grade, filler declaration, lot certificate, machining process, achievable finish, and dimensional inspection method.",
        "Wire-contact surfaces require supplier DFM review, burr-free polish, and later abrasion, dielectric, varnish and 60 N hot coupons. This packet is not production authorization.",
        "No price or supplier is selected. Return a marked-up drawing and quote before any order state may advance.",
    ])
    _footer(c, "flyer_peek_guide_bell",
            authority_dir="out/custom/successor/step")
    c.showPage()

    _header(c, "SUCCESSOR MACHINED STRUCTURE", "M2-010", 2,
            pages, rev="B", date="2026-07-12")
    _table(c, 35, 500, [205, 70, 165, 305],
           ["STEP PART", "QTY", "MATERIAL", "CONTROLLED FUNCTION"], [
        ["active_sector_aluminum_yoke", "1", "Certified 6061-T6 only",
         "One-solid keyed M0 carriage yoke; four M4 tower stacks and four M3 guide stacks"],
    ], 28)
    _notes(c, 35, 355, 720, "RFQ AND RELEASE BLOCKERS", [
        "Preserve all keyed datums and M3/M4 hole positions from the yoke STEP. No decorative substitutions are allowed.",
        "Supplier must confirm certified 6061-T6 and return heat treatment, machining tolerance, concentricity, runout, balance and surface-treatment proposal. No alloy or temper substitution is pre-authorized; prices remain TBD.",
        "The selected flyer pulley is the separate stock NBK P30-3GT-BLP-6C-10 and is not a custom machined part in this packet.",
        "All M3 and M4 yoke fasteners are ordered separately. Motor-side BNW witnesses remain outside this packet and have purchase quantity zero.",
    ])
    _footer(c, "active_sector_aluminum_yoke",
            authority_dir="out/custom/successor/step")
    c.showPage()

    _header(c, "SERIALIZED ASTM-B777 BALANCE TRIMS", "M2-011", 3, pages,
            rev="A", date="2026-07-11")
    rows = []
    for index, value in enumerate(rear, 1):
        rows.append([
            f"balance_b777_rear_{index}", "1", "Rear annular stack",
            f"Solved axial length {value:.6f} mm",
        ])
    rows.extend([
        ["balance_b777_front_left", "1", "OD6 / ID2.2 annulus",
         f"Common thickness {front:.6f} mm"],
        ["balance_b777_front_right", "1", "OD6 / ID2.2 annulus",
         f"Common thickness {front:.6f} mm"],
    ])
    _table(c, 35, 505, [220, 55, 180, 290],
           ["STEP PART", "QTY", "GEOMETRY", "SOLVED CONTROL"], rows, 25)
    _notes(c, 35, 285, 720, "MATERIAL, WEIGHING AND RELEASE BLOCKERS", [
        "Quote ASTM-B777 tungsten heavy alloy but state grade, certified density, chemistry, lot and achievable dimensional tolerance. No grade substitution is pre-authorized.",
        "Machine each serialized trim to its own STEP. Deburr without rounding bearing faces or changing the annular bore. Weigh each finished trim and record actual mass and serial number.",
        "Four rear trims use separate printed retainers, M3x6 countersunk screws and 94459A130 inserts. Two front trims use M2x8 screws, washers and standard M2 inserts.",
        "The six solved dimensions are nominal digital balance values. Final installed G2.5 balance, retention pull tests and hot 300 RPM endurance remain mandatory before use.",
    ])
    _footer(c, "balance_b777_rear_1_rear_left",
            authority_dir="out/custom/successor/step")
    c.save()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--include-successor",
        action="store_true",
        help="emit the selected rev6 active-sector successor RFQ PDF",
    )
    args = parser.parse_args(argv)
    OUT.mkdir(parents=True, exist_ok=True)
    generators = {
        "fixed_eyelet_id4_od9_t3.pdf": fixed_eyelet_pdf,
        "shaft_wrap_sleeve_family.pdf": sleeve_pdf,
        "t8x8_leadscrew_188_journal30.pdf": t8_pdf,
        "flyer_shaft_d10_id6_to_id9_l79.pdf": flyer_tube_pdf,
        "shaft8_socket_holder.pdf": shaft8_holder_pdf,
        "bearing_retention_spacers.pdf": spacer_pdf,
    }
    successor_path = OUT / "successor_custom_parts_rfq.pdf"
    if args.include_successor:
        generators[successor_path.name] = successor_custom_parts_pdf
    elif successor_path.exists():
        successor_path.unlink()
    for obsolete_name in (
        "tip_toroid_guide.pdf",
        "flyer_tube_12x9x80_4flats.pdf",
    ):
        obsolete = OUT / obsolete_name
        if obsolete.exists():
            obsolete.unlink()
    for name, generate in generators.items():
        path = OUT / name
        generate(path)
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
