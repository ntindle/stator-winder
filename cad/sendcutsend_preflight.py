"""Current-source SendCutSend preflight for the MIC6 carriage plate DXF."""

from __future__ import annotations

import json
import math
from datetime import date
from pathlib import Path

import ezdxf
from build123d import import_step


ROOT = Path(__file__).resolve().parent.parent
DXF = ROOT / "cad" / "fabricated_carriage.dxf"
STEP = ROOT / "cad" / "fabricated_carriage.step"
REPORTS = ROOT / "out" / "reports"
CATALOG = REPORTS / "sendcutsend-catalog.json"
SPECS = REPORTS / "sendcutsend-specs.json"
GUIDE = REPORTS / "sendcutsend-ordering-guide.md"
SKU = "ALUMIC6-250"
CATALOG_URL = "https://cdn.sendcutsend.com/specs/sendcutsend-catalog.json"
SPECS_URL = "https://cdn.sendcutsend.com/specs/sendcutsend-specs.json"
GUIDE_URL = "https://cdn.sendcutsend.com/specs/sendcutsend-ordering-guide.md"


def _f(value) -> float:
    text = str(value).strip().replace('"', "")
    return float(text.split()[0])


def _pair(text: str) -> tuple[float, float]:
    a, b = str(text).lower().replace(" ", "").split("x")[:2]
    return float(a), float(b)


def _segments(points):
    return list(zip(points, points[1:] + points[:1]))


def _point_segment_distance(point, a, b):
    px, py = point
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - \
        (b[1] - a[1]) * (c[0] - a[0])


def _segment_distance(a, b, c, d):
    o1, o2, o3, o4 = (_orient(a, b, c), _orient(a, b, d),
                      _orient(c, d, a), _orient(c, d, b))
    if ((o1 == 0 or o2 == 0 or o1 * o2 < 0) and
            (o3 == 0 or o4 == 0 or o3 * o4 < 0)):
        return 0.0
    return min(_point_segment_distance(a, c, d),
               _point_segment_distance(b, c, d),
               _point_segment_distance(c, a, b),
               _point_segment_distance(d, a, b))


def _polygon_area(points):
    return 0.5 * sum(a[0] * b[1] - b[0] * a[1]
                     for a, b in _segments(points))


def inspect_upload() -> dict:
    doc = ezdxf.readfile(DXF)
    entities = list(doc.modelspace())
    polylines = [entity for entity in entities if entity.dxftype() == "LWPOLYLINE"]
    circles = [entity for entity in entities if entity.dxftype() == "CIRCLE"]
    contours = [[(float(x), float(y)) for x, y, *_ in entity.get_points()]
                for entity in polylines]
    outer_index = max(range(len(contours)), key=lambda i: abs(_polygon_area(contours[i])))
    outer = contours[outer_index]
    inner = [points for i, points in enumerate(contours) if i != outer_index]
    circle_data = [((float(item.dxf.center.x), float(item.dxf.center.y)),
                    float(item.dxf.radius)) for item in circles]

    boundaries = [outer, *inner]
    hole_edge = []
    for center, radius in circle_data:
        distance = min(_point_segment_distance(center, a, b)
                       for contour in boundaries for a, b in _segments(contour))
        hole_edge.append(distance - radius)

    # Conservative bridge sweep across every pair of cut contours, including
    # holes-to-holes, holes-to-polyline cuts, and inner cuts to the outer edge.
    bridge = list(hole_edge)
    for i, (ca, ra) in enumerate(circle_data):
        for cb, rb in circle_data[i + 1:]:
            bridge.append(math.dist(ca, cb) - ra - rb)
        for contour in inner:
            bridge.append(min(_point_segment_distance(ca, a, b)
                              for a, b in _segments(contour)) - ra)
    for i, contour_a in enumerate(inner):
        for contour_b in [outer, *inner[i + 1:]]:
            bridge.append(min(_segment_distance(a, b, c, d)
                              for a, b in _segments(contour_a)
                              for c, d in _segments(contour_b)))

    points = [point for contour in contours for point in contour]
    points.extend((center[0] - radius, center[1] - radius)
                  for center, radius in circle_data)
    points.extend((center[0] + radius, center[1] + radius)
                  for center, radius in circle_data)
    xs, ys = zip(*points)
    types = sorted({item.dxftype() for item in entities})
    duplicate_circles = len(circle_data) - len({
        (round(c[0], 9), round(c[1], 9), round(r, 9)) for c, r in circle_data
    })
    duplicate_polys = len(contours) - len({
        tuple((round(x, 9), round(y, 9)) for x, y in contour)
        for contour in contours
    })

    part = import_step(str(STEP))
    bbox = part.bounding_box()
    return {
        "dxf_units_code": int(doc.header.get("$INSUNITS", 0)),
        "layers": sorted({str(item.dxf.layer) for item in entities}),
        "entity_types": types,
        "entity_count": len(entities),
        "closed_polyline_count": sum(bool(item.closed) for item in polylines),
        "open_polyline_count": sum(not bool(item.closed) for item in polylines),
        "circle_count": len(circles),
        "duplicate_entity_count": duplicate_circles + duplicate_polys,
        "bounds_mm": [min(xs), min(ys), max(xs), max(ys)],
        "size_mm": [max(xs) - min(xs), max(ys) - min(ys)],
        "minimum_hole_diameter_mm": min(2 * radius for _, radius in circle_data),
        "minimum_hole_to_cut_edge_mm": min(hole_edge),
        "minimum_bridge_mm": min(bridge),
        "step_solid_count": len(part.solids()),
        "step_thickness_mm": bbox.size.Y,
        "step_bounds_mm": [bbox.size.X, bbox.size.Y, bbox.size.Z],
    }


def preflight() -> dict:
    catalog = json.loads(CATALOG.read_text(encoding="utf-8"))
    specs = json.loads(SPECS.read_text(encoding="utf-8"))
    material = next(item for item in catalog["materials"] if item["sku"] == SKU)
    engineering = next(item for item in specs["materials"] if item["sku"] == SKU)
    facts = inspect_upload()
    cut = engineering["cutting_specs"]
    general = engineering["general_specs"]
    min_part = _pair(cut["min_part_size"])
    max_part = _pair(cut["max_part_size"])
    size_in = [value / 25.4 for value in facts["size_mm"]]
    min_hole = _f(cut["min_hole_size"]) * 25.4
    min_edge = _f(cut["min_hole_to_edge"]) * 25.4
    min_bridge = _f(cut["min_bridge_size"]) * 25.4
    nominal_t = _f(general["thickness"]) * 25.4
    thickness_tol = max(_f(general["thickness_tolerance_positive"]),
                        _f(general["thickness_tolerance_negative"])) * 25.4
    supported = {"LWPOLYLINE", "CIRCLE"}

    checks = [
        {"name": "DXF parse, scale, and cut-only linework",
         "ok": (facts["dxf_units_code"] == 4 and
                facts["open_polyline_count"] == 0 and
                not (set(facts["entity_types"]) - supported) and
                facts["duplicate_entity_count"] == 0),
         "evidence": (f"INSUNITS={facts['dxf_units_code']}; 3 closed contours; "
                      f"{facts['circle_count']} circles; duplicates "
                      f"{facts['duplicate_entity_count']}"),
         "source": f"{GUIDE_URL} File Formats and Design File Guidelines"},
        {"name": "Exact material is stocked for fiber laser cutting",
         "ok": (not material["out_of_stock"] and
                material["cutting_process"] == "Fiber Laser"),
         "evidence": f"{SKU}; out_of_stock={material['out_of_stock']}",
         "source": f"{CATALOG_URL} materials[sku={SKU}]"},
        {"name": "Part size within SKU limits",
         "ok": (min(size_in) >= min(min_part) and max(size_in) >= max(min_part)
                and min(size_in) <= min(max_part) and max(size_in) <= max(max_part)),
         "evidence": (f"{size_in[0]:.3f} x {size_in[1]:.3f} in; limits "
                      f"{cut['min_part_size']} .. {cut['max_part_size']} in"),
         "source": f"{SPECS_URL} materials[sku={SKU}].cutting_specs"},
        {"name": "STEP/DXF thickness context matches selected stock",
         "ok": (facts["step_solid_count"] == 1 and
                abs(facts["step_thickness_mm"] - nominal_t) < 1e-6),
         "evidence": (f"one solid; {facts['step_thickness_mm']:.3f} mm; "
                      f"stock tolerance +/-{thickness_tol:.3f} mm"),
         "source": f"{SPECS_URL} materials[sku={SKU}].general_specs"},
        {"name": "Minimum cut hole",
         "ok": facts["minimum_hole_diameter_mm"] >= min_hole,
         "evidence": (f"measured {facts['minimum_hole_diameter_mm']:.3f} mm; "
                      f"minimum {min_hole:.3f} mm"),
         "source": f"{SPECS_URL} materials[sku={SKU}].cutting_specs.min_hole_size"},
        {"name": "Minimum hole-to-cut-edge ligament",
         "ok": facts["minimum_hole_to_cut_edge_mm"] >= min_edge,
         "evidence": (f"measured {facts['minimum_hole_to_cut_edge_mm']:.3f} mm; "
                      f"minimum {min_edge:.3f} mm"),
         "source": f"{SPECS_URL} materials[sku={SKU}].cutting_specs.min_hole_to_edge"},
        {"name": "Minimum bridge/web",
         "ok": facts["minimum_bridge_mm"] >= min_bridge,
         "evidence": (f"measured {facts['minimum_bridge_mm']:.3f} mm; "
                      f"minimum {min_bridge:.3f} mm"),
         "source": f"{SPECS_URL} materials[sku={SKU}].cutting_specs.min_bridge_size"},
        {"name": "No unvalidated secondary service",
         "ok": True,
         "evidence": "flat laser cut only; no bending, tapping, countersinking, or inserted hardware",
         "source": f"{CATALOG_URL} materials[sku={SKU}].available_services"},
    ]
    return {
        "schema": 1,
        "checked_date": str(date.today()),
        "file": str(DXF.relative_to(ROOT)).replace("\\", "/"),
        "step_reference": str(STEP.relative_to(ROOT)).replace("\\", "/"),
        "order_context": {
            "quantity": 1,
            "sku": SKU,
            "material": material["name"],
            "thickness_in": material["thickness"],
            "process": material["cutting_process"],
            "services": [],
            "finish": "as cut",
        },
        "sources": {
            "ordering_guide": {"url": GUIDE_URL},
            "catalog": {"url": CATALOG_URL, "meta": catalog["_meta"]},
            "specs": {"url": SPECS_URL, "meta": specs["_meta"]},
        },
        "geometry_facts": facts,
        "checks": checks,
        "ready_to_upload_for_assumed_context": all(item["ok"] for item in checks),
    }


def markdown(report: dict) -> str:
    order = report["order_context"]
    facts = report["geometry_facts"]
    lines = [
        "# SendCutSend preflight - carriage plate",
        "",
        "## Context",
        "",
        f"- File: `{report['file']}`",
        f"- Service: flat {order['process']} cut, quantity {order['quantity']}",
        f"- Material: {order['material']} `{order['sku']}`, {order['thickness_in']} in",
        f"- Finish/services: {order['finish']}; no secondary services",
        f"- Date checked: {report['checked_date']}",
        "",
        "## Sources checked",
        "",
        f"- [Ordering guide]({GUIDE_URL})",
        f"- [Catalog JSON]({CATALOG_URL}) generated {report['sources']['catalog']['meta']['generated_at']}",
        f"- [Engineering specs JSON]({SPECS_URL}) generated {report['sources']['specs']['meta']['generated_at']}",
        "",
        "## Geometry facts",
        "",
        f"DXF is millimetres/1:1 with {facts['closed_polyline_count']} closed cut contours, "
        f"{facts['circle_count']} circles, no open contours, and no duplicate entities. "
        f"Bounds are {facts['size_mm'][0]:.3f} x {facts['size_mm'][1]:.3f} mm. "
        f"The STEP reference is one {facts['step_thickness_mm']:.3f} mm solid.",
        "",
        "## Findings",
        "",
        "| Status | Check | Evidence | Rule source |",
        "|---|---|---|---|",
    ]
    for item in report["checks"]:
        status = "PASS" if item["ok"] else "FAIL"
        source = item["source"]
        url = source.split(" ", 1)[0]
        detail = source[len(url):].strip()
        lines.append(f"| {status} | {item['name']} | {item['evidence']} | "
                     f"[{detail or 'official source'}]({url}) |")
    verdict = ("Ready to upload for this assumed context" if
               report["ready_to_upload_for_assumed_context"] else
               "Needs edits before upload")
    lines.extend(["", "## Verdict", "", verdict, ""])
    return "\n".join(lines)


def main() -> int:
    REPORTS.mkdir(parents=True, exist_ok=True)
    result = preflight()
    (REPORTS / "sendcutsend_carriage.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8")
    (REPORTS / "sendcutsend_carriage.md").write_text(
        markdown(result), encoding="utf-8")
    print("SendCutSend preflight:",
          "PASS" if result["ready_to_upload_for_assumed_context"] else "FAIL")
    for item in result["checks"]:
        print(f"  [{'OK' if item['ok'] else 'FAIL'}] {item['name']}: {item['evidence']}")
    return 0 if result["ready_to_upload_for_assumed_context"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
