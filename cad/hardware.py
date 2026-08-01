"""Lightweight, dimensioned hardware for the stator-winder assembly.

This module is intentionally independent from :mod:`assembly`, :mod:`cots`,
and :mod:`printed`.  It is a staging library for the hardware-detail pass:
root may import and place occurrences without making the current assembly pay
for threaded BREP detail.  All dimensions are millimetres.

Geometry policy
---------------

* Standard screws, nuts, washers, and heat-set inserts use ``bd_warehouse``.
  ``thread_detail=False`` (the default) preserves the exact mounting and outer
  envelope while omitting helical threads for assembly performance.
* Catalog-specific parts that bd_warehouse does not provide are compact
  parametric solids.  Their mounting interfaces and catalog envelope are
  exact; cosmetic knurling, nylon colour breaks, spring coils, and thread
  helices are intentionally omitted.
* Screw local frame: the underside of the head is ``z=0``; the shank extends
  along ``-Z`` and the head along ``+Z``.  Nuts/washers/collars start at
  ``z=0`` and extend along ``+Z``.  ``place()`` maps local ``+Z`` to a named
  machine axis.

Catalog / standard references
-----------------------------

* ISO 4762 socket-head cap screws, ISO 4026 set screws, ISO 4032 hex nuts,
  ISO 7089 plain washers (DIN 125-A equivalent): ``bd_warehouse`` 0.1 data.
* McMaster 94459A140 / 94459A130 M3 heat-set inserts:
  ``bd_warehouse``'s McMaster-Carr parameter table (OD4.7, installed lengths
  5.7 / 4.3).  McMaster 94459A769 retains the same OD4.7 interface at 3.4 mm
  installed length and is modeled as an exact lightweight envelope because
  that catalog row is absent from bd_warehouse 0.1.
* McMaster 90265A420, ISO 7379: 3 mm shoulder diameter, 16 mm shoulder,
  M2.5x0.45 thread x 4, head OD 5 x 2.
* McMaster 90265A115: 3 mm shoulder diameter, 10 mm shoulder,
  M2x0.4 thread x 4, head OD 5 x 2.
* McMaster 96654A127: 5 mm shoulder x 10, M4x0.7 thread x 5,
  head OD 9 x 4.  This is the correct fit for the current dancer's OD5.2
  pivot bore; "M5 shoulder screw" in the audit means a 5 mm shoulder, not an
  M5 thread.
* MISUMI HBKTST5 5-series/slot-6 right-angle bracket: 20 mm wide,
  25x25 legs, 5 mm wall, two OD5.5 mounting holes; catalog load 833 N and
  specified mates CBM5-10 + HNTT5-5.  Alignment nibs are cosmetic and omitted.
* MISUMI HNTA5 post-assembly insertion nut, 5-series/slot-6: L15 x B8,
  A5.8 slot neck, E2.5 body and T3.2 overall height, M3/M4/M5 interface.
  These are the manufacturer's HFS5 dimensions; the old 11.5 x 7.8 x 6.3
  envelope was an accidental mixture of HFS6 table values.
* ISO 10511 / DIN 985 prevailing-torque nut and DIN 315 wing-nut nominal
  envelopes.  Internal nylon/thread detail is omitted.
* DIN 705-A set-screw shaft collars; DIN 471/472 nominal retaining-ring
  envelopes; DIN 988 shim/thrust washers.

``HARDWARE_SCHEDULE`` is the assembly demand from the 2026-07-10 hardware
audit.  ``procurement_schedule()`` aggregates identical SKUs and applies the
one-time spares in ``SPARES_BY_SKU``.  Do not add spares to the assembly.
"""

from __future__ import annotations

from collections import defaultdict
from copy import copy, deepcopy
from functools import lru_cache
import json
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from build123d import (
    Align,
    Box,
    BuildPart,
    BuildSketch,
    Compound,
    Cylinder,
    Part,
    Pos,
    RegularPolygon,
    Rot,
    extrude,
)
from bd_warehouse.fastener import (
    CounterSunkScrew,
    HeatSetNut,
    HexNut,
    PlainWasher,
    SetScrew,
    SocketHeadCapScrew,
)


CTR = (Align.CENTER, Align.CENTER, Align.CENTER)
MIN = (Align.CENTER, Align.CENTER, Align.MIN)

_PITCH = {
    "M2": 0.4,
    "M2.5": 0.45,
    "M3": 0.5,
    "M4": 0.7,
    "M5": 0.8,
    "M6": 1.0,
    "M8": 1.25,
}


def _thread_size(size: str) -> str:
    try:
        pitch = _PITCH[size.upper()]
    except KeyError as exc:
        raise ValueError(f"unsupported metric size {size!r}") from exc
    return f"{size.upper()}-{pitch:g}"


def _fresh(part: Part, label: str | None) -> Part:
    """Copy cached geometry so occurrence labels never mutate the cache."""
    result = copy(part)
    if label:
        result.label = label
    return result


def _hex_prism(across_flats: float, height: float) -> Part:
    with BuildPart() as body:
        with BuildSketch():
            RegularPolygon(across_flats / 2.0, 6, major_radius=False)
        extrude(amount=height)
    return body.part


# ---------------------------------------------------------------------------
# ISO fasteners backed by bd_warehouse


@lru_cache(maxsize=None)
def _socket_head_cached(size: str, length: float, thread_detail: bool) -> Part:
    return SocketHeadCapScrew(
        _thread_size(size), float(length), fastener_type="iso4762",
        simple=not thread_detail,
    )


def socket_head_cap_screw(
    size: str,
    length: float,
    *,
    thread_detail: bool = False,
    label: str | None = None,
) -> Part:
    """ISO 4762 screw; head bearing plane at z=0, shank toward -Z."""
    return _fresh(_socket_head_cached(size, float(length), thread_detail),
                  label or f"iso4762_{size.lower()}x{length:g}")


@lru_cache(maxsize=None)
def _countersunk_cached(
    size: str, length: float, standard: str, thread_detail: bool,
) -> Part:
    return CounterSunkScrew(
        _thread_size(size), float(length), fastener_type=standard,
        simple=not thread_detail,
    )


def countersunk_screw(
    size: str,
    length: float,
    *,
    standard: str = "iso10642",
    thread_detail: bool = False,
    label: str | None = None,
) -> Part:
    """Metric countersunk screw with its top face at local ``z=0``.

    ``iso10642`` is used for M5 rear-post bases and ``iso14581`` for the
    M2 dancer spring anchor.  The placement convention matches the socket
    screws: the shank extends along local ``-Z``.
    """
    return _fresh(
        _countersunk_cached(size, float(length), standard, thread_detail),
        label or f"{standard}_{size.lower()}x{length:g}",
    )


@lru_cache(maxsize=None)
def _set_screw_cached(size: str, length: float, thread_detail: bool) -> Part:
    return SetScrew(
        _thread_size(size), float(length), fastener_type="iso4026",
        simple=not thread_detail,
    )


def set_screw(
    size: str,
    length: float,
    *,
    thread_detail: bool = False,
    label: str | None = None,
) -> Part:
    """ISO 4026 flat-point set screw, z=-length..0."""
    return _fresh(_set_screw_cached(size, float(length), thread_detail),
                  label or f"iso4026_{size.lower()}x{length:g}")


@lru_cache(maxsize=None)
def _hex_nut_cached(size: str, thread_detail: bool) -> Part:
    return HexNut(
        _thread_size(size), fastener_type="iso4032",
        simple=not thread_detail,
    )


def hex_nut(
    size: str,
    *,
    thread_detail: bool = False,
    label: str | None = None,
) -> Part:
    """ISO 4032 style-1 hex nut, z=0..height."""
    return _fresh(_hex_nut_cached(size, thread_detail),
                  label or f"iso4032_{size.lower()}")


@lru_cache(maxsize=None)
def _washer_cached(size: str, standard: str) -> Part:
    return PlainWasher(size.upper(), fastener_type=standard)


def plain_washer(
    size: str,
    *,
    standard: str = "iso7089",
    label: str | None = None,
) -> Part:
    """ISO 7089 normal-series washer (DIN 125-A equivalent by default)."""
    return _fresh(_washer_cached(size, standard),
                  label or f"{standard}_{size.lower()}")


@lru_cache(maxsize=None)
def _heat_set_cached(size: str, length: str, thread_detail: bool) -> Part:
    key = f"{size.upper()}-{_PITCH[size.upper()]:g}-{length.title()}"
    return HeatSetNut(
        key, fastener_type="McMaster-Carr", simple=not thread_detail,
    )


def heat_set_insert(
    size: str,
    *,
    length: str = "standard",
    thread_detail: bool = False,
    label: str | None = None,
) -> Part:
    """McMaster heat-set insert from bd_warehouse's catalog parameter table."""
    return _fresh(_heat_set_cached(size, length, thread_detail),
                  label or f"heat_set_{size.lower()}_{length.lower()}")


def heat_set_insert_m3_3p4(
    *,
    label: str = "mcmaster_94459a769",
) -> Part:
    """McMaster 94459A769 short M3 heat-set insert envelope.

    bd_warehouse's McMaster table stops at the 4.3 mm ``Short`` variant.
    The 3.4 mm installed-length catalog variant uses the same OD4.7 / M3
    interface, so retain that exact fit envelope without cosmetic knurling.
    Local frame is z=0..3.4, matching :func:`heat_set_insert`.
    """
    outer = Cylinder(4.7 / 2.0, 3.4,
                     align=(Align.CENTER, Align.CENTER, Align.MIN))
    bore = Pos(0, 0, -1.0) * Cylinder(
        3.0 / 2.0, 5.4,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    part = outer - bore
    part.label = label
    return part


# ---------------------------------------------------------------------------
# Catalog-specific lightweight solids


def angle_bracket_2020(label: str = "hbktst5_bracket") -> Part:
    """MISUMI HBKTST5 one-slot bracket for 2020/slot-6 extrusion.

    Local frame: bracket outside corner at y=z=0; width is centered on X.
    The two clearance-hole axes are +Z and +Y respectively.
    """
    width = 20.0
    leg = 25.0
    wall = 5.0
    hole_d = 5.5
    hole_offset = 12.0

    floor = Box(width, leg, wall,
                align=(Align.CENTER, Align.MIN, Align.MIN))
    upright = Box(width, wall, leg,
                  align=(Align.CENTER, Align.MIN, Align.MIN))
    floor_hole = Pos(0, hole_offset, -1) * Cylinder(
        hole_d / 2, wall + 2,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    # RotX(+90) sends local +Z toward machine -Y.  Start beyond +Y so the
    # cutter overshoots both bracket faces.
    upright_hole = Pos(0, wall + 1, hole_offset) * (
        Rot(90, 0, 0) * Cylinder(
            hole_d / 2, wall + 2,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
    )
    part = floor + upright - floor_hole - upright_hole
    part.label = label
    return part


def extra_low_head_screw_cbsa5_10(
    label: str = "misumi_cbsa5_10",
) -> Part:
    """MISUMI CBSA5-10 extra-low M5x10 screw collision envelope.

    CBSA is the current trivalent-chromate successor to discontinued CBSM
    with the same standard.  The M5 variant has an OD8 head no more than
    1.5 mm high.  Local convention matches the other screws: head bearing
    plane at z=0, shank z=-10..0, head z=0..1.5.
    """
    shank = Pos(0, 0, -10.0) * Cylinder(2.5, 10.0, align=MIN)
    head = Cylinder(4.0, 1.5, align=MIN)
    part = shank + head
    part.label = label
    return part


_TNUT_DATA = {
    # MISUMI HNTA5 post-assembly insertion body; same metal envelope,
    # different tap.  Official drawing fields: L=15, B=8, A=5.8,
    # E=2.5 and T=3.2 mm.
    "M3": {"length": 15.0, "width": 8.0, "neck_width": 5.8,
           "body_h": 2.5, "overall_h": 3.2,
           "thread_d": 3.0, "catalog": "HNTA5-3"},
    "M4": {"length": 15.0, "width": 8.0, "neck_width": 5.8,
           "body_h": 2.5, "overall_h": 3.2,
           "thread_d": 4.0, "catalog": "HNTA5-4"},
    "M5": {"length": 15.0, "width": 8.0, "neck_width": 5.8,
           "body_h": 2.5, "overall_h": 3.2,
           "thread_d": 5.0, "catalog": "HNTA5-5"},
}


def tnut_slot6(size: str, label: str | None = None) -> Part:
    """MISUMI HNTA5 post-assembly nut envelope for a 6 mm 2020 slot.

    Local X is across the slot (B/A); local Y is the extrusion-slot direction
    (L).  That convention matches the cardinal-axis placement helper and is
    essential now that the real nut is 15 mm long rather than nearly square.
    """
    size = size.upper()
    try:
        d = _TNUT_DATA[size]
    except KeyError as exc:
        raise ValueError("slot-6 post-assembly nut is supported for M3/M4/M5") from exc
    base = Box(d["width"], d["length"], d["body_h"], align=MIN)
    neck_h = d["overall_h"] - d["body_h"]
    neck = Pos(0, 0, d["body_h"]) * Box(
        d["neck_width"], d["length"], neck_h, align=MIN,
    )
    bore = Pos(0, 0, -1) * Cylinder(
        d["thread_d"] / 2, d["overall_h"] + 2,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    part = base + neck - bore
    part.label = label or d["catalog"].lower()
    return part


def tnut_slot6_short_m5(label: str | None = None) -> Part:
    """MISUMI HNTAJ5-5 short post-assembly nut for 12 mm hole pitch.

    Official HFS5 short-nut drawing: L10 x B8, A5.8 slot neck, E2.4 body,
    T3.2 overall.  It is used only at the paired M0 fixed-support holes where
    two L15 HNTA5 bodies would overlap by 3 mm.
    """
    length, width, neck_width = 10.0, 8.0, 5.8
    body_h, overall_h = 2.4, 3.2
    base = Box(width, length, body_h, align=MIN)
    neck = Pos(0, 0, body_h) * Box(
        neck_width, length, overall_h - body_h, align=MIN,
    )
    bore = Pos(0, 0, -1) * Cylinder(
        2.5, overall_h + 2,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    part = base + neck - bore
    part.label = label or "hntaj5-5"
    return part


_NYLOC = {
    # ISO 10511 / DIN 985 nominal envelope: across flats, total height.
    "M2": (4.0, 3.8),
    "M2.5": (5.0, 4.5),
    "M3": (5.5, 4.0),
    "M4": (7.0, 5.0),
    "M5": (8.0, 5.0),
    "M8": (13.0, 8.0),
}


def nyloc_nut(size: str, label: str | None = None) -> Part:
    """ISO 10511 / DIN 985 nominal prevailing-torque nut envelope."""
    size = size.upper()
    try:
        across_flats, height = _NYLOC[size]
        thread_d = float(size[1:])
    except (KeyError, ValueError) as exc:
        raise ValueError(f"unsupported nyloc size {size!r}") from exc
    metal_h = height * 0.72
    body = _hex_prism(across_flats, metal_h)
    collar = Pos(0, 0, metal_h) * Cylinder(
        across_flats * 0.38, height - metal_h,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    bore = Pos(0, 0, -1) * Cylinder(
        thread_d / 2, height + 2,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    part = body + collar - bore
    part.label = label or f"iso10511_{size.lower()}"
    return part


def shoulder_screw(
    shoulder_d: float,
    shoulder_length: float,
    thread_d: float,
    thread_length: float,
    head_d: float,
    head_h: float,
    *,
    label: str = "shoulder_screw",
) -> Part:
    """ISO-style shoulder screw, head bearing plane at z=0."""
    head = Cylinder(head_d / 2, head_h,
                    align=(Align.CENTER, Align.CENTER, Align.MIN))
    shoulder = Pos(0, 0, -shoulder_length) * Cylinder(
        shoulder_d / 2, shoulder_length,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    thread = Pos(0, 0, -(shoulder_length + thread_length)) * Cylinder(
        thread_d / 2, thread_length,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    part = head + shoulder + thread
    part.label = label
    return part


def shoulder_screw_90265a420(
    label: str = "mcmaster_90265a420",
) -> Part:
    """McMaster 90265A420: OD3 shoulder x16, M2.5 thread x4, OD5x2 head."""
    return shoulder_screw(3.0, 16.0, 2.5, 4.0, 5.0, 2.0, label=label)


def shoulder_screw_90265a115(
    label: str = "mcmaster_90265a115",
) -> Part:
    """McMaster 90265A115: OD3 shoulder x10, M2 thread x4, OD5x2 head."""
    return shoulder_screw(3.0, 10.0, 2.0, 4.0, 5.0, 2.0, label=label)


def dancer_pivot_shoulder_screw(
    label: str = "mcmaster_96654a127",
) -> Part:
    """McMaster 96654A127: OD5 shoulder x10, M4 thread x5, OD9x4 head."""
    return shoulder_screw(5.0, 10.0, 4.0, 5.0, 9.0, 4.0, label=label)


def thrust_washer(
    bore_d: float,
    od: float,
    thickness: float,
    *,
    label: str = "din988_thrust_washer",
) -> Part:
    """DIN 988 style shim/thrust washer, z=0..thickness."""
    outer = Cylinder(od / 2, thickness,
                     align=(Align.CENTER, Align.CENTER, Align.MIN))
    inner = Pos(0, 0, -1) * Cylinder(
        bore_d / 2, thickness + 2,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    part = outer - inner
    part.label = label
    return part


_DIN705 = {
    # bore: (OD, width, radial set-screw major diameter)
    6.0: (12.0, 8.0, 4.0),
    8.0: (16.0, 8.0, 4.0),
    10.0: (20.0, 10.0, 5.0),
    12.0: (22.0, 12.0, 6.0),
}


def shaft_collar(
    bore_d: float,
    *,
    split: bool = False,
    label: str | None = None,
) -> Part:
    """DIN 705-A nominal set-screw collar; optional split-collar slit."""
    try:
        od, width, set_d = _DIN705[float(bore_d)]
    except KeyError as exc:
        raise ValueError(f"unsupported DIN 705 bore {bore_d:g}") from exc
    part = Cylinder(od / 2, width,
                    align=(Align.CENTER, Align.CENTER, Align.MIN))
    part -= Pos(0, 0, -1) * Cylinder(
        bore_d / 2, width + 2,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    radial = Pos(0, 0, width / 2) * (
        Rot(0, 90, 0) * Cylinder(set_d / 2, od + 2, align=CTR)
    )
    part -= radial
    if split:
        part -= Pos(od / 4, 0, width / 2) * Box(
            od / 2 + 1, 0.8, width + 2, align=CTR,
        )
    part.label = label or f"din705_{bore_d:g}mm_collar"
    return part


_DIN471 = {
    # nominal shaft: (free ID, maximum OD, thickness)
    8.0: (7.4, 14.3, 0.8),
    12.0: (11.0, 19.0, 1.0),
}
_DIN472 = {
    # nominal bore: (minimum ID, free OD, thickness)
    22.0: (18.5, 23.1, 1.0),
    28.0: (24.6, 29.4, 1.2),
}


def retaining_ring_external(
    shaft_d: float,
    label: str | None = None,
) -> Part:
    """DIN 471 external retaining-ring nominal envelope with assembly gap."""
    try:
        inner_d, outer_d, thickness = _DIN471[float(shaft_d)]
    except KeyError as exc:
        raise ValueError(f"unsupported DIN 471 shaft {shaft_d:g}") from exc
    ring = Cylinder(outer_d / 2, thickness, align=MIN) - Cylinder(
        inner_d / 2, thickness + 2, align=CTR,
    )
    ring -= Pos(outer_d / 3, 0, thickness / 2) * Box(
        outer_d, 1.2, thickness + 2, align=CTR,
    )
    ring.label = label or f"din471_{shaft_d:g}mm"
    return ring


def retaining_ring_internal(
    bore_d: float,
    label: str | None = None,
) -> Part:
    """DIN 472 internal retaining-ring nominal envelope with assembly gap."""
    try:
        inner_d, outer_d, thickness = _DIN472[float(bore_d)]
    except KeyError as exc:
        raise ValueError(f"unsupported DIN 472 bore {bore_d:g}") from exc
    ring = Cylinder(outer_d / 2, thickness, align=MIN) - Cylinder(
        inner_d / 2, thickness + 2, align=CTR,
    )
    ring -= Pos(outer_d / 3, 0, thickness / 2) * Box(
        outer_d, 1.2, thickness + 2, align=CTR,
    )
    ring.label = label or f"din472_{bore_d:g}mm"
    return ring


def wing_nut_m4(label: str = "din315_m4_wingnut") -> Part:
    """DIN 315 M4 wing-nut nominal 20 x 4 x 10.5 envelope."""
    span, wing_t, height = 20.0, 4.0, 10.5
    hub = Cylinder(4.0, 5.0,
                   align=(Align.CENTER, Align.CENTER, Align.MIN))
    wings = Pos(0, 0, height / 2) * Box(span, wing_t, height, align=CTR)
    # Taper the visual envelope with two smaller upper boxes while retaining
    # the exact maximum span and height; internal detail is intentionally low.
    bore = Pos(0, 0, -1) * Cylinder(
        2.0, height + 2,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    part = hub + wings - bore
    part.label = label
    return part


def threaded_stud(
    size: str,
    length: float,
    *,
    label: str | None = None,
) -> Part:
    """Threaded-rod envelope, local z=0..length."""
    diameter = float(size.upper()[1:])
    part = Cylinder(diameter / 2, length,
                    align=(Align.CENTER, Align.CENTER, Align.MIN))
    part.label = label or f"{size.lower()}x{length:g}_stud"
    return part


def machine_foot_m5_17(label: str = "elesa_432001_machine_foot") -> Part:
    """Elesa 432001 DVB.6 OD20 x 17 foot with an M5x6 stud.

    Local mounting plane is z=0; rubber extends to z=-17 and the stud to
    z=+6 for engagement in the selected female/female standoff.
    """
    rubber = Cylinder(10.0, 17.0,
                      align=(Align.CENTER, Align.CENTER, Align.MAX))
    stud = Cylinder(2.5, 6.0,
                    align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = rubber + stud
    part.label = label
    return part


def foot_standoff_m5_ff_18(
    label: str = "wurth_970180581_foot_standoff",
) -> Part:
    """Würth 970180581 WA-SSTII M5 female/female, AF8 x 18 mm.

    Local z=0..18. Threads are represented by the full OD5 clearance bore;
    the separate foot stud and ISO 4026 set screw provide the two male
    occurrences and their engagement evidence.
    """
    body = _hex_prism(8.0, 18.0)
    bore = Pos(0, 0, -1.0) * Cylinder(
        2.5, 20.0, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )
    part = body - bore
    part.label = label
    return part


def felt_washer(
    od: float = 20.0,
    bore: float = 4.5,
    thickness: float = 3.0,
    *,
    label: str = "felt_washer",
) -> Part:
    """Dimensioned wool-felt drag washer envelope."""
    part = Cylinder(od / 2, thickness, align=MIN) - (
        Pos(0, 0, -1) * Cylinder(bore / 2, thickness + 2, align=MIN)
    )
    part.label = label
    return part


def spring_envelope(
    od: float,
    bore: float,
    length: float,
    *,
    label: str = "spring_envelope",
) -> Part:
    """Fast collision envelope for a selected compression spring."""
    part = Cylinder(od / 2, length, align=MIN) - (
        Pos(0, 0, -1) * Cylinder(bore / 2, length + 2, align=MIN)
    )
    part.label = label
    return part


# ---------------------------------------------------------------------------
# Placement helpers


_AXIS_ROTATION = {
    "+z": (0.0, 0.0, 0.0),
    "-z": (180.0, 0.0, 0.0),
    "+x": (0.0, 90.0, 0.0),
    "-x": (0.0, -90.0, 0.0),
    "+y": (-90.0, 0.0, 0.0),
    "-y": (90.0, 0.0, 0.0),
}


def place(
    part: Part,
    origin: Sequence[float] = (0.0, 0.0, 0.0),
    *,
    axis: str = "+z",
    spin_deg: float = 0.0,
    label: str | None = None,
) -> Part:
    """Place local z-axis hardware on one of the six machine cardinal axes.

    ``origin`` is the screw's head-bearing plane centre or the z=0 datum of a
    washer/nut/collar.  Positive axis is the direction local +Z points; screw
    shanks therefore extend opposite that axis.
    """
    try:
        rx, ry, rz = _AXIS_ROTATION[axis.lower()]
    except KeyError as exc:
        raise ValueError(f"axis must be one of {tuple(_AXIS_ROTATION)}") from exc
    result = Pos(*map(float, origin)) * (
        Rot(rx, ry, rz) * (Rot(0, 0, float(spin_deg)) * part)
    )
    result.label = label or getattr(part, "label", "hardware")
    return result


def place_many(
    part: Part,
    placements: Iterable[Mapping[str, Any]],
    *,
    label_prefix: str | None = None,
) -> list[Part]:
    """Return individually labelled occurrences from placement dictionaries."""
    result: list[Part] = []
    for index, spec in enumerate(placements):
        occurrence_label = spec.get("label")
        if occurrence_label is None and label_prefix is not None:
            occurrence_label = f"{label_prefix}_{index:02d}"
        result.append(place(
            _fresh(part, getattr(part, "label", None)),
            spec.get("origin", (0.0, 0.0, 0.0)),
            axis=spec.get("axis", "+z"),
            spin_deg=spec.get("spin_deg", 0.0),
            label=occurrence_label,
        ))
    return result


# ---------------------------------------------------------------------------
# Machine-readable assembly demand and procurement aggregation


def _item(
    item_id: str,
    group: str,
    description: str,
    qty: int,
    sku: str,
    standard: str,
    factory: str | None,
    kwargs: Mapping[str, Any] | None,
    usage: str,
    *,
    status: str = "selected",
    note: str = "",
    placement_authority: str = "hardware_placements",
) -> dict[str, Any]:
    return {
        "id": item_id,
        "group": group,
        "description": description,
        "qty": int(qty),
        "sku": sku,
        "standard": standard,
        "model": None if factory is None else {
            "factory": factory,
            "kwargs": dict(kwargs or {}),
        },
        "usage": usage,
        "status": status,
        "note": note,
        "placement_authority": placement_authority,
    }


HARDWARE_SCHEDULE: tuple[dict[str, Any], ...] = (
    _item("frame_brackets", "frame", "2020 right-angle bracket", 15,
          "MISUMI-HBKTST5", "MISUMI 5-series slot-6",
          "angle_bracket_2020", {}, "all frame extrusion joints"),
    _item("frame_bracket_screws", "frame",
          "M5x10 extra-low-head bracket screw", 30,
          "MISUMI-CBSA5-10", "MISUMI CBSA5-10",
          "extra_low_head_screw_cbsa5_10", {},
          "two per HBKTST5; 1.5 mm head clears orthogonal mate"),
    _item("frame_bracket_tnuts", "frame", "M5 slot-6 post-assembly T-nut", 30,
          "MISUMI-HNTA5-5", "MISUMI HNTA5-5", "tnut_slot6",
          {"size": "M5"}, "two per frame bracket"),
    _item("rear_post_shoe_screws", "frame",
          "M5x12 rear-post shoe screw", 2,
          "ISO4762-M5x12", "ISO 4762", "socket_head_cap_screw",
          {"size": "M5", "length": 12.0},
          "printed rear-post left shoe floor and upright"),
    _item("rear_post_shoe_tnuts", "frame",
          "M5 slot-6 rear-post shoe T-nut", 2,
          "MISUMI-HNTA5-5", "MISUMI HNTA5-5", "tnut_slot6",
          {"size": "M5"}, "printed rear-post left shoe stacks"),
    _item("machine_feet", "frame", "Elesa DVB.6 OD20x17 M5 rubber foot", 4,
          "ELESA-432001", "Elesa 432001 / DVB.6-20-17-M5-6-55",
          "machine_foot_m5_17", {},
          "bottom of 35 mm support stack; integral M5x6 stud"),
    _item("machine_foot_standoffs", "frame",
          "M5 female/female AF8x18 foot standoff", 4,
          "WURTH-970180581", "Würth 970180581 WA-SSTII",
          "foot_standoff_m5_ff_18", {},
          "18 mm rigid spacer between rail and 17 mm rubber foot"),
    _item("machine_foot_set_screws", "frame",
          "M5x12 flat-point bonded foot stud", 4,
          "ISO4026-M5x12", "ISO 4026 / DIN 913",
          "set_screw", {"size": "M5", "length": 12.0},
          "bond into standoff with 4.8 +/-0.1 mm projection"),
    _item("machine_foot_tnuts", "frame", "M5 slot-6 foot T-nut", 4,
          "MISUMI-HNTA5-5", "MISUMI HNTA5-5", "tnut_slot6",
          {"size": "M5"}, "captures four feet in base-rail underside slots"),

    _item("rail_screws", "m0", "M3x8 rail screw", 12,
          "ISO4762-M3x8", "ISO 4762", "socket_head_cap_screw",
          {"size": "M3", "length": 8.0}, "six per MGN12 rail"),
    _item("rail_tnuts", "m0", "M3 slot-6 post-assembly T-nut", 12,
          "MISUMI-HNTA5-3", "MISUMI HNTA5-3", "tnut_slot6",
          {"size": "M3"}, "MGN12 rails to stringers"),
    _item("block_screws", "m0", "M3x10 carriage-block screw", 8,
          "ISO4762-M3x10", "ISO 4762", "socket_head_cap_screw",
          {"size": "M3", "length": 10.0}, "four per MGN12H block"),
    _item("t8_nut_screws", "m0", "M3x12 T8 flange-nut screw", 4,
          "ISO4762-M3x12", "ISO 4762", "socket_head_cap_screw",
          {"size": "M3", "length": 12.0},
          "bracket-side screw into threaded T8 flange"),
    _item("t8_nut_washers", "m0", "M3 normal washer", 4,
          "ISO7089-M3", "ISO 7089 / DIN 125-A", "plain_washer",
          {"size": "M3"}, "T8 flange-nut stack"),
    _item("carriage_m4_screws", "m0_m1", "M4x20 front tower screw", 2,
          "ISO4762-M4x20", "ISO 4762", "socket_head_cap_screw",
          {"size": "M4", "length": 20.0},
          "front spindle-tower row to carriage plate"),
    _item("carriage_flag_m4_screws", "m0_m1",
          "M4x25 rear tower/flag screw", 2,
          "ISO4762-M4x25", "ISO 4762", "socket_head_cap_screw",
          {"size": "M4", "length": 25.0},
          "rear spindle-tower row through printable endstop flag"),
    _item("nut_bracket_m4_screws", "m0", "M4x25 nut-bracket screw", 2,
          "ISO4762-M4x25", "ISO 4762", "socket_head_cap_screw",
          {"size": "M4", "length": 25.0}, "T8 nut bracket to carriage plate"),
    _item("carriage_m4_washers", "m0_m1", "M4 normal washer", 6,
          "ISO7089-M4", "ISO 7089 / DIN 125-A", "plain_washer",
          {"size": "M4"}, "tower and nut-bracket stacks"),
    _item("carriage_m4_nylocs", "m0_m1", "M4 prevailing-torque nut", 6,
          "ISO10511-M4", "ISO 10511 / DIN 985", "nyloc_nut",
          {"size": "M4"}, "tower and nut-bracket stacks"),

    _item("motor_screws", "motion", "M3x10 NEMA17 mounting screw", 12,
          "ISO4762-M3x10", "ISO 4762", "socket_head_cap_screw",
          {"size": "M3", "length": 10.0}, "four each for M0/M1/M2"),
    _item("m0_mount_screws", "m0", "M5x12 M0-mount screw", 2,
          "ISO4762-M5x12", "ISO 4762", "socket_head_cap_screw",
          {"size": "M5", "length": 12.0}, "M0 motor mount to stringer"),
    _item("m0_mount_tnuts", "m0", "M5 slot-6 post-assembly T-nut", 2,
          "MISUMI-HNTA5-5", "MISUMI HNTA5-5", "tnut_slot6",
          {"size": "M5"}, "M0 motor mount to stringer"),
    _item("m0_support_screws", "m0",
          "M5x12 flush M0 bearing-support screw", 2,
          "ISO10642-M5x12", "ISO 10642", "countersunk_screw",
          {"size": "M5", "length": 12.0, "standard": "iso10642"},
          "fixed-end bearing support; flush in printed foot"),
    _item("m0_support_tnuts", "m0", "M5 short slot-6 post-assembly T-nut", 2,
          "MISUMI-HNTAJ5-5", "MISUMI HNTAJ5-5", "tnut_slot6_short_m5",
          {}, "paired fixed-end bearing support holes at 12 mm pitch"),
    _item("endstop_pedestal_screws", "m0", "M5x12 endstop-foot screw", 2,
          "ISO4762-M5x12", "ISO 4762", "socket_head_cap_screw",
          {"size": "M5", "length": 12.0},
          "endstop side-foot ears to cross rail"),
    _item("endstop_pedestal_tnuts", "m0", "M5 slot-6 post-assembly T-nut", 2,
          "MISUMI-HNTA5-5", "MISUMI HNTA5-5", "tnut_slot6",
          {"size": "M5"}, "endstop pedestal to cross rail"),
    _item("endstop_switch_screws", "m0", "M2x16 switch screw", 2,
          "ISO4762-M2x16", "ISO 4762", "socket_head_cap_screw",
          {"size": "M2", "length": 16.0},
          "Omron D2F-01L2-D3 through rear nut-pocket shoulder"),
    _item("endstop_switch_washers", "m0", "M2 normal washer", 2,
          "ISO7089-M2", "ISO 7089 / DIN 125-A", "plain_washer",
          {"size": "M2"}, "Omron D2F-01L2-D3 switch stack"),
    _item("endstop_switch_nylocs", "m0", "M2 prevailing-torque nut", 2,
          "ISO10511-M2", "ISO 10511 / DIN 985", "nyloc_nut",
          {"size": "M2"}, "Omron D2F-01L2-D3 switch stack"),

    _item("flyer_block_screws", "m2", "M5x16 flyer-block screw", 4,
          "ISO4762-M5x16", "ISO 4762", "socket_head_cap_screw",
          {"size": "M5", "length": 16.0}, "flyer bearing block to posts"),
    _item("flyer_block_tnuts", "m2", "M5 slot-6 post-assembly T-nut", 4,
          "MISUMI-HNTA5-5", "MISUMI HNTA5-5", "tnut_slot6",
          {"size": "M5"}, "flyer bearing block to posts"),
    _item("m2_mount_screws", "m2", "M5x12 M2-mount screw", 4,
          "ISO4762-M5x12", "ISO 4762", "socket_head_cap_screw",
          {"size": "M5", "length": 12.0}, "M2 motor mount to posts"),
    _item("m2_mount_tnuts", "m2", "M5 slot-6 post-assembly T-nut", 4,
          "MISUMI-HNTA5-5", "MISUMI HNTA5-5", "tnut_slot6",
          {"size": "M5"}, "M2 motor mount to posts"),
    _item("m3_base_screws", "m3", "M5x12 flush tensioner-base screw", 8,
          "ISO10642-M5x12", "ISO 10642", "countersunk_screw",
          {"size": "M5", "length": 12.0, "standard": "iso10642"},
          "spool/felt/dancer/entry bases, two each"),
    _item("m3_base_tnuts", "m3", "M5 slot-6 post-assembly T-nut", 6,
          "MISUMI-HNTA5-5", "MISUMI HNTA5-5", "tnut_slot6",
          {"size": "M5"}, "spool/felt/entry bases"),
    _item("dancer_base_tnuts", "m3", "M5 short dancer-base T-nut", 2,
          "MISUMI-HNTAJ5-5", "MISUMI HNTAJ5-5", "tnut_slot6_short_m5",
          {}, "dancer base at y21/y69; clears entry and pivot stacks"),

    _item("flyer_set_screws", "m2", "M3x8 flat-point arm set screw", 2,
          "ISO4026-M3x8", "ISO 4026", "set_screw",
          {"size": "M3", "length": 8.0}, "retained flyer-arm collar x2"),
    _item("flyer_set_inserts", "m2", "M3 standard heat-set insert", 2,
          "MCMASTER-94459A140", "McMaster 94459A140", "heat_set_insert",
          {"size": "M3", "length": "standard"},
          "flyer-arm collar x2"),
    _item("counterweight_screws", "m2", "M3x6 countersunk rear-balance screw", 4,
          "ISO10642-M3x6", "McMaster 92125A126 / ISO 10642",
          "countersunk_screw",
          {"size": "M3", "length": 6.0, "standard": "iso10642"},
          "one per rear ASTM-B777 slug and printed retainer stack",
          placement_authority="integrated_release_candidate"),
    _item("counterweight_inserts", "m2", "M3 short heat-set insert", 4,
          "MCMASTER-94459A130", "McMaster 94459A130", "heat_set_insert",
          {"size": "M3", "length": "short"},
          "one per rear printed counterweight-retainer boss",
          placement_authority="integrated_release_candidate"),
    _item("front_trim_screws", "m2", "M2x8 front-balance screw", 2,
          "ISO4762-M2x8", "ISO 4762", "socket_head_cap_screw",
          {"size": "M2", "length": 8.0},
          "one per front OD6/ID2.2 ASTM-B777 trim",
          placement_authority="integrated_release_candidate"),
    _item("front_trim_washers", "m2", "M2 front-balance washer", 2,
          "ISO7089-M2", "ISO 7089 / DIN 125-A", "plain_washer",
          {"size": "M2"}, "one under each front-balance screw",
          placement_authority="integrated_release_candidate"),
    _item("front_trim_inserts", "m2", "M2 standard heat-set insert", 2,
          "MCMASTER-94459A120", "McMaster 94459A120", "heat_set_insert",
          {"size": "M2", "length": "standard"},
          "blind printed-arm pilot for each front-balance screw",
          placement_authority="integrated_release_candidate"),
    _item("flyer_guide_screws", "m2", "M2x6 PEEK flyer-guide screw", 3,
          "ISO4762-M2x6", "ISO 4762", "socket_head_cap_screw",
          {"size": "M2", "length": 6.0},
          "three removable one-piece PEEK flyer-guide ears",
          placement_authority="integrated_release_candidate"),
    _item("flyer_guide_inserts", "m2", "M2 standard heat-set insert", 3,
          "MCMASTER-94459A120", "McMaster 94459A120", "heat_set_insert",
          {"size": "M2", "length": "standard"},
          "three blind printed-arm pilots for the PEEK flyer guide",
          placement_authority="integrated_release_candidate"),
    _item("cap_retention_screws", "m1", "M2x20 paired-cap through screw", 3,
          "ISO4762-M2x20", "ISO 4762", "socket_head_cap_screw",
          {"size": "M2", "length": 20.0},
          "three stacks clamp the front/rear short-leadin PEEK caps",
          placement_authority="integrated_release_candidate"),
    _item("cap_retention_washers", "m1", "M2 paired-cap washer", 6,
          "ISO7089-M2", "ISO 7089 / DIN 125-A", "plain_washer",
          {"size": "M2"}, "front and rear washer in each cap stack",
          placement_authority="integrated_release_candidate"),
    _item("cap_retention_nylocs", "m1", "M2 paired-cap prevailing-torque nut", 3,
          "ISO10511-M2", "ISO 10511 / DIN 985", "nyloc_nut",
          {"size": "M2"}, "rear retention of the three paired-cap stacks",
          placement_authority="integrated_release_candidate"),
    _item("active_sector_m3_screws", "m0_m1", "M3x14 active-sector guide screw", 4,
          "ISO4762-M3x14", "ISO 4762", "socket_head_cap_screw",
          {"size": "M3", "length": 14.0},
          "two clamps per M0-following PEEK active-sector guide",
          placement_authority="integrated_release_candidate"),
    _item("active_sector_m3_washers", "m0_m1", "M3 active-sector guide washer", 4,
          "ISO7089-M3", "ISO 7089 / DIN 125-A", "plain_washer",
          {"size": "M3"}, "one beneath each active-sector M3 screw",
          placement_authority="integrated_release_candidate"),
    _item("active_sector_m3_inserts", "m0_m1", "M3 short active-sector insert", 4,
          "MCMASTER-94459A130", "McMaster 94459A130", "heat_set_insert",
          {"size": "M3", "length": "short"},
          "one in each accessible PEEK guide pad",
          placement_authority="integrated_release_candidate"),
    _item("active_sector_m4_screws", "m0_m1", "M4x10 yoke-to-tower screw", 4,
          "ISO4762-M4x10", "ISO 4762", "socket_head_cap_screw",
          {"size": "M4", "length": 10.0},
          "four front-installed aluminum-yoke adapter stacks",
          placement_authority="integrated_release_candidate"),
    _item("active_sector_m4_washers", "m0_m1", "M4 yoke-to-tower washer", 4,
          "ISO7089-M4", "ISO 7089 / DIN 125-A", "plain_washer",
          {"size": "M4"}, "one beneath each yoke adapter screw",
          placement_authority="integrated_release_candidate"),
    _item("active_sector_m4_inserts", "m0_m1", "M4 short tower heat-set insert", 4,
          "MCMASTER-94459A150", "McMaster 94459A150", "heat_set_insert",
          {"size": "M4", "length": "short"},
          "four blind revised-spindle-tower pilots",
          placement_authority="integrated_release_candidate"),

    _item("dancer_pulley_shoulder", "m3", "OD3x16 shoulder screw", 1,
          "MCMASTER-90265A420", "ISO 7379 / McMaster 90265A420",
          "shoulder_screw_90265a420", {}, "623ZZ dancer-pulley axle"),
    _item("dancer_pulley_shims", "m3", "OD3 DIN 988 thrust washer", 7,
          "DIN988-3x6x0.5", "DIN 988", "thrust_washer",
          {"bore_d": 3.1, "od": 6.0, "thickness": 0.5},
          "one each side of pulley plus five behind arm to reach thread shoulder"),
    _item("dancer_pulley_nyloc", "m3", "M2.5 prevailing-torque nut", 1,
          "ISO10511-M2.5", "ISO 10511 / DIN 985", "nyloc_nut",
          {"size": "M2.5"}, "90265A420 thread retention"),
    _item("dancer_pivot_shoulder", "m3", "OD5x10 shoulder screw", 1,
          "MCMASTER-96654A127", "McMaster 96654A127",
          "dancer_pivot_shoulder_screw", {}, "dancer-arm pivot",
          note="OD5 shoulder has an M4 thread; this matches the current OD5.2 bore."),
    _item("dancer_pivot_shims", "m3", "OD5 DIN 988 thrust washer", 2,
          "DIN988-5x10x0.5", "DIN 988", "thrust_washer",
          {"bore_d": 5.1, "od": 10.0, "thickness": 0.5},
          "one each side of dancer arm"),
    _item("dancer_pivot_tnut", "m3", "M4 slot-6 pivot T-nut", 1,
          "MISUMI-HNTA5-4", "MISUMI HNTA5-4", "tnut_slot6",
          {"size": "M4"}, "pivot shoulder screw retained in rear-post slot"),
    _item("dancer_stop_screws", "m3", "M3x10 dancer-stop screw", 2,
          "ISO4762-M3x10", "ISO 4762", "socket_head_cap_screw",
          {"size": "M3", "length": 10.0},
          "hard-stop pins retained in embedded short M3 inserts"),
    _item("dancer_stop_sleeves", "m3", "OD5xID3.2x4 stop sleeve", 2,
          "SLEEVE-5-3.2-4", "dimensioned steel sleeve", "spring_envelope",
          {"od": 5.0, "bore": 3.2, "length": 4.0},
          "only the sleeve crosses the dancer-arm plane"),
    _item("dancer_stop_washers", "m3", "M3 stop-pin washer", 2,
          "ISO7089-M3", "ISO 7089", "plain_washer", {"size": "M3"},
          "front of each stop sleeve"),
    _item("dancer_stop_inserts", "m3", "M3 short heat-set stop insert", 2,
          "MCMASTER-94459A769", "McMaster 94459A769",
          "heat_set_insert_m3_3p4", {},
          "installed from dancer-base rear before the base is mounted"),
    _item("dancer_fixed_anchor_screw", "m3",
          "M2x12 flush fixed spring pin", 1,
          "ISO14581-M2x12", "ISO 14581", "countersunk_screw",
          {"size": "M2", "length": 12.0, "standard": "iso14581"},
          "spring overpass anchor"),
    _item("dancer_moving_anchor_screw", "m3",
          "M2x16 flush moving spring pin", 1,
          "ISO14581-M2x16", "ISO 14581", "countersunk_screw",
          {"size": "M2", "length": 16.0, "standard": "iso14581"},
          "dancer-arm spring anchor"),
    _item("dancer_fixed_anchor_sleeve", "m3", "OD4xID2.2x1.5 sleeve", 1,
          "SLEEVE-4-2.2-1.5", "dimensioned steel sleeve", "spring_envelope",
          {"od": 4.0, "bore": 2.2, "length": 1.5},
          "fixed spring-loop stand-off"),
    _item("dancer_moving_anchor_sleeve", "m3", "OD4xID2.2x4 sleeve", 1,
          "SLEEVE-4-2.2-4", "dimensioned steel sleeve", "spring_envelope",
          {"od": 4.0, "bore": 2.2, "length": 4.0},
          "moving spring-loop stand-off"),
    _item("dancer_anchor_washers", "m3", "M2 spring-pin washer", 4,
          "ISO7089-M2", "ISO 7089", "plain_washer", {"size": "M2"},
          "one on each side of both spring loops"),
    _item("dancer_anchor_nylocs", "m3", "M2 spring-pin nyloc", 2,
          "ISO10511-M2", "ISO 10511", "nyloc_nut", {"size": "M2"},
          "spring-pin retention"),

    _item("spool_axle", "m3", "M8x75 socket-head axle bolt", 1,
          "ISO4762-M8x75", "ISO 4762", "socket_head_cap_screw",
          {"size": "M8", "length": 75.0}, "wire-spool axle",
          note="A smooth-shoulder axle is preferable if the spool rides directly on it."),
    _item("spool_axle_washers", "m3", "M8 normal washer", 2,
          "ISO7089-M8", "ISO 7089 / DIN 125-A", "plain_washer",
          {"size": "M8"}, "one at each spool-bracket ear"),
    _item("spool_axle_nyloc", "m3", "M8 prevailing-torque nut", 1,
          "ISO10511-M8", "ISO 10511 / DIN 985", "nyloc_nut",
          {"size": "M8"}, "spool axle retention"),

    _item("felt_stud", "m3", "M4x55 fully-threaded stud", 1,
          "DIN976-M4x55", "DIN 976-1", "threaded_stud",
          {"size": "M4", "length": 55.0}, "felt compression stack"),
    _item("felt_jam_nut", "m3", "M4 jam/hex nut", 1,
          "ISO4032-M4", "ISO 4032", "hex_nut",
          {"size": "M4"}, "locks stud into tensioner base"),
    _item("felt_backing_washers", "m3",
          "OD20xID4.5x1 304 backing disc", 2,
          "CUSTOM-304-DISC-20-4.5-1", "dimensioned laser-cut disc",
          "felt_washer", {"od": 20.0, "bore": 4.5, "thickness": 1.0},
          "felt pad backing; OD20 preserves 3.88 mm wire support margin"),
    _item("felt_pads", "m3", "OD20xID4.5x3 felt washer", 2,
          "CUSTOM-FELT-20-4.5-3", "dimensioned consumable", "felt_washer",
          {}, "adjustable wire drag pads"),
    _item("felt_compression_spring", "m3",
          "McMaster 94125K614 compression spring", 1,
          "94125K614", "McMaster 94125K614", "spring_envelope",
          {"od": 9.25, "bore": 6.75, "length": 21.0632960704},
          "felt preload stack; nominal 5 N drag pose",
          note="22 mm free; 8.896 N/mm; order package of five, four are spares."),
    _item("felt_spring_thrust_washer", "m3",
          "McMaster 91116A130 M4 oversized flat thrust washer", 1,
          "91116A130", "McMaster 91116A130", "thrust_washer",
          {"bore_d": 4.3, "od": 12.0, "thickness": 1.1},
          "between compression spring and wingnut"),
    _item("felt_wingnut", "m3", "M4 wing nut", 1,
          "DIN315-M4", "DIN 315", "wing_nut_m4", {},
          "tool-free felt preload adjustment"),
    _item("dancer_extension_spring", "m3",
          "Lee Spring LEM050AB 01 extension spring", 1,
          "LEM050AB01", "Lee Spring metric instrument series",
          "spring_envelope",
          {"od": 3.5, "bore": 2.0, "length": 10.84243517554},
          "dancer return spring at nominal arm pose",
          note="Free 9.50 mm; 2.350 N/mm; 12.00 N max; 13.82 mm max length."),
)


# Added once per SKU during purchasing, never to assembly occurrence counts.
SPARES_BY_SKU: dict[str, int] = {
    "MISUMI-HBKTST5": 1,
    "MISUMI-CBSA5-10": 4,
    "MISUMI-HNTA5-5": 6,
    "MISUMI-HNTAJ5-5": 1,
    "MISUMI-HNTA5-3": 2,
    "MISUMI-HNTA5-4": 1,
    "ISO4762-M5x12": 4,
    "ISO10642-M5x12": 2,
    "ISO4762-M5x16": 1,
    "ISO4762-M4x20": 2,
    "ISO4762-M4x25": 2,
    "ISO4762-M3x8": 3,
    "ISO4762-M3x10": 4,
    "ISO4762-M3x12": 1,
    "ISO4762-M2x16": 2,
    "ISO14581-M2x16": 1,
    "ISO14581-M2x12": 1,
    "ISO4026-M3x8": 2,
    "MCMASTER-94459A140": 2,
    "MCMASTER-94459A130": 1,
    "MCMASTER-94459A769": 1,
    "ISO7089-M2": 2,
    "ISO7089-M3": 4,
    "ISO7089-M4": 2,
    "ISO7089-M8": 2,
    "ISO10511-M2": 2,
    "ISO10511-M2.5": 1,
    "ISO10511-M4": 2,
    "ISO10511-M8": 1,
    "94125K614": 4,
    "91116A130": 4,
    "LEM050AB01": 1,
}


_FACTORIES = {
    "socket_head_cap_screw": socket_head_cap_screw,
    "countersunk_screw": countersunk_screw,
    "set_screw": set_screw,
    "hex_nut": hex_nut,
    "plain_washer": plain_washer,
    "heat_set_insert": heat_set_insert,
    "heat_set_insert_m3_3p4": heat_set_insert_m3_3p4,
    "angle_bracket_2020": angle_bracket_2020,
    "extra_low_head_screw_cbsa5_10": extra_low_head_screw_cbsa5_10,
    "tnut_slot6": tnut_slot6,
    "tnut_slot6_short_m5": tnut_slot6_short_m5,
    "nyloc_nut": nyloc_nut,
    "shoulder_screw_90265a420": shoulder_screw_90265a420,
    "dancer_pivot_shoulder_screw": dancer_pivot_shoulder_screw,
    "thrust_washer": thrust_washer,
    "threaded_stud": threaded_stud,
    "machine_foot_m5_17": machine_foot_m5_17,
    "foot_standoff_m5_ff_18": foot_standoff_m5_ff_18,
    "felt_washer": felt_washer,
    "spring_envelope": spring_envelope,
    "wing_nut_m4": wing_nut_m4,
}


def hardware_schedule() -> list[dict[str, Any]]:
    """Return a mutable deep copy of exact assembly demand."""
    return deepcopy(list(HARDWARE_SCHEDULE))


def schedule_item(item_id: str) -> dict[str, Any]:
    """Look up one assembly-demand row by stable ID."""
    for item in HARDWARE_SCHEDULE:
        if item["id"] == item_id:
            return deepcopy(item)
    raise KeyError(item_id)


def make_scheduled_part(item_id: str, *, label: str | None = None) -> Part:
    """Instantiate the lightweight geometry declared by a schedule row."""
    item = schedule_item(item_id)
    model = item["model"]
    if model is None:
        raise ValueError(f"{item_id} has no selected geometry yet")
    factory = _FACTORIES[model["factory"]]
    kwargs = dict(model["kwargs"])
    if label is not None:
        kwargs["label"] = label
    return factory(**kwargs)


def procurement_schedule() -> list[dict[str, Any]]:
    """Aggregate identical SKUs and add one set of purchasing spares."""
    grouped: dict[str, dict[str, Any]] = {}
    usage: dict[str, list[str]] = defaultdict(list)
    for item in HARDWARE_SCHEDULE:
        sku = item["sku"]
        if sku not in grouped:
            grouped[sku] = {
                "sku": sku,
                "description": item["description"],
                "standard": item["standard"],
                "required_qty": 0,
                "spare_qty": SPARES_BY_SKU.get(sku, 0),
                "status": item["status"],
            }
        grouped[sku]["required_qty"] += item["qty"]
        if item["status"] != "selected":
            grouped[sku]["status"] = item["status"]
        usage[sku].append(item["id"])
    result = []
    for sku, row in grouped.items():
        row["order_qty"] = row["required_qty"] + row["spare_qty"]
        row["schedule_ids"] = usage[sku]
        result.append(row)
    return sorted(result, key=lambda row: row["sku"])


def schedule_json(*, procurement: bool = False, indent: int = 2) -> str:
    """Serialize assembly demand or the aggregated purchasing list."""
    payload = procurement_schedule() if procurement else hardware_schedule()
    return json.dumps(payload, indent=indent, sort_keys=True)


def write_schedule(path: str | Path, *, procurement: bool = False) -> Path:
    """Write a reproducible JSON schedule when an integration report needs it."""
    target = Path(path)
    target.write_text(schedule_json(procurement=procurement) + "\n",
                      encoding="utf-8")
    return target


def catalog_sample() -> Compound:
    """Small labeled sample compound for optional STEP/viewer inspection."""
    samples = [
        angle_bracket_2020(),
        Pos(35, 0, 0) * tnut_slot6("M5"),
        Pos(50, 0, 12) * socket_head_cap_screw("M5", 12),
        Pos(65, 0, 0) * plain_washer("M5"),
        Pos(80, 0, 0) * nyloc_nut("M5"),
        Pos(95, 0, 20) * shoulder_screw_90265a420(),
        Pos(110, 0, 15) * dancer_pivot_shoulder_screw(),
        Pos(125, 0, 0) * shaft_collar(8),
        Pos(140, 0, 0) * retaining_ring_external(8),
        Pos(155, 0, 0) * heat_set_insert("M3"),
    ]
    result = Compound(children=samples)
    result.label = "hardware_catalog_sample"
    return result


def gen_step() -> Compound:
    """CAD-skill entry point; returns the optional representative catalog."""
    return catalog_sample()


if __name__ == "__main__":
    print(schedule_json(procurement=True))
