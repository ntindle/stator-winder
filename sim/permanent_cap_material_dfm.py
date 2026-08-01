"""Standalone material and DFM decision for the permanent R3 stator cap.

This report deliberately does not import any generated geometry or simulation
report.  It restates the frozen ``cap-r3-sector-lane-v1`` manufacturing
contract, records the dated supplier evidence used to screen processes, and
fails production closed until production-intent parts pass physical tests.

The selected *material family* is natural, unfilled PEEK.  CNC machining is
the low-volume route and Victrex 450G injection molding is the volume route.
Neither is a released part or BOM line: the physical cap CAD, retention,
supplier DFM, material certificate, and coupons are still missing.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORTS = ROOT / "out" / "reports"
JSON_OUT = REPORTS / "permanent_cap_material_dfm.json"
MD_OUT = REPORTS / "permanent_cap_material_dfm.md"

SCHEMA = "permanent-cap-material-dfm/v1"
AS_OF = "2026-07-11"

# Frozen, source-independent manufacturing contract.
LANE_ID = "cap-r3-sector-lane-v1"
CAP_NOMINAL_WALL_MM = 1.0
MIN_CONTACT_RADIUS_MM = 2.88824
MIN_CLEAR_GROOVE_MM = 0.47752
NOMINAL_WIRE_DIAMETER_MM = 0.22352
MAXIMUM_ACCESS_WIRE_DIAMETER_MM = 0.5
WIRE_TEMPERATURE_RATING_C = 155.0
MAXIMUM_MACHINE_TENSION_N = 10.0


def _sources() -> list[dict[str, Any]]:
    """Return dated primary-manufacturer and current-supplier evidence."""

    return [
        {
            "id": "bambu-a1-spec",
            "kind": "manufacturer_product_page",
            "organization": "Bambu Lab",
            "url": "https://us.store.bambulab.com/products/a1",
            "accessed": AS_OF,
            "supports": [
                "A1 lists PETG as an ideal filament",
                "A1 lists 0.2 mm as an available nozzle diameter",
                "A1 does not recommend PA and other conventional high-temperature polymers because its frame is open",
            ],
        },
        {
            "id": "bambu-a1-hotend",
            "kind": "manufacturer_product_page",
            "organization": "Bambu Lab",
            "url": "https://us.store.bambulab.com/products/bambu-hotend-a1-series",
            "accessed": AS_OF,
            "supports": [
                "A1-series 0.2 mm stainless hotend is orderable from the product family page",
                "PETG is compatible with the 0.2 mm nozzle",
                "particle-filled materials are unsuitable for the 0.2 mm nozzle",
            ],
        },
        {
            "id": "inland-petg-plus-white",
            "kind": "current_vendor_catalog",
            "organization": "Micro Center",
            "url": "https://www.microcenter.com/product/626531/inland-175mm-petg-3d-printer-filament-1kg-%2822-lbs%29-spool-white",
            "accessed": AS_OF,
            "supports": [
                "INLAND PETG+ white, 1.75 mm, 1 kg is orderable",
                "vendor SKU 151563 and manufacturer part PETG+175SW1",
            ],
        },
        {
            "id": "victrex-450g-tds",
            "kind": "manufacturer_tds",
            "organization": "Victrex",
            "url": "https://www.victrex.com/-/media/downloads/datasheets/victrex_tds_450g.pdf?rev=-1",
            "accessed": AS_OF,
            "supports": [
                "450G is natural, unreinforced PEEK for injection molding and extrusion",
                "dielectric strength is 23 kV/mm at 2 mm and electrical RTI is 260 C",
                "mechanical RTI with impact is 180 C and without impact is 240 C",
                "molding shrinkage is 0.90 percent with flow and 1.3 percent across flow",
                "typical injection melt/nozzle and mold temperatures are 375 C and 170-200 C",
                "the resin is dried at 120-150 C to less than 0.020 percent moisture",
            ],
        },
        {
            "id": "victrex-injection-guide",
            "kind": "manufacturer_processing_guide",
            "organization": "Victrex",
            "url": "https://images.victrex.com/-/media/downloads/technical-guides/victrex_injection-molding-brochure_jan2022.pdf",
            "accessed": AS_OF,
            "supports": [
                "PEEK molding dimensions depend on geometry, tool, and process",
                "typical tolerances near 0.05 percent require controlled molding conditions",
                "tooling, gate, ejection, and cooling require part-specific DFM",
            ],
        },
        {
            "id": "victrex-finishing-guide",
            "kind": "manufacturer_processing_guide",
            "organization": "Victrex",
            "url": "https://www.victrex.com/-/media/downloads/technical-guides/victrex_finishing-brochure_jan2022.pdf",
            "accessed": AS_OF,
            "supports": [
                "stress-relief annealing should precede final machining",
                "a representative PEEK cycle reaches 200 C, holds one hour per millimeter, and cools at 10 C/hour below 140 C",
            ],
        },
        {
            "id": "ensinger-precision-machining",
            "kind": "manufacturer_service_brochure",
            "organization": "Ensinger",
            "url": "https://www.ensingerplastics.com/-/media/ensinger/files/document-teaser-files/brochures/others/machining-ensinger.pdf",
            "accessed": AS_OF,
            "supports": [
                "published precision-plastic examples include dimensional tolerances of plus/minus 0.02 mm",
                "5-axis milling, thermal treatment, and coordinate inspection are used on distortion-sensitive PEEK parts",
            ],
        },
        {
            "id": "mcmaster-peek-sheet",
            "kind": "current_vendor_catalog",
            "organization": "McMaster-Carr",
            "url": "https://www.mcmaster.com/products/peek-sheets/",
            "accessed": AS_OF,
            "supports": [
                "8504K35 is beige extruded PEEK sheet, 1/2 x 6 x 6 inches",
                "catalog thickness tolerance is -0.010 to +0.025 inch",
                "catalog service range is -20 to 480 F and UL 94 V-0 is listed",
                "the catalog does not identify the base resin grade or explicitly certify absence of all fillers",
            ],
        },
        {
            "id": "xometry-peek-cnc",
            "kind": "current_vendor_service",
            "organization": "Xometry",
            "url": "https://www.xometry.com/capabilities/cnc-machining-service/peek/",
            "accessed": AS_OF,
            "supports": [
                "generic PEEK CNC service is currently quotable",
                "plus/minus 0.001 inch (0.025 mm) machining tolerance is described as achievable",
                "32 microinch Ra is auto-quotable and lower roughness requires manual review",
                "glass-filled PEEK is described as abrasive and less suitable for wear interfaces",
            ],
        },
        {
            "id": "protolabs-peek-molding",
            "kind": "current_vendor_service",
            "organization": "Protolabs",
            "url": "https://www.protolabs.com/en-gb/services/injection-moulding/peek/",
            "accessed": AS_OF,
            "supports": [
                "Victrex 450G is a currently listed PEEK molding material",
                "vendor lists gate greater than 1 mm or 0.5 times wall thickness and about 1 percent molding shrinkage",
            ],
        },
        {
            "id": "fictiv-spi-finish",
            "kind": "current_vendor_process_reference",
            "organization": "Fictiv",
            "url": "https://www.fictiv.com/articles/spi-guidelines-for-injection-mold-surface-finish",
            "accessed": AS_OF,
            "supports": [
                "SPI A-2 tool finish is listed at 0.025-0.05 micrometer Ra",
                "SPI B-1 is listed at 0.05-0.10 micrometer Ra",
                "SPI C-1 is listed at 0.35-0.40 micrometer Ra",
                "part material affects how faithfully a tool finish transfers",
            ],
        },
        {
            "id": "syensqo-ryton-design",
            "kind": "manufacturer_design_guide",
            "organization": "Syensqo (legacy Solvay Ryton)",
            "url": "https://www.solvay.com/sites/g/files/srpend616/files/2018-10/Ryton-PPS-Design-Guide_EN-v2.3_0.pdf",
            "accessed": AS_OF,
            "supports": [
                "PPS applications with 0.38-0.51 mm walls are documented",
                "uniform walls, radiused intersections, controlled flow, gate position, venting, and weld-line control are required",
                "the illustrated high-flow grades are not evidence that an unfilled wire-contact grade is qualified",
            ],
        },
        {
            "id": "celanese-fortron-unfilled",
            "kind": "manufacturer_property_guide",
            "organization": "Celanese",
            "url": "https://www.celanese.com/-/media/Engineered%20Materials/Files/Product%20Technical%20Guides/PPS-012-FortronShortTermProperties-TG-EN-0916.pdf",
            "accessed": AS_OF,
            "supports": [
                "unreinforced Fortron PPS grades are documented",
                "representative unreinforced grades list 17-18 kV/mm electric strength",
                "representative unreinforced grades list 110-120 C DTUL at 1.8 MPa and 90 C glass transition",
                "representative unreinforced grades list about 1.2/1.5 percent mold shrinkage",
            ],
        },
        {
            "id": "formlabs-nylon12-tds",
            "kind": "manufacturer_tds",
            "organization": "Formlabs",
            "url": "https://formlabs.com/tds/nylon-12-tds/",
            "accessed": AS_OF,
            "supports": [
                "SLS Nylon 12 lists 87 C HDT at 1.8 MPa and 60 C glass transition",
                "SLS Nylon 12 lists 8.82 kV/mm dielectric strength",
            ],
        },
        {
            "id": "formlabs-fuse-design",
            "kind": "manufacturer_design_guide",
            "organization": "Formlabs",
            "url": "https://formlabs.com/support/Design-specifications-for-3D-models-Fuse-1/",
            "accessed": AS_OF,
            "supports": [
                "Nylon 12 minimum unsupported vertical wall is 0.6 mm",
                "minimum engraved detail width is 0.30-0.35 mm depending on orientation",
            ],
        },
        {
            "id": "xometry-sls-standards",
            "kind": "current_vendor_service_standard",
            "organization": "Xometry",
            "url": "https://www.xometry.com/manufacturing-standards/",
            "accessed": AS_OF,
            "supports": [
                "SLS Nylon 12 typical tolerance is plus/minus 0.015 inch or 0.002 inch per inch",
                "as-printed SLS surface roughness is about 315-520 microinch Ra",
                "guaranteed tolerances require manual review and a successful test build",
            ],
        },
        {
            "id": "remington-32sns",
            "kind": "manufacturer_product_page",
            "organization": "Remington Industries",
            "url": "https://www.remingtonindustries.com/magnet-wire/magnet-wire-32-awg-enameled-copper-9-spool-sizes/",
            "accessed": AS_OF,
            "supports": [
                "32SNSP.125 is 32 AWG single-build magnet wire with 0.0088 inch finished diameter",
                "its modified-polyurethane/polyamide insulation is rated 155 C to NEMA MW 80-C",
            ],
        },
    ]


def _physical_coupon_plan() -> dict[str, Any]:
    return {
        "status": "REQUIRED_NOT_RUN",
        "common_rules": [
            "use production-intent geometry, process, finish, retention, stator stack, wire lot, and varnish lot",
            "record raw measurements and photographs; a supplier certificate does not replace part inspection",
            "perform high-voltage work only in a guarded laboratory with a current-limited tester and trained operator",
        ],
        "material_receipt": {
            "lots": 1,
            "samples": 3,
            "acceptance": [
                "certificate names natural unfilled PEEK and the exact resin/stock grade",
                "certificate and declaration exclude glass, carbon, ceramic, mineral, PTFE, graphite, and metallic filler",
                "lot identity remains traceable through every coupon and cap",
            ],
        },
        "dimensional_and_finish": {
            "parts": "5 front caps plus 5 rear caps from one production-intent lot",
            "inspection": [
                "optical CMM or traced profile of every wire-contact lane after finishing",
                "white-light profilometer or calibrated replica measurement on the highest-risk contact location of every cap",
                "0.500 +/- 0.002 mm smooth polymer gauge passed through every open access mouth with no forced seating in the narrower groove",
            ],
            "acceptance": {
                "nominal_wall_mm": [0.90, 1.10],
                "minimum_local_contact_radius_mm": MIN_CONTACT_RADIUS_MM,
                "minimum_clear_groove_width_mm": MIN_CLEAR_GROOVE_MM,
                "maximum_contact_surface_Ra_um": 0.4,
                "minimum_open_access_diameter_mm": MAXIMUM_ACCESS_WIRE_DIAMETER_MM,
                "flash_or_burr_on_wire_contact_mm": 0.0,
            },
        },
        "retention_and_fit": {
            "assemblies": 5,
            "fixtures": "actual lamination stacks and production-intent cap pair",
            "load_basis": "a 180 degree wire turn can react approximately twice the 10 N tension; 60 N is a three-times static proof screen",
            "procedure": [
                "apply 60 N for 60 seconds at the outermost support in each axial direction and each in-plane worst direction",
                "repeat after thermal/varnish conditioning",
                "install three conditioned pairs in actual rotor/end-bell hardware and verify the complete 34.654781 mm finished-wire axial envelope",
            ],
            "acceptance": [
                "no latch release, crack, whitening, or cap-to-stator slip",
                "residual displacement at each load point is at most 0.05 mm",
                "retention does not rely on friction-only interference or on the finished winding",
                "no retention feature enters the validated wire, rotor, housing, or air-gap envelope",
            ],
        },
        "thermal_and_varnish": {
            "assemblies": 5,
            "procedure": [
                "condition for 100 hours at 155 C without wire sliding",
                "perform 10 cycles from -20 C to 155 C with one-hour dwells and ramp no faster than 3 C/min",
                "apply and cure the selected production varnish/impregnant to its real schedule on a separate matched set",
                "repeat dimensions, finish, retention, and dielectric tests after conditioning",
            ],
            "acceptance": [
                "no crack, blister, delamination, tack, or transferred residue",
                "contact-profile change and cap-to-stator movement are each at most 0.05 mm",
                "mass change after varnish/cure is at most 1.0 percent after surface cleaning",
                "all other coupon gates still pass",
            ],
            "qualification_boundary": "this is an engineering screen, not a UL thermal-aging or insulation-system certification",
        },
        "enamel_abrasion": {
            "coupons": "9 production-intent R3/groove coupons: 3 each at 1 N, 5 N, and 10 N",
            "wire": "new Remington 32SNSP.125 from the production lot for each coupon",
            "geometry": "the minimum-radius, minimum-width, maximum-wrap, worst parting-line/gate location from the cap drawing",
            "procedure": [
                "run 10,000 full 25 mm reciprocating traversals at 100 mm/s and the assigned constant tension",
                "if the wire breaks at a claimed machine setting, record a failed machine/cap system result rather than substituting thicker wire",
                "inspect cap and wire every 1,000 traversals and at completion at 50x or greater",
                "repeat on three thermally and varnish-conditioned coupons at the maximum released tension",
            ],
            "acceptance": [
                "zero exposed copper, enamel cracking, snag, or wire break",
                "no embedded abrasive particle or transferred cap debris",
                "wire resistance change is at most 1.0 percent after temperature correction",
                "cap contact Ra remains at most 0.4 micrometer and profile loss is at most 0.02 mm",
            ],
        },
        "dielectric": {
            "assemblies": 5,
            "procedure": [
                "test the actual wire bundle to the lamination stack before conditioning",
                "repeat after thermal/varnish conditioning and after the abrasion sequence",
                "apply 500 VDC for one minute for insulation resistance",
                "apply max(1500 VAC, 2 times declared motor rated voltage plus 1000 VAC) for 60 seconds as an internal withstand screen",
            ],
            "acceptance": [
                "insulation resistance is at least 100 megohm",
                "withstand leakage is at most 5 mA",
                "no flashover, puncture, carbon track, or permanent resistance change",
            ],
            "required_input": "declared motor rated voltage and the applicable product safety standard before release",
        },
    }


def build_report() -> dict[str, Any]:
    sources = _sources()

    report: dict[str, Any] = {
        "schema": SCHEMA,
        "as_of": AS_OF,
        "status": "CONDITIONAL_PEEK_ROUTE_SELECTED_NOT_PRODUCTION",
        "decision": {
            "selected_material_family": "natural unfilled PEEK with lot certification",
            "selected_low_volume_process": "5-axis CNC from certified stock, stress-relieved before finish machining",
            "selected_volume_process": "injection molded Victrex PEEK 450G natural with polished contact inserts and T1 inspection",
            "selected_in_house_prototype": "INLAND PETG+ on Bambu A1 with 0.2 mm nozzle, dimensional/assembly fit only",
            "production_authorized": False,
            "purchasing_authorized": False,
            "bom_or_release_catalog_edited": False,
            "why_no_catalog_line": "there is no released cap CAD, supplier-accepted drawing, certified material lot, physical coupon result, or retained motor fit",
        },
        "contract": {
            "lane_id": LANE_ID,
            "cap_kind": "permanent two-ended stator-attached wire guide and insulator",
            "nominal_wall_mm": CAP_NOMINAL_WALL_MM,
            "minimum_contact_surface_radius_mm": MIN_CONTACT_RADIUS_MM,
            "minimum_clear_polished_groove_width_mm": MIN_CLEAR_GROOVE_MM,
            "maximum_wire_contact_surface_Ra_um": 0.4,
            "nominal_wire": {
                "manufacturer_part": "Remington 32SNSP.125",
                "finished_diameter_mm": NOMINAL_WIRE_DIAMETER_MM,
                "temperature_rating_C": WIRE_TEMPERATURE_RATING_C,
            },
            "maximum_open_access_wire_diameter_mm": MAXIMUM_ACCESS_WIRE_DIAMETER_MM,
            "finished_wire_total_axial_envelope_mm": 34.654781,
            "important_distinction": "0.47752 mm is the minimum clear polished groove, while 0.5 mm is a separate open-mouth access test; a 0.5 mm pin is not forced into the narrower groove",
        },
        "drawing_and_supplier_dfm": {
            "status": "REQUIRES_FINAL_CAP_CAD_AND_SUPPLIER_ACCEPTANCE",
            "critical_characteristics": [
                "the complete wire-center offset of the manufactured contact surface contains the named lane",
                "all local contact radii and clear widths satisfy unilateral minima after tool wear, shrinkage, polishing, and inspection uncertainty",
                "no parting line, gate, ejector witness, flash, burr, support scar, or latch edge lies on a wire-contact surface",
                "retention and anti-rotation features remain outside validated copper, rotor, housing, and air-gap envelopes",
            ],
            "low_volume_cnc_planning_values": {
                "wall_mm": [0.90, 1.10],
                "critical_profile_tolerance_mm": 0.025,
                "proposed_model_minimum_contact_radius_mm": 2.95,
                "worst_radius_after_negative_tolerance_mm": 2.925,
                "proposed_model_clear_groove_mm": 0.55,
                "worst_groove_after_negative_tolerance_mm": 0.525,
                "required_final_surface_Ra_um": 0.4,
                "note": "the proposed manufacturing reserve must be put into the physical cap CAD and the exact offset-lane/collision gate rerun before quoting",
            },
            "volume_molding_planning_values": {
                "wall_mm": [0.90, 1.10],
                "critical_part_profile_tolerance_mm": 0.05,
                "proposed_model_minimum_contact_radius_mm": 3.00,
                "worst_radius_after_negative_tolerance_mm": 2.95,
                "proposed_model_clear_groove_mm": 0.60,
                "worst_groove_after_negative_tolerance_mm": 0.55,
                "contact_tool_finish": "SPI A-2 or better on removable contact inserts; finished PEEK still must measure Ra <= 0.4 um",
                "gate_rule": "gate >1 mm or >=0.5 x wall per the listed Victrex/Protolabs guidance; final gate count and location require flow analysis",
                "note": "a full roughly 86 mm class cap with many thin radial guides needs mold-flow, weld-line, vent, ejection, draft, and tool-polish review; generic thin-wall data are not supplier DFM",
            },
            "retention_concept": {
                "preferred_architecture": "positive paired-cap latches or a positive OD capture plus tooth-profile anti-rotation, all outside the proven wire cells",
                "prohibited_as_sole_retention": [
                    "friction-only press fit",
                    "finished copper winding",
                    "uncertified adhesive or varnish",
                    "features projecting into the rotor air gap or the unverified end-bell cavity",
                ],
                "thermal_fit_note": "unfilled PEEK expands substantially more than steel, especially near its glass transition; the latch must tolerate hot differential growth instead of depending on cold interference",
                "release_test": "60 N multi-direction proof before and after 155 C/varnish conditioning",
            },
        },
        "routes": {
            "in_house_petg_fit_prototype": {
                "status": "SELECTED_FOR_FIT_ONLY",
                "printer": "Bambu Lab A1",
                "material": "INLAND PETG+ white, Micro Center SKU 151563, MPN PETG+175SW1",
                "tooling": "0.2 mm stainless A1-series hotend; exact US variant SKU must be selected on the current Bambu product page",
                "process_start": {
                    "layer_height_mm": 0.08,
                    "nominal_wall_mm": 1.0,
                    "walls": 5,
                    "infill_percent": 100,
                    "orientation": "split front/rear parts; wire grooves face upward and no support touches a contact surface",
                },
                "attainable_claim": "1.0 mm walls and gross fit are printable; no catalog tolerance or as-printed Ra proves the 0.47752 mm/Ra 0.4 contact contract",
                "planning_tolerance_mm": 0.10,
                "prototype_groove_model_mm": 0.70,
                "postprocess": [
                    "remove strings and burrs without a knife edge toward the wire",
                    "wet finish 1000 then 2000 grit followed by plastic polish, only along the wire travel direction",
                    "wash and inspect for retained abrasive; measure profile and Ra rather than assuming polish succeeded",
                    "use smooth polymer gauge/monofilament only until the abrasion coupon passes",
                ],
                "thermal_dielectric_limit": "PETG is not released for the 155 C winding system and the vendor listing supplies no insulation-system dielectric qualification",
                "wire_winding_authorized": False,
            },
            "natural_unfilled_peek_cnc": {
                "status": "SELECTED_LOW_VOLUME_CANDIDATE_REQUIRES_MANUAL_QUOTE",
                "material_spec": "natural unfilled PEEK; exact grade and lot certificate required",
                "stock_candidate": {
                    "supplier": "McMaster-Carr",
                    "supplier_sku": "8504K35",
                    "description": "1/2 x 6 x 6 inch beige extruded PEEK sheet",
                    "quantity_planning": "one sheet per cap until nesting and finished axial thickness are proven",
                    "selection_status": "RFQ/receipt candidate only; McMaster page does not prove Victrex 450G or explicitly certify zero filler",
                },
                "service_candidate": "Xometry PEEK CNC or an equivalent certified 5-axis PEEK specialist with manual DFM",
                "attainable_wall_tolerance_finish": [
                    "1.0 mm wall is above published 0.5 mm generic CNC minimums but needs sacrificial support/soft jaws or vacuum fixturing",
                    "plus/minus 0.025 mm critical-profile tolerance is a supplier-review target supported by Xometry/Ensinger capability examples",
                    "standard 32 microinch Ra equals 0.813 micrometer and fails this contract; Ra <=0.4 micrometer requires a manually accepted finish operation",
                ],
                "postprocess": [
                    "rough machine with 0.10-0.20 mm stock on the contact surface",
                    "stress-relief anneal before final machining using the resin/stock supplier's approved cycle",
                    "finish with sharp dedicated tools and no glass-bead or mineral-blast contact finish",
                    "polish only along wire travel, ultrasonically/aqueously clean, then optically inspect and measure Ra",
                ],
                "thermal_dielectric_basis": "Victrex 450G data exceed the 155 C wire rating by electrical and mechanical RTI, but the 152 C as-molded HDT at 1.8 MPa and the unverified cap stress still require the 155 C loaded coupon",
                "wire_winding_authorized": False,
            },
            "victrex_450g_injection_molding": {
                "status": "SELECTED_VOLUME_CANDIDATE_REQUIRES_MOLD_FLOW_AND_T1",
                "material": "Victrex PEEK 450G natural, unreinforced",
                "procurement": "Victrex/authorized molder request-for-quote; no retail pellet SKU verified",
                "service_candidate": "Protolabs PEEK molding or equivalent high-temperature PEEK molder after engineer review",
                "attainable_wall_tolerance_finish": [
                    "a 1.0 mm material flow test and thinner PPS examples show thin-wall feasibility, not this cap's fill",
                    "plus/minus 0.05 mm critical part-profile is a planning target, not an accepted quote",
                    "SPI A-2 tool finish is far smoother than 0.4 micrometer nominally, but every T1 PEEK contact surface still requires finished-part profilometry",
                ],
                "process_controls": [
                    "dry to less than 0.020 percent moisture",
                    "use the resin supplier's 355-375 C barrel/nozzle and 170-200 C mold window as the starting process",
                    "put gates, weld lines, vents, ejectors, and parting lines off all wire-contact surfaces",
                    "tune tool steel from measured T1 shrink; do not bury negative tolerance at the exact contract minimum",
                ],
                "wire_winding_authorized": False,
            },
            "unfilled_pps": {
                "status": "TECHNICALLY_PLAUSIBLE_BACKUP_NOT_SELECTED",
                "basis": "PPS thin walls and unreinforced dielectric grades exist",
                "why_not_selected": [
                    "the thin-wall Ryton examples do not establish an unfilled contact grade for this cap",
                    "representative unfilled Fortron DTUL of 110-120 C is below the 155 C wire-system screen",
                    "current exact grade availability, toughness, polished-contact wear, and lot-certified stock are not closed",
                ],
            },
            "unfilled_pa12_sls": {
                "status": "REJECTED_FOR_PRODUCTION__SERVICE_FIT_MODEL_ONLY",
                "basis": "1.0 mm wall is printable in Formlabs Nylon 12",
                "why_not_selected": [
                    "typical SLS tolerance of about plus/minus 0.3-0.38 mm is too coarse for a 0.47752 mm groove without secondary machining",
                    "315-520 microinch Ra is about 8.0-13.2 micrometers and is far rougher than the 0.4 micrometer contact limit",
                    "60 C glass transition and 87 C HDT at 1.8 MPa do not support the 155 C winding-system screen",
                ],
            },
            "fiber_or_mineral_filled_polymers": {
                "status": "PROHIBITED_ON_WIRE_CONTACT_SURFACES",
                "excluded": [
                    "PEEK GF/CF or ceramic-filled machinable PEEK",
                    "PPS GF/mineral compounds",
                    "PA12 GF/CF",
                    "glass-bead or mineral-blasted finished wire surfaces",
                ],
                "reason": "cut fibers/filler and retained blasting media can present an enamel-abrasive surface even when the bulk material is strong",
            },
        },
        "physical_coupon_plan": _physical_coupon_plan(),
        "release_gates": {
            "final_cap_cad_contains_exact_lane_offset_surface": False,
            "manufacturing_reserve_revalidated_in_aggregate_and_collision_gates": False,
            "positive_retention_cad_and_actual_motor_cavity_proven": False,
            "supplier_accepts_wall_tolerance_finish_and_no_contact_witnesses": False,
            "certified_unfilled_natural_peek_lot_received": False,
            "dimensional_finish_coupon_passed": False,
            "retention_fit_coupon_passed": False,
            "thermal_varnish_coupon_passed": False,
            "enamel_abrasion_coupon_passed": False,
            "dielectric_coupon_passed": False,
            "production_authorized": False,
        },
        "sources": sources,
    }

    # Hash only the substantive report; do not recurse through report_sha256.
    canonical = json.dumps(
        report, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode()
    report["report_sha256"] = hashlib.sha256(canonical).hexdigest()
    return report


def markdown(report: dict[str, Any]) -> str:
    contract = report["contract"]
    decision = report["decision"]
    cnc = report["routes"]["natural_unfilled_peek_cnc"]
    mold = report["routes"]["victrex_450g_injection_molding"]
    prototype = report["routes"]["in_house_petg_fit_prototype"]
    plan = report["physical_coupon_plan"]

    lines = [
        "# Permanent cap material and DFM decision",
        "",
        f"**{report['status']}** — no production, purchasing, or assembly authorization.",
        "",
        "## Decision",
        "",
        f"- Material family: **{decision['selected_material_family']}**.",
        f"- Low volume: {decision['selected_low_volume_process']}.",
        f"- Volume: {decision['selected_volume_process']}.",
        f"- In-house: {decision['selected_in_house_prototype']}.",
        "- No BOM/release-catalog line was added because the physical cap, supplier DFM, certified lot, retention, and physical coupons are not released.",
        "",
        "## Frozen cap contract",
        "",
        f"- Lane: `{contract['lane_id']}`.",
        f"- Nominal wall: {contract['nominal_wall_mm']:.2f} mm.",
        f"- Manufactured local contact radius: >= {contract['minimum_contact_surface_radius_mm']:.5f} mm.",
        f"- Clear polished groove: >= {contract['minimum_clear_polished_groove_width_mm']:.5f} mm.",
        f"- Finished contact surface: Ra <= {contract['maximum_wire_contact_surface_Ra_um']:.1f} um.",
        f"- Nominal wire: {contract['nominal_wire']['finished_diameter_mm']:.5f} mm; separate open access: {contract['maximum_open_access_wire_diameter_mm']:.1f} mm.",
        "",
        "The groove and access dimensions are different tests: a 0.500 mm gauge passes the open mouth but is not forced into a 0.47752 mm groove.",
        "",
        "## Process comparison",
        "",
        "| Route | Wall / tolerance / finish result | Use |",
        "|---|---|---|",
        f"| A1 + INLAND PETG+ | {prototype['attainable_claim']} | Fit/handling only; no magnet-wire winding |",
        f"| CNC natural unfilled PEEK | 1.0 mm wall; +/-0.025 mm critical-profile target; standard 0.813 um Ra is insufficient, so <=0.4 um needs manual supplier acceptance | Selected low-volume candidate, coupons required |",
        f"| Molded Victrex 450G | 1.0 mm wall is plausible; +/-0.05 mm planning target; SPI A-2 contact insert plus finished-part profilometry | Selected volume candidate, mold-flow and T1 required |",
        "| Unfilled PPS | Thin wall and dielectric grades exist, but representative unfilled DTUL is 110-120 C and the exact grade/contact finish is open | Backup only |",
        "| SLS PA12 | Wall is printable, but typical tolerance is roughly +/-0.3-0.38 mm, Ra about 8-13 um, Tg 60 C, HDT 87 C | Rejected for production; fit model only |",
        "",
        "All GF, CF, ceramic/mineral-filled grades and bead/mineral-blasted wire surfaces are excluded. No exposed abrasive filler is accepted at the enamel interface.",
        "",
        "## Orderable/RFQ inputs",
        "",
        f"- Fit prototype: {prototype['material']}.",
        f"- Machining-stock candidate: McMaster `8504K35`, {cnc['stock_candidate']['description']}. This is receipt/RFQ material only because the catalog does not name the base resin grade or certify zero filler.",
        f"- CNC service: {cnc['service_candidate']}; lower-than-standard contact Ra requires manual review.",
        f"- Molding: {mold['material']} through {mold['procurement']}.",
        "",
        "## Retention",
        "",
        "Use positive paired-cap latches or a positive OD capture plus tooth-profile anti-rotation, entirely outside the proven wire/rotor/housing envelopes. Friction-only interference, the winding itself, and unqualified varnish/adhesive are not sole retention. The retained assembly must withstand 60 N in the worst directions before and after thermal/varnish conditioning with <=0.05 mm residual motion.",
        "",
        "## Required physical coupons",
        "",
        f"Status: **{plan['status']}**.",
        "",
        "1. Receive a lot-certified natural unfilled PEEK lot and exclude every fiber/mineral/lubricating filler.",
        "2. Inspect 5 front + 5 rear caps: every contact lane meets R2.88824, 0.47752 mm clear width, Ra0.4 um, 1.0 +/-0.1 mm wall, and open 0.500 mm access.",
        "3. Proof five cap/stator assemblies at 60 N in every worst direction, before and after conditioning; residual motion <=0.05 mm.",
        "4. Condition 100 h at 155 C, then 10 cycles -20..155 C; repeat dimensions, finish, retention, and dielectric checks. Repeat with the real cured varnish.",
        "5. Run 9 abrasion coupons (three each at 1, 5, and 10 N): 10,000 25 mm traversals at 100 mm/s using fresh Remington 32SNSP.125. Accept zero exposed copper, crack, snag, or break; <=1% resistance change; <=0.02 mm profile loss.",
        "6. On five actual wire/cap/stator assemblies, require >=100 Mohm at 500 VDC and withstand `max(1500 VAC, 2*V_rated + 1000 VAC)` for 60 s with <=5 mA and no breakdown. A guarded HV lab is mandatory.",
        "7. Fit three conditioned pairs in the real rotor/end-bell hardware and prove the 34.654781 mm finished-wire axial envelope; the current stator-only model cannot close this gate.",
        "",
        "These are engineering qualification screens, not a UL/NEMA insulation-system certification. Supplier DFM and the applicable motor product standard remain mandatory.",
        "",
        "## Release gates",
        "",
    ]
    for gate, value in report["release_gates"].items():
        lines.append(f"- {'PASS' if value else 'OPEN'} — `{gate}`")
    lines.extend([
        "",
        "## Sources",
        "",
    ])
    for source in report["sources"]:
        lines.append(
            f"- [{source['organization']} — {source['id']}]({source['url']}) "
            f"(accessed {source['accessed']})"
        )
    lines.extend(["", f"Report SHA-256: `{report['report_sha256']}`", ""])
    return "\n".join(lines)


def write_reports() -> dict[str, Any]:
    report = build_report()
    REPORTS.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    MD_OUT.write_text(markdown(report), encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--check", action="store_true",
        help="fail if checked-in JSON/Markdown differ from regenerated content",
    )
    args = parser.parse_args()
    report = build_report()
    expected_json = json.dumps(report, indent=2) + "\n"
    expected_md = markdown(report)
    if args.check:
        if not JSON_OUT.exists() or JSON_OUT.read_text(encoding="utf-8") != expected_json:
            raise SystemExit(f"stale or missing {JSON_OUT}")
        if not MD_OUT.exists() or MD_OUT.read_text(encoding="utf-8") != expected_md:
            raise SystemExit(f"stale or missing {MD_OUT}")
        print({"status": report["status"], "check": "PASS"})
        return
    write_reports()
    print({
        "status": report["status"],
        "production_authorized": report["decision"]["production_authorized"],
        "json": str(JSON_OUT),
        "markdown": str(MD_OUT),
    })


if __name__ == "__main__":
    main()
