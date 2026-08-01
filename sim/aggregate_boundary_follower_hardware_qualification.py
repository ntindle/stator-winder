"""Fail-closed hardware/load qualification for the aggregate follower.

This is an isolated simulator-side contract.  It does not create follower CAD,
modify the assembly, select production hardware, or authorize winding.  The
report binds the smallest defensible prototype load path to current project
evidence while keeping every unresolved physical gate explicit.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORTS = ROOT / "out" / "reports"

M1_LOAD_PATH = REPORTS / "m1_selector_alternating_former.json"
YOKE_AUDIT_PATH = REPORTS / "carriage_active_sector_terminal_guide_audit.json"
SUCCESSOR_TRADE_PATH = REPORTS / "cap_live_tail_manufactured_support_trade.json"
DANCER_LOAD_PATH = REPORTS / "dancer_loads.json"
RELEASE_CATALOG_PATH = ROOT / "cad" / "release_catalog.json"
YOKE_SOURCE_PATH = ROOT / "cad" / "carriage_active_sector_terminal_guide.py"

OUTPUT_JSON = REPORTS / "aggregate_boundary_follower_hardware_qualification.json"
OUTPUT_MD = REPORTS / "aggregate_boundary_follower_hardware_qualification.md"

SCHEMA = "aggregate-boundary-follower-hardware-qualification/v1"

DESIGN_WIRE_TENSION_N = 10.0
UNBOUND_MAXIMUM_WRAP_DEG = 180.0
PROOF_FACTOR = 2.0

PRIMARY_M4_COUNT = 4
PRIMARY_M4_SCREW_LENGTH_MM = 10.0
PRIMARY_M4_WASHER_MM = 0.90
PRIMARY_TOWER_PLATE_MM = 4.0
PRIMARY_M4_INSERT_MM = 4.70
PRIMARY_M4_PILOT_MM = 6.0

SECONDARY_M3_PER_AXIAL_CASSETTE = 2
SECONDARY_M3_SCREW_LENGTH_MM = 14.0
SECONDARY_M3_WASHER_MM = 0.55
SECONDARY_M3_CURRENT_CLAMP_MM = 10.0
SECONDARY_M3_INSERT_MM = 4.30
SECONDARY_M3_PILOT_MM = 5.50

RADIAL_MOTION_RATIO = 0.29
RADIAL_USABLE_TRAVEL_MM = 6.0
RADIAL_HARD_TRAVEL_RANGE_MM = (6.4, 6.6)
CONTACT_FORCE_HARD_CAP_N = 2.0
TANGENTIAL_USABLE_HALF_TRAVEL_MM = 0.5
TANGENTIAL_HARD_STOP_HALF_TRAVEL_MM = 0.6


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value: Any, field: str | None = None) -> str:
    body = deepcopy(value)
    if field is not None and isinstance(body, dict):
        body.pop(field, None)
    return hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def _require_close(name: str, observed: float, expected: float) -> None:
    if not math.isclose(float(observed), float(expected), rel_tol=0.0,
                        abs_tol=1.0e-9):
        raise ValueError(
            f"{name} authority drift: observed {observed}, expected {expected}"
        )


def _purchase_line(catalog: Mapping[str, Any], key: str) -> dict[str, Any]:
    row = catalog["hardware_purchase_map"].get(key)
    if not isinstance(row, dict):
        raise ValueError(f"release catalog is missing {key}")
    if row.get("purchase_status") != "cart_ready":
        raise ValueError(f"release catalog line {key} is not cart_ready")
    selection = row.get("selection")
    if not isinstance(selection, dict) or not selection.get("supplier_sku"):
        raise ValueError(f"release catalog line {key} lacks an exact SKU")
    return {
        "catalog_key": key,
        "supplier": selection.get("supplier"),
        "supplier_sku": selection.get("supplier_sku"),
        "manufacturer": selection.get("manufacturer"),
        "mpn": selection.get("mpn"),
        "purchase_status": row.get("purchase_status"),
        "pack_qty": row.get("pack_qty"),
        "catalog_selection_only_not_motion_authority": True,
    }


def _spring_output_force(
    initial_load_n: float, rate_n_per_mm: float, ratio: float,
    follower_travel_mm: float,
) -> float:
    return ratio * (
        initial_load_n + rate_n_per_mm * ratio * follower_travel_mm
    )


def analyze() -> dict[str, Any]:
    m1 = _load(M1_LOAD_PATH)
    yoke = _load(YOKE_AUDIT_PATH)
    trade = _load(SUCCESSOR_TRADE_PATH)
    dancer = _load(DANCER_LOAD_PATH)
    catalog = _load(RELEASE_CATALOG_PATH)

    m1_loads = m1["loads_balance_sensors"]
    yoke_dfm = yoke["guide_structure_DFM_and_attachments"]
    yoke_attachment = yoke_dfm["attachments"]
    yoke_load = yoke_dfm["mass_and_yoke_load"]
    successor = trade["recommended_successor"]
    dofs = successor["smallest_additional_physical_DOF"]

    _require_close(
        "M1 design wire tension",
        m1_loads["design_wire_tension_N"], DESIGN_WIRE_TENSION_N,
    )
    inherited_90_resultant = 2.0 * DESIGN_WIRE_TENSION_N * math.sin(
        math.radians(90.0) / 2.0
    )
    _require_close(
        "M1 90 degree horn resultant",
        m1_loads["maximum_90deg_horn_resultant_N"],
        inherited_90_resultant,
    )
    inherited_90_proof = PROOF_FACTOR * inherited_90_resultant
    _require_close(
        "M1 90 degree hard-stop proof",
        m1_loads["hard_stop_proof_load_N"], inherited_90_proof,
    )
    _require_close(
        "successor radial usable stroke",
        dofs[0]["prototype_design_stroke_mm"], RADIAL_USABLE_TRAVEL_MM,
    )
    _require_close(
        "successor tangential usable stroke",
        dofs[1]["prototype_design_stroke_mm"],
        2.0 * TANGENTIAL_USABLE_HALF_TRAVEL_MM,
    )
    if yoke_attachment["M4_hardware"] != (
            "4x M4x10 + washer + short M4 insert"):
        raise ValueError("primary M4 tower-stack authority drift")
    if yoke_attachment["M3_hardware"] != (
            "4x M3x14 + washer + short M3 insert"):
        raise ValueError("secondary M3 cassette-stack authority drift")

    spring = dancer["recommended"]["spring"]
    if spring["sku"] != "LEM050AB 01":
        raise ValueError("LEM050AB 01 spring authority is missing")
    spring_free = float(spring["free_length_mm"])
    spring_max_length = float(spring["max_length_mm"])
    spring_initial = float(spring["initial_load_n"])
    spring_max_load = float(spring["max_load_n"])
    spring_rate = float(spring["rate_n_per_mm"])

    unbound_static_reaction = (
        2.0 * DESIGN_WIRE_TENSION_N
        * math.sin(math.radians(UNBOUND_MAXIMUM_WRAP_DEG) / 2.0)
    )
    unbound_proof_load = PROOF_FACTOR * unbound_static_reaction
    primary_per_fastener = unbound_proof_load / PRIMARY_M4_COUNT
    secondary_per_fastener = (
        unbound_proof_load / SECONDARY_M3_PER_AXIAL_CASSETTE
    )

    primary_penetration = (
        PRIMARY_M4_SCREW_LENGTH_MM
        - PRIMARY_M4_WASHER_MM
        - PRIMARY_TOWER_PLATE_MM
    )
    secondary_engagement = (
        SECONDARY_M3_SCREW_LENGTH_MM
        - SECONDARY_M3_WASHER_MM
        - SECONDARY_M3_CURRENT_CLAMP_MM
    )
    secondary_full_engagement_shortfall = (
        SECONDARY_M3_INSERT_MM - secondary_engagement
    )
    secondary_clamp_for_full_engagement_max = (
        SECONDARY_M3_SCREW_LENGTH_MM
        - SECONDARY_M3_WASHER_MM
        - SECONDARY_M3_INSERT_MM
    )
    secondary_clamp_before_pilot_bottom_min = (
        SECONDARY_M3_SCREW_LENGTH_MM
        - SECONDARY_M3_WASHER_MM
        - SECONDARY_M3_PILOT_MM
    )

    radial_initial_output = RADIAL_MOTION_RATIO * spring_initial
    radial_effective_rate = spring_rate * RADIAL_MOTION_RATIO ** 2
    radial_usable_extension = RADIAL_MOTION_RATIO * RADIAL_USABLE_TRAVEL_MM
    radial_usable_length = spring_free + radial_usable_extension
    radial_usable_spring_load = (
        spring_initial + spring_rate * radial_usable_extension
    )
    radial_usable_output = _spring_output_force(
        spring_initial, spring_rate, RADIAL_MOTION_RATIO,
        RADIAL_USABLE_TRAVEL_MM,
    )
    hard_travel_rows = []
    for travel in RADIAL_HARD_TRAVEL_RANGE_MM:
        extension = RADIAL_MOTION_RATIO * travel
        spring_load = spring_initial + spring_rate * extension
        hard_travel_rows.append({
            "follower_travel_mm": travel,
            "spring_extension_mm": extension,
            "spring_length_mm": spring_free + extension,
            "spring_load_N": spring_load,
            "follower_output_force_N": RADIAL_MOTION_RATIO * spring_load,
        })
    maximum_hard_row = hard_travel_rows[-1]
    direct_extension = RADIAL_USABLE_TRAVEL_MM
    direct_length = spring_free + direct_extension
    direct_load = spring_initial + spring_rate * direct_extension

    definition_gates = {
        "10N_wire_tension_bound_to_current_M1_authority": True,
        "unknown_wrap_uses_180deg_40N_proof": math.isclose(
            unbound_proof_load, 40.0, abs_tol=1.0e-12
        ),
        "primary_four_M4_equal_share_is_10N_each": math.isclose(
            primary_per_fastener, 10.0, abs_tol=1.0e-12
        ),
        "primary_M4_stack_fully_covers_insert_without_bottoming": (
            primary_penetration >= PRIMARY_M4_INSERT_MM
            and primary_penetration <= PRIMARY_M4_PILOT_MM
        ),
        "secondary_M3_is_explicitly_non_proof": True,
        "secondary_current_M3_stack_does_not_claim_full_engagement": (
            secondary_engagement < SECONDARY_M3_INSERT_MM
        ),
        "radial_0p29_spring_curve_stays_below_2N_through_hard_travel": (
            maximum_hard_row["follower_output_force_N"]
            <= CONTACT_FORCE_HARD_CAP_N
            and maximum_hard_row["spring_length_mm"] <= spring_max_length
            and maximum_hard_row["spring_load_N"] <= spring_max_load
        ),
        "radial_and_tangential_travel_bounds_defined": True,
    }

    release_gates = {
        "exact_follower_wrap_angle_bound_at_all_required_routes": False,
        "positive_volume_follower_CAD_and_rigid_sweep_complete": False,
        "follower_mass_inertia_and_M0_M2_load_update_complete": False,
        "nose_to_datum_moment_arm_and_key_bearing_proven": False,
        "primary_M4_insert_pull_and_40N_nose_proof_complete": False,
        "tangential_centering_spring_selected_and_cycled": False,
        "tangential_bushing_or_flexure_selected_and_friction_proven": False,
        "fail_safe_positive_dock_and_NC_interlock_physically_proven": False,
        "0p2_to_0p5mm_wire_contact_enamel_and_wear_coupons_complete": False,
        "300rpm_reversal_endurance_and_spring_life_complete": False,
    }
    passed = all(definition_gates.values()) and all(release_gates.values())

    input_paths = {
        "M1_selector_loads": M1_LOAD_PATH,
        "active_sector_yoke_audit": YOKE_AUDIT_PATH,
        "successor_trade": SUCCESSOR_TRADE_PATH,
        "LEM_spring_authority": DANCER_LOAD_PATH,
        "release_catalog": RELEASE_CATALOG_PATH,
    }

    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS" if passed else "FAIL",
        "qualification_status": (
            "RELEASE_QUALIFIED" if passed else "FAIL_CLOSED"
        ),
        "decision": (
            "FOLLOWER_HARDWARE_RELEASE_QUALIFIED"
            if passed else "HARDWARE_ENVELOPE_DEFINED__PHYSICAL_QUALIFICATION_OPEN"
        ),
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "selected_release_modified": False,
        "scope": {
            "artifact": "isolated hardware/load qualification only",
            "wire_diameter_range_mm": [0.2, 0.5],
            "proved": [
                "conservative unbound-wrap proof load",
                "primary equal-share tower-fastener load",
                "M4/M3 stack arithmetic",
                "LEM050AB 01 radial motion-ratio force/length envelope",
                "usable and hard-stop travel bounds",
            ],
            "not_proved": [
                "positive-volume geometry, placement, or collision clearance",
                "actual follower wrap and nose-to-datum moment",
                "received hardware strength, fit, friction, fatigue, or wear",
                "wire enamel safety or production endurance",
            ],
        },
        "load_contract": {
            "design_wire_tension_N": DESIGN_WIRE_TENSION_N,
            "reaction_formula": "R=2*T*sin(wrap_angle/2)",
            "proof_factor": PROOF_FACTOR,
            "exact_successor_wrap_known": False,
            "unbound_maximum_wrap_deg": UNBOUND_MAXIMUM_WRAP_DEG,
            "unbound_static_reaction_N": unbound_static_reaction,
            "required_proof_load_N": unbound_proof_load,
            "inherited_90deg_static_reaction_N": inherited_90_resultant,
            "proof_load_if_all_exact_routes_are_le_90deg_N": inherited_90_proof,
            "reduction_condition": (
                "Reduce from 40 N only after every exact required route proves "
                "follower wrap <= 90 degrees."
            ),
            "old_3g_finger_inertial_force_N": m1_loads[
                "finger_inertial_force_N"
            ],
            "old_finger_mass_assumption_g": m1_loads[
                "finger_mass_assumption_g"
            ],
            "current_M0_added_yoke_guide_hardware_mass_g": yoke[
                "coupled_live_line_loads"
            ]["M0"]["added_fixed_guide_yoke_hardware_mass_g"],
            "current_M0_conservative_axis_force_N": yoke[
                "coupled_live_line_loads"
            ]["M0"]["final_conservative_axial_force_N"],
            "current_M0_margin_multiple_not_inherited": yoke[
                "coupled_live_line_loads"
            ]["M0"]["available_to_required_multiple"],
        },
        "primary_tower_mount": {
            "classification": "PRIMARY_PROTOTYPE_LOAD_PATH_PENDING_PHYSICAL_PROOF",
            "datum": {
                "A_plane": "machine y=-114 mm tower front face",
                "positive_keys": {
                    "machine_x_mm": [-10.0, 10.0],
                    "machine_z_mm": 61.0,
                    "size_x_z_depth_mm": [3.0, 2.0, 1.5],
                    "clearance_mm_per_side": 0.05,
                },
                "fastener_axes": {
                    "machine_x_mm": [-21.0, 21.0],
                    "machine_z_mm": [60.0, 66.0],
                },
                "location_control": yoke_attachment["M4_location_control"],
            },
            "fastener_count": PRIMARY_M4_COUNT,
            "proof_load_N": unbound_proof_load,
            "equal_share_per_fastener_N": primary_per_fastener,
            "existing_analytical_per_screw_3x_screen_N": yoke_load[
                "M4_per_screw_3x_proof_load_N"
            ],
            "equal_share_below_existing_analytical_screen": (
                primary_per_fastener
                <= yoke_load["M4_per_screw_3x_proof_load_N"]
            ),
            "hardware": {
                "screw": _purchase_line(catalog, "ISO4762-M4x10"),
                "washer": _purchase_line(catalog, "ISO7089-M4"),
                "insert": _purchase_line(catalog, "MCMASTER-94459A150"),
            },
            "stack": {
                "screw_length_mm": PRIMARY_M4_SCREW_LENGTH_MM,
                "washer_thickness_mm": PRIMARY_M4_WASHER_MM,
                "tower_adapter_plate_mm": PRIMARY_TOWER_PLATE_MM,
                "screw_penetration_mm": primary_penetration,
                "insert_length_mm": PRIMARY_M4_INSERT_MM,
                "pilot_depth_mm": PRIMARY_M4_PILOT_MM,
                "full_insert_engagement_analytical": (
                    primary_penetration >= PRIMARY_M4_INSERT_MM
                ),
                "pilot_bottoming_analytical": (
                    primary_penetration > PRIMARY_M4_PILOT_MM
                ),
            },
            "physical_proof_complete": False,
        },
        "secondary_axial_cassette_mount": {
            "classification": "NON_PROOF_LOCATOR_AND_CLAMP_ONLY",
            "proof_load_path_authorized": False,
            "one_carrier_must_span_both_tangential_identities": True,
            "fasteners_per_axial_carrier": SECONDARY_M3_PER_AXIAL_CASSETTE,
            "proof_load_if_misapplied_N": unbound_proof_load,
            "equal_share_if_misapplied_per_fastener_N": secondary_per_fastener,
            "existing_analytical_per_screw_3x_screen_N": yoke_load[
                "M3_per_screw_3x_proof_load_N"
            ],
            "equal_share_exceeds_existing_analytical_screen": (
                secondary_per_fastener
                > yoke_load["M3_per_screw_3x_proof_load_N"]
            ),
            "hardware": {
                "screw": _purchase_line(catalog, "ISO4762-M3x14"),
                "washer": _purchase_line(catalog, "ISO7089-M3"),
                "insert": _purchase_line(catalog, "MCMASTER-94459A130"),
            },
            "current_stack": {
                "screw_length_mm": SECONDARY_M3_SCREW_LENGTH_MM,
                "washer_thickness_mm": SECONDARY_M3_WASHER_MM,
                "clamp_stack_mm": SECONDARY_M3_CURRENT_CLAMP_MM,
                "calculated_screw_insert_overlap_mm": secondary_engagement,
                "insert_length_mm": SECONDARY_M3_INSERT_MM,
                "full_engagement_shortfall_mm": (
                    secondary_full_engagement_shortfall
                ),
                "reported_engagement_claim_mm": yoke_attachment[
                    "M3_insert_engagement_mm"
                ],
                "reported_claim_is_not_reused_as_proof": True,
            },
            "reuse_window_for_M3x14": {
                "minimum_clamp_stack_to_avoid_5p5mm_pilot_bottom_mm": (
                    secondary_clamp_before_pilot_bottom_min
                ),
                "maximum_clamp_stack_for_full_4p3mm_engagement_mm": (
                    secondary_clamp_for_full_engagement_max
                ),
                "otherwise_required": (
                    "new exact longer screw selection and complete stack audit"
                ),
            },
        },
        "conditional_pin_candidates": {
            "radial_parallel_guide": {
                "hardware": _purchase_line(catalog, "MCMASTER-90265A420"),
                "catalog_envelope": (
                    "OD3 x 16 mm shoulder, M2.5x0.45 x 4 mm thread, "
                    "OD5 x 2 mm head"
                ),
                "minimum_count": 2,
                "minimum_guided_overlap_at_max_travel_mm": 5.0,
                "full_thread_engagement_required_mm": 4.0,
                "precision_linear_guide_authorized": False,
            },
            "gimbal_pivot": {
                "hardware": _purchase_line(catalog, "MCMASTER-96654A127"),
                "catalog_envelope": (
                    "OD5 x 10 mm shoulder, M4x0.7 x 5 mm thread, "
                    "OD9 x 4 mm head"
                ),
                "direct_tap_into_aluminum_authorized": False,
                "reason": "5 mm thread is below the 6 mm M4-in-aluminum rule",
            },
            "qualification_limit": (
                "Catalog envelopes do not prove shoulder tolerance, finish, "
                "bushing fit, friction, retention, or cycle life."
            ),
        },
        "thread_engagement_contract": {
            "direct_steel_minimum_diameters": 1.0,
            "direct_aluminum_minimum_diameters": 1.5,
            "direct_PEEK_structural_threads_authorized": False,
            "M2p5_aluminum_minimum_mm": 3.75,
            "M3_aluminum_minimum_mm": 4.5,
            "M4_aluminum_minimum_mm": 6.0,
            "heat_set_insert_rule": (
                "Engage the complete female thread length and pass the "
                "material/process-specific pull coupon."
            ),
        },
        "radial_spring_contract": {
            "spring": {
                "sku": spring["sku"],
                "free_length_mm": spring_free,
                "maximum_length_mm": spring_max_length,
                "initial_load_N": spring_initial,
                "maximum_load_N": spring_max_load,
                "rate_N_per_mm": spring_rate,
                "catalog_selection": _purchase_line(catalog, "LEM050AB01"),
            },
            "motion_ratio_spring_extension_per_follower_travel": (
                RADIAL_MOTION_RATIO
            ),
            "usable_follower_travel_mm": RADIAL_USABLE_TRAVEL_MM,
            "hard_travel_range_mm": list(RADIAL_HARD_TRAVEL_RANGE_MM),
            "contact_force_hard_cap_N": CONTACT_FORCE_HARD_CAP_N,
            "initial_follower_output_force_N": radial_initial_output,
            "effective_follower_rate_N_per_mm": radial_effective_rate,
            "at_usable_travel": {
                "spring_extension_mm": radial_usable_extension,
                "spring_length_mm": radial_usable_length,
                "spring_load_N": radial_usable_spring_load,
                "follower_output_force_N": radial_usable_output,
            },
            "at_hard_travel_bounds": hard_travel_rows,
            "direct_one_to_one_6mm_extension_rejected": {
                "spring_length_mm": direct_length,
                "spring_load_N": direct_load,
                "length_exceeds_catalog_max": direct_length > spring_max_length,
                "load_exceeds_catalog_max": direct_load > spring_max_load,
            },
            "topology_released": False,
            "topology_blocker": (
                "Contact preload, fail-safe return, and positive M0 docking "
                "must be separated or proven in one explicit mechanism."
            ),
        },
        "tangential_contract": {
            "usable_half_travel_mm": TANGENTIAL_USABLE_HALF_TRAVEL_MM,
            "usable_total_travel_mm": 2.0 * TANGENTIAL_USABLE_HALF_TRAVEL_MM,
            "hard_stop_half_travel_mm": TANGENTIAL_HARD_STOP_HALF_TRAVEL_MM,
            "hard_stop_total_travel_mm": (
                2.0 * TANGENTIAL_HARD_STOP_HALF_TRAVEL_MM
            ),
            "centering_spring_selected": False,
            "bushing_or_flexure_selected": False,
            "friction_and_hysteresis_acceptance_defined": False,
        },
        "fail_closed_unknowns": [
            "exact maximum follower wrap angle across required routes",
            "follower and selected moving mass/inertia",
            "nose-to-datum force moment arm and key bearing distribution",
            "tangential centering spring and fatigue life",
            "tangential bushing/flexure fit, friction, and hysteresis",
            "0.2/0.5 mm production-wire enamel/contact/wear coupons",
            "300 rpm reversal endurance, return-to-dock, and spring life",
        ],
        "qualification_required": [
            "Bind one positive-volume follower CAD to every required route and exact wrap angle.",
            "Apply 40 N at the nose in every worst-case direction and through each hard stop.",
            "Pull-test all four primary M4 tower inserts in the production tower material/process.",
            "Measure radial and tangential force-displacement, breakaway, hysteresis, and cross-axis binding hot and cold.",
            "Fault-inject lost spring, broken link, jammed slide, and intermediate M0; NC interlock must block M1 unless all retracted.",
            "Run 0.2 and 0.5 mm production wire from low tension through 10 N; inspect enamel, dielectric integrity, debris, R3/groove geometry, and Ra.",
            "Run 300 rpm reversal/endurance and full-stroke spring fatigue after duty-life and hot-temperature requirements are supplied.",
        ],
        "definition_gates": definition_gates,
        "release_gates": release_gates,
        "input_files": {
            name: str(path.relative_to(ROOT)).replace("\\", "/")
            for name, path in input_paths.items()
        },
        "input_sha256": {
            name: _sha256(path) for name, path in input_paths.items()
        },
        "source_hashes": {
            "sim/aggregate_boundary_follower_hardware_qualification.py": (
                _sha256(Path(__file__))
            ),
            "cad/carriage_active_sector_terminal_guide.py": (
                _sha256(YOKE_SOURCE_PATH)
            ),
        },
    }
    report["report_sha256"] = _canonical_hash(report, "report_sha256")
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported aggregate-follower hardware schema")
    if report.get("report_sha256") != _canonical_hash(
            report, "report_sha256"):
        raise ValueError("aggregate-follower hardware report hash mismatch")
    for relative, expected in report.get("source_hashes", {}).items():
        path = ROOT / str(relative).replace("/", "\\")
        if not path.is_file() or _sha256(path) != expected:
            raise ValueError(f"stale aggregate-follower source {relative}")
    for name, relative in report.get("input_files", {}).items():
        path = ROOT / str(relative).replace("/", "\\")
        if not path.is_file() or _sha256(path) != report["input_sha256"].get(name):
            raise ValueError(f"stale aggregate-follower input {name}")
    expected_status = (
        "PASS"
        if all(report.get("definition_gates", {}).values())
        and all(report.get("release_gates", {}).values())
        else "FAIL"
    )
    if report.get("status") != expected_status:
        raise ValueError("aggregate-follower status/gate mismatch")
    if report.get("production_authorized") is not False:
        raise ValueError("qualification artifact cannot authorize production")
    if report.get("assembly_integration_authorized") is not False:
        raise ValueError("qualification artifact cannot authorize integration")
    _require_close(
        "integrity proof load",
        report["load_contract"]["required_proof_load_N"], 40.0,
    )
    _require_close(
        "integrity primary per-fastener load",
        report["primary_tower_mount"]["equal_share_per_fastener_N"], 10.0,
    )
    if report["secondary_axial_cassette_mount"][
            "proof_load_path_authorized"] is not False:
        raise ValueError("secondary M3 cassette must remain non-proof")


def render_markdown(report: Mapping[str, Any]) -> str:
    load = report["load_contract"]
    primary = report["primary_tower_mount"]
    secondary = report["secondary_axial_cassette_mount"]
    spring = report["radial_spring_contract"]
    usable = spring["at_usable_travel"]
    maximum_hard = spring["at_hard_travel_bounds"][-1]
    lines = [
        "# Aggregate-boundary follower hardware/load qualification", "",
        f"**{report['status']} — {report['decision']}**", "",
        "This is an isolated, fail-closed qualification contract. It does not "
        "modify CAD, the assembly, the BOM, procurement, or the selected release.",
        "", "## Load contract", "",
        f"- Wire tension: {load['design_wire_tension_N']:.1f} N",
        f"- Unknown-wrap reaction: {load['unbound_static_reaction_N']:.1f} N at 180 deg",
        f"- Required proof: {load['required_proof_load_N']:.1f} N",
        f"- Conditional <=90 deg proof: {load['proof_load_if_all_exact_routes_are_le_90deg_N']:.6f} N",
        "- Reduction is forbidden until every exact route proves wrap <=90 deg.",
        "", "## Primary tower load path", "",
        f"- Four M4x10 tower stacks: {primary['equal_share_per_fastener_N']:.1f} N proof share each",
        f"- Analytical penetration: {primary['stack']['screw_penetration_mm']:.2f} mm into a {primary['stack']['insert_length_mm']:.2f} mm insert",
        "- Positive tower keys locate; screw clearance does not locate.",
        "- Physical insert-pull and 40 N nose proof remain open.",
        "", "## Secondary cassette interface", "",
        f"- Classification: `{secondary['classification']}`",
        f"- Current M3x14 overlap: {secondary['current_stack']['calculated_screw_insert_overlap_mm']:.2f} mm versus {secondary['current_stack']['insert_length_mm']:.2f} mm insert",
        f"- Shortfall: {secondary['current_stack']['full_engagement_shortfall_mm']:.2f} mm",
        f"- Misapplied 40 N equal share: {secondary['equal_share_if_misapplied_per_fastener_N']:.1f} N per M3 versus {secondary['existing_analytical_per_screw_3x_screen_N']:.1f} N screen",
        "- It is a locator/clamp interface, not a proof load path.",
        "", "## Radial spring and travel", "",
        f"- LEM050AB 01 motion ratio: {spring['motion_ratio_spring_extension_per_follower_travel']:.2f}",
        f"- Initial follower force: {spring['initial_follower_output_force_N']:.6f} N",
        f"- At 6.0 mm: {usable['follower_output_force_N']:.6f} N; spring length {usable['spring_length_mm']:.3f} mm",
        f"- At 6.6 mm hard travel: {maximum_hard['follower_output_force_N']:.6f} N; spring length {maximum_hard['spring_length_mm']:.3f} mm",
        f"- Hard contact cap: {spring['contact_force_hard_cap_N']:.1f} N",
        "- Direct one-to-one 6 mm spring extension is rejected.",
        "", "## Tangential travel", "",
        f"- Usable: +/-{report['tangential_contract']['usable_half_travel_mm']:.1f} mm",
        f"- Hard stops: +/-{report['tangential_contract']['hard_stop_half_travel_mm']:.1f} mm",
        "- Centering spring, bushing/flexure, and friction acceptance remain open.",
        "", "## Release gates", "",
    ]
    lines.extend(
        f"- {'PASS' if value else 'OPEN'} — `{name}`"
        for name, value in report["release_gates"].items()
    )
    lines.extend(["", "## Required qualification", ""])
    lines.extend(f"- {item}" for item in report["qualification_required"])
    lines.extend([
        "", "Production and assembly integration remain unauthorized.", "",
    ])
    return "\n".join(lines)


def write_outputs(report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    result = dict(report or analyze())
    validate_report_integrity(result)
    OUTPUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    OUTPUT_MD.write_text(render_markdown(result), encoding="utf-8")
    return result


def main() -> int:
    report = write_outputs()
    print(
        f"aggregate follower hardware {report['status']}: "
        f"proof={report['load_contract']['required_proof_load_N']:.1f} N; "
        f"M4={report['primary_tower_mount']['equal_share_per_fastener_N']:.1f} N each; "
        f"open={sum(not value for value in report['release_gates'].values())}"
    )
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
