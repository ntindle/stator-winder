"""Regression tests for the isolated flyer horn/nozzle trade study."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import flyer_nozzle_trade as trade


def _report() -> dict:
    return json.loads(trade.JSON_OUT.read_text())


def test_any_passive_feature_has_more_than_half_mm_launch() -> None:
    kinematics = trade.target_kinematics()
    bound = trade.passive_orbit_lower_bound(kinematics)
    assert bound["status"] == "FAIL"
    assert bound["minimum_possible_worst_launch_mm"] > 6.2
    assert bound["minimum_possible_worst_launch_mm"] > 0.5
    assert len(trade._cases()) == 360 * 9 * 2


def test_closed_nozzle_and_open_horn_aperture_budget() -> None:
    budget = trade.mouth_budget()
    rows = budget["closed_nozzle_sweep"]
    assert [round(row["od_mm"], 6) for row in rows] == [
        0.7, 0.8, 0.9, 1.0, 1.1, 1.2
    ]
    assert rows[0]["fits_current_unrelieved_cap"]
    assert not rows[-1]["fits_current_unrelieved_cap"]
    assert all(row["fits_fully_relived_liner_only_mouth"] for row in rows)
    assert math.isclose(
        budget["open_horn_minimum_physical_od_mm"], 5.5,
        rel_tol=0.0, abs_tol=1e-12,
    )
    assert not budget["open_horn_can_enter_lined_mouth"]


def test_written_dense_report_is_bound_complete_and_fail_closed() -> None:
    report = _report()
    assert report["schema"] == trade.SCHEMA
    assert report["status"] == "DESIGN_NO_GO"
    assert report["release_authorized"] is False
    assert report["assembly_integration_authorized"] is False
    assert report["scope"]["required_route_cases"] == 6480
    assert report["closed_nozzle"]["candidate_count"] == 39
    assert report["closed_nozzle"]["passing_candidate_count"] == 0
    assert report["open_horn"]["candidate_count"] == 13
    assert report["open_horn"]["passing_candidate_count"] == 0
    assert all(
        row["case_count"] == 6480 and row["status"] == "FAIL"
        for row in report["closed_nozzle"]
        ["progressive_copper_at_z2_by_exit_radius"].values()
    )

    payload = dict(report)
    expected = payload.pop("report_sha256")
    canonical = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False
    )
    assert hashlib.sha256(canonical.encode()).hexdigest() == expected
    assert report["source_hashes"]["sim/flyer_nozzle_trade.py"] == (
        hashlib.sha256(Path(trade.__file__).read_bytes()).hexdigest()
    )

