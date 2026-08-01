"""Durable, standalone evidence for the pinned upstream shaft-wrap regression.

This generator is deliberately outside the release/candidate/player pipeline.  It
binds two current captures, the exact upstream source and Git history, and a
review-only patch without editing the upstream ``winder`` checkout.  A report is
always fail-closed while the pinned upstream source still commands both wraps
from ``m1_zero`` instead of the live M1 position.
"""

from __future__ import annotations

import argparse
import difflib
import hashlib
import json
import math
import subprocess
from pathlib import Path
from typing import Any, Iterable

from traj import Timeline, load_events


HERE = Path(__file__).resolve().parent
MACHINE = HERE.parent
ROOT = MACHINE.parent
DEFAULT_WINDER = ROOT / "winder"
DEFAULT_RAW_CAPTURE = (
    MACHINE / "out" / "capture" / "independent_upstream_wrap_6039.jsonl"
)
DEFAULT_SERIAL_CAPTURE = (
    MACHINE / "out" / "capture" / "upstream_serial_twin_raw.jsonl"
)
DEFAULT_SERIAL_HARNESS = HERE / "capture_upstream_serial_twin.py"
DEFAULT_JSON = MACHINE / "out" / "reports" / "shaft_wrap_regression_evidence.json"
DEFAULT_MD = MACHINE / "out" / "reports" / "shaft_wrap_regression_evidence.md"
DEFAULT_PATCH = (
    MACHINE / "out" / "reports" / "shaft_wrap_live_position_review_only.patch"
)

CURRENT_COMMIT = "6039b33c8f15a20086c2195c3f2d02b3a833e8ca"
PRE_REGRESSION_COMMIT = "8ae82f9e9ebf8cba7afe48e75e5d255d96bdfe3f"
REGRESSION_COMMIT = "8e7904a21f83854fb8034fa1f7612f88e9083c58"
RAW_EXPECTED_TURNS = (11.0 / 8.0, 67.0 / 24.0)
SERIAL_EXPECTED_TURNS = (1.3749395533708841, 2.791736856774936)
TWO_TURN_RADIANS = 4.0 * math.pi
SERIAL_COMMAND_QUANTIZATION_RAD_MAX = 0.0005


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _sha256(path: Path) -> str:
    return _sha256_bytes(Path(path).read_bytes())


def _git_bytes(winder: Path, *args: str) -> bytes:
    return subprocess.check_output(["git", "-C", str(winder), *args])


def _git_text(winder: Path, *args: str) -> str:
    return _git_bytes(winder, *args).decode("utf-8").strip()


def _method_block(source: str, method_name: str) -> tuple[int, str]:
    lines = source.splitlines()
    signature = f"    def {method_name}("
    start = next(
        index for index, line in enumerate(lines)
        if line.startswith(signature)
    )
    end = next(
        (
            index for index in range(start + 1, len(lines))
            if lines[index].startswith("    def ")
        ),
        len(lines),
    )
    return start + 1, "\n".join(lines[start:end]).rstrip() + "\n"


def _wrap_rows(capture: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    events = load_events(capture)
    timeline = Timeline(events)
    meta = timeline.meta
    calls = [row for row in events if row.get("e") == "wind_wire_around_shaft"]
    rows: list[dict[str, Any]] = []
    for ordinal, call in enumerate(calls, 1):
        start_t = float(call["t"])
        command = next(
            row for row in events
            if row.get("e") == "cmd" and row.get("m") == 1
            and float(row["t"]) >= start_t - 1.0e-12
        )
        start = float(timeline.axes[1].pos_at(start_t))
        target = float(command["model_target"])
        delta = target - start
        rows.append({
            "index": ordinal,
            "next_wire_index": int(call["args"][0]),
            "start_time_s": start_t,
            "live_start_rad": start,
            "absolute_target_rad": target,
            "delta_rad": delta,
            "completed_turns": abs(delta) / (2.0 * math.pi),
            "command": str(command["command"]).rstrip("\n"),
            "exactly_two_turns": math.isclose(
                abs(delta), TWO_TURN_RADIANS, rel_tol=0.0, abs_tol=1.0e-9,
            ),
        })
    return meta, rows


def _build_review_patch(
    current_source: str,
    current_tests: str,
) -> tuple[str, str, str]:
    """Return minimally edited source/tests and one unified review patch."""

    old_anchor = (
        "        starting_from_cw = self.is_starting_from_cw(next_wire_idx)\n"
        "\n"
        "        if starting_from_cw:\n"
    )
    new_anchor = (
        "        starting_from_cw = self.is_starting_from_cw(next_wire_idx)\n"
        "        motor1_pos = self.get_motor_position(1)\n"
        "\n"
        "        if starting_from_cw:\n"
    )
    if current_source.count(old_anchor) != 1:
        raise ValueError("current shaft-wrap anchor is not unique")
    edited = current_source.replace(old_anchor, new_anchor, 1)
    replacements = (
        (
            "            self.move_motor(1, self.m1_zero - motor1_rotation)\n",
            "            self.move_motor(1, motor1_pos - motor1_rotation)\n",
        ),
        (
            "            self.move_motor(1, self.m1_zero + motor1_rotation)\n",
            "            self.move_motor(1, motor1_pos + motor1_rotation)\n",
        ),
    )
    for old, new in replacements:
        if edited.count(old) != 1:
            raise ValueError(f"current shaft-wrap target is not unique: {old!r}")
        edited = edited.replace(old, new, 1)
    source_diff = "".join(difflib.unified_diff(
        current_source.splitlines(keepends=True),
        edited.splitlines(keepends=True),
        fromfile="a/src/winding.py",
        tofile="b/src/winding.py",
        n=4,
    ))
    if not source_diff:
        raise ValueError("review patch unexpectedly empty")

    import_old = "from src.winding import Wind\n"
    import_new = "from src.winding import Motor2State, Wind\n"
    if current_tests.count(import_old) != 1:
        raise ValueError("current winding-test import anchor is not unique")
    edited_tests = current_tests.replace(import_old, import_new, 1)
    regression_test = '''

@pytest.mark.parametrize(
    ("next_wire_idx", "starting_from_cw", "live_m1_rad", "expected_delta_rad"),
    [
        (1, True, -3.926990817, -4.0 * math.pi),
        (2, False, -17.540558983, 4.0 * math.pi),
    ],
)
def test_shaft_wrap_is_two_turns_from_live_m1(
    next_wire_idx, starting_from_cw, live_m1_rad, expected_delta_rad,
):
    wind = Wind(config_file_24n22p, True, turns=1)
    wind.is_starting_from_cw = lambda _next_wire_idx: starting_from_cw
    wind.motor2_pos = Motor2State.TOP_LEFT
    commands = []

    def position(motor_id):
        return live_m1_rad if motor_id == 1 else 0.0

    wind.get_motor_position = position
    wind.move_motor = lambda motor_id, target, **_kwargs: commands.append(
        (motor_id, target)
    )
    wind.wind_wire_around_shaft(next_wire_idx)

    m1_target = next(target for motor_id, target in commands if motor_id == 1)
    assert m1_target - live_m1_rad == pytest.approx(
        expected_delta_rad, abs=1.0e-12
    )
'''
    if "def test_shaft_wrap_is_two_turns_from_live_m1(" in edited_tests:
        raise ValueError("shaft-wrap regression test already exists")
    edited_tests = edited_tests.rstrip("\n") + "\n\n\n" + (
        regression_test.strip("\n") + "\n"
    )
    if "import math\n" not in edited_tests:
        edited_tests = edited_tests.replace(
            "import time\n", "import math\nimport time\n", 1,
        )
    test_diff = "".join(difflib.unified_diff(
        current_tests.splitlines(keepends=True),
        edited_tests.splitlines(keepends=True),
        fromfile="a/tests/test_winding.py",
        tofile="b/tests/test_winding.py",
        n=4,
    ))
    if not test_diff:
        raise ValueError("review regression-test patch unexpectedly empty")
    review_patch = (
        "diff --git a/src/winding.py b/src/winding.py\n"
        + source_diff
        + "diff --git a/tests/test_winding.py b/tests/test_winding.py\n"
        + test_diff
    )
    return edited, edited_tests, review_patch


def _patch_applies_cleanly(winder: Path, patch: str) -> tuple[bool, str]:
    result = subprocess.run(
        ["git", "-C", str(winder), "apply", "--check", "--recount", "-"],
        input=patch,
        text=True,
        capture_output=True,
        check=False,
    )
    detail = (result.stdout + result.stderr).strip()
    return result.returncode == 0, detail


def analyze(
    *,
    winder: Path = DEFAULT_WINDER,
    raw_capture: Path = DEFAULT_RAW_CAPTURE,
    serial_capture: Path = DEFAULT_SERIAL_CAPTURE,
    serial_harness: Path = DEFAULT_SERIAL_HARNESS,
    patch_path: Path = DEFAULT_PATCH,
) -> tuple[dict[str, Any], str]:
    winder = Path(winder).resolve()
    raw_capture = Path(raw_capture).resolve()
    serial_capture = Path(serial_capture).resolve()
    serial_harness = Path(serial_harness).resolve()
    patch_path = Path(patch_path).resolve()
    source_path = winder / "src" / "winding.py"
    tests_path = winder / "tests" / "test_winding.py"

    commit = _git_text(winder, "rev-parse", "HEAD")
    worktree_status = _git_text(winder, "status", "--porcelain")
    current_source_bytes = source_path.read_bytes()
    # ``Path.read_text`` performs universal-newline normalization.  Keep the
    # byte hash above authoritative while generating a portable LF patch that
    # Git can check against either an LF or core.autocrlf worktree.
    current_source = source_path.read_text(encoding="utf-8")
    current_source_sha = _sha256_bytes(current_source_bytes)
    current_source_blob = _git_text(winder, "rev-parse", "HEAD:src/winding.py")
    current_tests_bytes = tests_path.read_bytes()
    current_tests = tests_path.read_text(encoding="utf-8")
    current_tests_sha = _sha256_bytes(current_tests_bytes)
    current_tests_blob = _git_text(
        winder, "rev-parse", "HEAD:tests/test_winding.py",
    )
    current_method_line, current_method = _method_block(
        current_source, "wind_wire_around_shaft",
    )

    pre_source_bytes = _git_bytes(
        winder, "show", f"{PRE_REGRESSION_COMMIT}:src/winding.py",
    )
    pre_source = pre_source_bytes.decode("utf-8")
    pre_method_line, pre_method = _method_block(
        pre_source, "wind_wire_around_shaft",
    )
    pre_source_blob = _git_text(
        winder, "rev-parse", f"{PRE_REGRESSION_COMMIT}:src/winding.py",
    )
    regression_parent = _git_text(winder, "rev-parse", f"{REGRESSION_COMMIT}^")

    raw_meta, raw_wraps = _wrap_rows(raw_capture)
    serial_meta, serial_wraps = _wrap_rows(serial_capture)
    edited_source, edited_tests, review_patch = _build_review_patch(
        current_source, current_tests,
    )
    patch_clean, patch_detail = _patch_applies_cleanly(winder, review_patch)

    raw_turns = [float(row["completed_turns"]) for row in raw_wraps]
    serial_turns = [float(row["completed_turns"]) for row in serial_wraps]
    raw_expected = len(raw_turns) == 2 and all(
        math.isclose(value, expected, rel_tol=0.0, abs_tol=1.0e-9)
        for value, expected in zip(raw_turns, RAW_EXPECTED_TURNS)
    )
    serial_expected = len(serial_turns) == 2 and all(
        math.isclose(value, expected, rel_tol=0.0, abs_tol=1.0e-12)
        for value, expected in zip(serial_turns, SERIAL_EXPECTED_TURNS)
    )
    serial_meta_bound = (
        serial_meta.get("winder_commit") == CURRENT_COMMIT
        and serial_meta.get("winding_source_sha256") == current_source_sha
        and serial_meta.get("serial_twin_source_sha256") == _sha256(serial_harness)
        and serial_meta.get("controller_mode") == "upstream"
        and serial_meta.get("upstream_transport")
        == "serial_position_digital_twin"
        and serial_meta.get("upstream_source_subclassed") is False
        and serial_meta.get("upstream_source_modified_by_harness") is False
    )

    pre_source_proof = {
        "commit": PRE_REGRESSION_COMMIT,
        "source_blob_oid": pre_source_blob,
        "source_sha256": _sha256_bytes(pre_source_bytes),
        "method_start_line": pre_method_line,
        "method_sha256": _sha256_bytes(pre_method.encode("utf-8")),
        "queries_live_m1": "motor1_pos = self.get_motor_position(1)" in pre_method,
        "sets_rotation_count_two": "rotation_count = 2" in pre_method,
        "forms_four_pi": (
            "motor1_rotation = math.pi * 2 * rotation_count" in pre_method
        ),
        "commands_from_live_position": (
            "self.move_motor(1, motor1_pos + motor1_rotation)" in pre_method
        ),
        "requested_delta_rad": TWO_TURN_RADIANS,
        "requested_delta_turns": TWO_TURN_RADIANS / (2.0 * math.pi),
        "serial_command_quantization_rad_max": SERIAL_COMMAND_QUANTIZATION_RAD_MAX,
        "serial_command_turn_error_max": (
            SERIAL_COMMAND_QUANTIZATION_RAD_MAX / (2.0 * math.pi)
        ),
    }
    pre_source_proves_two = all((
        pre_source_proof["queries_live_m1"],
        pre_source_proof["sets_rotation_count_two"],
        pre_source_proof["forms_four_pi"],
        pre_source_proof["commands_from_live_position"],
        math.isclose(
            float(pre_source_proof["requested_delta_turns"]),
            2.0,
            rel_tol=0.0,
            abs_tol=1.0e-15,
        ),
    ))

    patch_method_line, patch_method = _method_block(
        edited_source, "wind_wire_around_shaft",
    )
    patch_predicted = [
        {
            "wrap": index,
            "requested_delta_rad": TWO_TURN_RADIANS,
            "requested_delta_turns": 2.0,
            "serial_command_turn_error_max": (
                SERIAL_COMMAND_QUANTIZATION_RAD_MAX / (2.0 * math.pi)
            ),
        }
        for index in (1, 2)
    ]
    report: dict[str, Any] = {
        "schema": "shaft-wrap-regression-evidence/v1",
        "status": "FAIL_CLOSED",
        "decision": "CURRENT_PINNED_UPSTREAM_DOES_NOT_EXECUTE_TWO_SHAFT_WRAPS",
        "release_authority": False,
        "scope": (
            "Standalone review evidence only; no candidate, active audit, "
            "adapter, player, controller, or upstream source is modified."
        ),
        "current_upstream": {
            "path": str(winder),
            "commit": commit,
            "expected_commit": CURRENT_COMMIT,
            "worktree_clean": worktree_status == "",
            "status_porcelain": worktree_status.splitlines(),
            "source_path": str(source_path),
            "source_blob_oid": current_source_blob,
            "source_sha256": current_source_sha,
            "method_start_line": current_method_line,
            "method_sha256": _sha256_bytes(current_method.encode("utf-8")),
            "uses_bookkeeping_zero_targets": (
                "self.move_motor(1, self.m1_zero - motor1_rotation)" in current_method
                and "self.move_motor(1, self.m1_zero + motor1_rotation)" in current_method
            ),
            "queries_live_m1_inside_wrap": (
                "get_motor_position(1)" in current_method
            ),
            "tests_path": str(tests_path),
            "tests_blob_oid": current_tests_blob,
            "tests_sha256": current_tests_sha,
        },
        "current_raw_capture": {
            "path": str(raw_capture),
            "sha256": _sha256(raw_capture),
            "capture_schema": raw_meta.get("capture_schema"),
            "controller_mode": raw_meta.get("controller_mode"),
            "winder_commit": raw_meta.get("winder_commit"),
            "wraps": raw_wraps,
            "observed_turns": raw_turns,
            "expected_turns": list(RAW_EXPECTED_TURNS),
            "matches_expected_regression": raw_expected,
        },
        "independent_serial_position_evidence": {
            "path": str(serial_capture),
            "sha256": _sha256(serial_capture),
            "capture_schema": serial_meta.get("capture_schema"),
            "controller_mode": serial_meta.get("controller_mode"),
            "transport": serial_meta.get("upstream_transport"),
            "winder_commit": serial_meta.get("winder_commit"),
            "winding_source_sha256": serial_meta.get("winding_source_sha256"),
            "harness_path": str(serial_harness),
            "harness_sha256": _sha256(serial_harness),
            "capture_harness_sha256": serial_meta.get("serial_twin_source_sha256"),
            "upstream_source_subclassed": serial_meta.get(
                "upstream_source_subclassed"
            ),
            "upstream_source_modified_by_harness": serial_meta.get(
                "upstream_source_modified_by_harness"
            ),
            "wraps": serial_wraps,
            "observed_turns": serial_turns,
            "expected_quantized_turns": list(SERIAL_EXPECTED_TURNS),
            "matches_expected_regression": serial_expected,
            "evidence_bound_to_current_source_and_harness": serial_meta_bound,
            "interpretation": (
                "A serial-protocol position twin with live axis state reproduces "
                "both non-two-turn moves because current upstream never queries "
                "M1 inside wind_wire_around_shaft."
            ),
        },
        "pre_regression_two_turn_source_evidence": pre_source_proof,
        "regression_boundary": {
            "first_bad_commit": REGRESSION_COMMIT,
            "first_bad_parent": regression_parent,
            "parent_is_pre_regression_commit": (
                regression_parent == PRE_REGRESSION_COMMIT
            ),
            "change": (
                "8e7904a replaced a target based on the queried live M1 pose "
                "with absolute m1_zero +/-4pi targets."
            ),
        },
        "review_only_patch": {
            "path": str(patch_path),
            "sha256": _sha256_bytes(review_patch.encode("utf-8")),
            "applied": False,
            "git_apply_check_pass": patch_clean,
            "git_apply_check_detail": patch_detail,
            "method_start_line_after_patch": patch_method_line,
            "method_sha256_after_patch": _sha256_bytes(
                patch_method.encode("utf-8")
            ),
            "restores_live_position_query": (
                "motor1_pos = self.get_motor_position(1)" in patch_method
            ),
            "restores_live_position_minus_four_pi": (
                "self.move_motor(1, motor1_pos - motor1_rotation)" in patch_method
            ),
            "restores_live_position_plus_four_pi": (
                "self.move_motor(1, motor1_pos + motor1_rotation)" in patch_method
            ),
            "adds_two_start_angle_regression_cases": (
                "def test_shaft_wrap_is_two_turns_from_live_m1(" in edited_tests
                and edited_tests.count("live_m1_rad") >= 4
                and "-3.926990817" in edited_tests
                and "-17.540558983" in edited_tests
            ),
            "changed_files": ["src/winding.py", "tests/test_winding.py"],
            "predicted_requested_moves": patch_predicted,
            "note": (
                "Review artifact only. Upstream must accept a correction and a "
                "new untouched capture must pass before release can open."
            ),
        },
    }
    report["gates"] = {
        "current_commit_and_clean_source_bound": (
            commit == CURRENT_COMMIT and worktree_status == ""
        ),
        "raw_capture_bound_to_current_commit": (
            raw_meta.get("winder_commit") == CURRENT_COMMIT
            and raw_meta.get("controller_mode") == "upstream"
        ),
        "raw_capture_reproduces_1p375_and_2p7916667_turns": raw_expected,
        "serial_position_evidence_bound": serial_meta_bound,
        "serial_position_evidence_reproduces_regression": serial_expected,
        "pre_regression_commit_is_first_bad_parent": (
            regression_parent == PRE_REGRESSION_COMMIT
        ),
        "pre_regression_source_proves_live_position_two_turn_request": (
            pre_source_proves_two
        ),
        "review_patch_applies_cleanly_without_being_applied": (
            patch_clean and worktree_status == ""
        ),
        "review_patch_contains_focused_regression_test": (
            report["review_only_patch"][
                "adds_two_start_angle_regression_cases"
            ]
        ),
        "current_upstream_satisfies_two_turn_requirement": False,
        "release_authorized": False,
    }
    evidence_gates = [
        value for key, value in report["gates"].items()
        if key not in {
            "current_upstream_satisfies_two_turn_requirement",
            "release_authorized",
        }
    ]
    report["evidence_bundle_complete"] = all(evidence_gates)
    return report, review_patch


def _markdown(report: dict[str, Any]) -> str:
    raw = report["current_raw_capture"]["wraps"]
    serial = report["independent_serial_position_evidence"]["wraps"]
    historical = report["pre_regression_two_turn_source_evidence"]
    patch = report["review_only_patch"]
    lines = [
        "# Shaft-wrap upstream regression evidence",
        "",
        f"**Status: {report['status']} - release authority remains false.**",
        "",
        "The pinned untouched upstream source commands the two inter-phase M1 "
        "moves from its bookkeeping zero, not from the live M1 pose. The "
        "authoritative raw capture completes 1.375000000 and 2.791666667 turns. "
        "An independent serial-position twin reproduces the same defect after "
        "normal three-decimal controller quantization.",
        "",
        "| evidence | wrap 1 turns | wrap 2 turns | source/transport binding |",
        "|---|---:|---:|:---:|",
        f"| raw upstream capture | {raw[0]['completed_turns']:.9f} | "
        f"{raw[1]['completed_turns']:.9f} | commit {CURRENT_COMMIT[:7]} |",
        f"| independent serial-position twin | {serial[0]['completed_turns']:.9f} | "
        f"{serial[1]['completed_turns']:.9f} | source and harness SHA-256 bound |",
        "",
        "## Regression boundary",
        "",
        f"Commit `{PRE_REGRESSION_COMMIT}` is the direct parent of first-bad "
        f"commit `{REGRESSION_COMMIT}`. Its exact `src/winding.py` blob "
        f"(`{historical['source_blob_oid']}`) queries live M1 and requests "
        "`live_position +/- 4*pi`: exactly two requested turns, independent of "
        "the starting angle. Serial command rounding can add at most "
        f"{historical['serial_command_turn_error_max']:.9f} turn.",
        "",
        "## Review-only correction artifact",
        "",
        f"`{patch['path']}` restores the live-position query and the two "
        "relative `+/-4*pi` targets, and adds two nonzero-start-angle "
        "regression cases. `git apply --check` passes against the pinned "
        "checkout; the patch was not applied.",
        "",
        "## Fail-closed rule",
        "",
        "This evidence does not authorize motion or release. Authority remains "
        "false until upstream accepts a correction and a new untouched upstream "
        "capture demonstrates both physical wraps under the production controller.",
        "",
        "All current capture, source, historical blob, harness, and patch hashes "
        "are recorded in the JSON companion report.",
    ]
    return "\n".join(lines) + "\n"


def generate(
    *,
    winder: Path = DEFAULT_WINDER,
    raw_capture: Path = DEFAULT_RAW_CAPTURE,
    serial_capture: Path = DEFAULT_SERIAL_CAPTURE,
    serial_harness: Path = DEFAULT_SERIAL_HARNESS,
    json_path: Path = DEFAULT_JSON,
    markdown_path: Path = DEFAULT_MD,
    patch_path: Path = DEFAULT_PATCH,
) -> dict[str, Any]:
    report, patch = analyze(
        winder=winder,
        raw_capture=raw_capture,
        serial_capture=serial_capture,
        serial_harness=serial_harness,
        patch_path=patch_path,
    )
    json_path = Path(json_path).resolve()
    markdown_path = Path(markdown_path).resolve()
    patch_path = Path(patch_path).resolve()
    for path in (json_path, markdown_path, patch_path):
        path.parent.mkdir(parents=True, exist_ok=True)
    patch_path.write_text(patch, encoding="utf-8", newline="\n")
    json_path.write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8", newline="\n",
    )
    markdown_path.write_text(
        _markdown(report), encoding="utf-8", newline="\n",
    )
    return report


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--winder", type=Path, default=DEFAULT_WINDER)
    parser.add_argument("--raw-capture", type=Path, default=DEFAULT_RAW_CAPTURE)
    parser.add_argument(
        "--serial-capture", type=Path, default=DEFAULT_SERIAL_CAPTURE,
    )
    parser.add_argument(
        "--serial-harness", type=Path, default=DEFAULT_SERIAL_HARNESS,
    )
    parser.add_argument("--json", type=Path, default=DEFAULT_JSON)
    parser.add_argument("--markdown", type=Path, default=DEFAULT_MD)
    parser.add_argument("--patch", type=Path, default=DEFAULT_PATCH)
    args = parser.parse_args(list(argv) if argv is not None else None)
    report = generate(
        winder=args.winder,
        raw_capture=args.raw_capture,
        serial_capture=args.serial_capture,
        serial_harness=args.serial_harness,
        json_path=args.json,
        markdown_path=args.markdown,
        patch_path=args.patch,
    )
    print(json.dumps({
        "status": report["status"],
        "evidence_bundle_complete": report["evidence_bundle_complete"],
        "raw_turns": report["current_raw_capture"]["observed_turns"],
        "serial_turns": report[
            "independent_serial_position_evidence"
        ]["observed_turns"],
        "patch_apply_check": report[
            "review_only_patch"
        ]["git_apply_check_pass"],
        "release_authority": report["release_authority"],
        "json": str(Path(args.json).resolve()),
        "markdown": str(Path(args.markdown).resolve()),
        "patch": str(Path(args.patch).resolve()),
    }, indent=2))
    return 0 if report["evidence_bundle_complete"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
