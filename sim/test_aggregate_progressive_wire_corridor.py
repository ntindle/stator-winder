import aggregate_progressive_wire_corridor as study


def _report():
    return study.analyze()


def test_all_raw_loci_and_contact_classes_are_covered():
    report = _report()
    assert report["raw_coverage"]["pass_count"] == 24
    assert report["raw_coverage"]["locus_count"] == 2400
    assert sum(report["raw_coverage"]["contact_class_counts"].values()) == 2400


def test_aggregate_does_not_require_exact_layer_schedule():
    report = _report()
    assert not report["aggregate_model"]["exact_layer_centres_predicted"]
    assert report["gates"]["aggregate_slot_capacity"]
    assert report["gates"]["support_scaffold_nonpenetrating"]


def test_all_raw_to_support_transfers_are_analytic_C1_R3_and_stack_bounded():
    report = _report()
    transfer = report["capture_transfer"]
    assert transfer["C1"]
    assert transfer["piece_radius_mm"] >= 3.0
    assert transfer["maximum_required_axial_run_mm"] <= transfer["available_stack_run_mm"]
    assert transfer["all_loci_mathematically_constructible"]


def test_free_space_curve_is_not_promoted_to_physical_support():
    report = _report()
    assert not report["capture_transfer"]["physical_support_authority"]
    assert not report["gates"]["physical_R3_first_turn_support_surface_identified"]
    assert "physical_R3_first_turn_support_surface_identified" in report["controlling_blockers"]


def test_report_stays_advisory_and_fail_closed():
    report = _report()
    assert report["status"] == "ADVISORY_NO_GO"
    assert not report["production_authorized"]
    assert not report["assembly_integration_authorized"]
    assert study._canonical_hash(report) == report["report_sha256"]
