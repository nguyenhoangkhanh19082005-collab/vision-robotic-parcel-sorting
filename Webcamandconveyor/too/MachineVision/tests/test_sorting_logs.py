from pathlib import Path

import numpy as np

from machine_vision.models import (
    ClassificationResult,
    DamageAssessment,
    Decision,
    FocusMetrics,
    InspectionResult,
    LogoAssessment,
    ParcelData,
)
from machine_vision.repositories.sorting_logs import SortingLogReporter, build_sorting_log

ORDER = {"id": 8, "tracking_code": "SPXVN40228347634", "destination_chute_id": 1}


def _classification(decision, *, record=None, waybill="SPXVN40228347634", damage=None, logo=None):
    return ClassificationResult(
        decision=decision,
        destination_bin="1",
        reason="test",
        parcel=ParcelData(
            qr_text=waybill,
            waybill_code=waybill,
            logo=logo or LogoAssessment(True, True),
            damage=damage or DamageAssessment(True, False),
        ),
        database_record=record,
    )


def _inspection(classification):
    image = np.zeros((4, 4, 3), dtype=np.uint8)
    return InspectionResult(classification, FocusMetrics(1.0, 1.0), image, image)


def test_pass_goes_to_planned_chute():
    row = build_sorting_log(_classification(Decision.PASS, record=ORDER), {}, inspection_uuid="u1")
    assert row["chute_id"] == 1
    assert row["order_id"] == 8
    assert row["status"] == "SORTED"
    assert row["decision"] == "PASS"
    assert (row["qr_ok"], row["logo_ok"], row["damage_detected"]) == (True, True, False)


def test_damaged_parcel_goes_to_reject_chute_with_damage_types():
    damage = DamageAssessment(True, True, damage_types=("hole_or_puncture",))
    row = build_sorting_log(
        _classification(Decision.REJECT, record=ORDER, damage=damage),
        {"reject_chute_id": 0},
        inspection_uuid="u2",
    )
    assert row["chute_id"] == 0
    assert row["status"] == "REJECTED"
    assert row["damage_detected"] is True
    assert row["damage_types"] == ["hole_or_puncture"]


def test_unreadable_qr_and_uncalibrated_detectors_are_not_reported_as_good():
    classification = _classification(
        Decision.MANUAL_REVIEW,
        waybill=None,
        logo=LogoAssessment(False, None),
        damage=DamageAssessment(False, None),
    )
    row = build_sorting_log(classification, {"reject_chute_id": 9}, inspection_uuid="u3")
    assert row["tracking_code"] == "UNKNOWN"
    assert row["order_id"] is None
    assert row["chute_id"] == 9
    assert row["qr_ok"] is False
    assert row["logo_ok"] is None
    assert row["damage_detected"] is None


def test_failed_upload_is_queued_then_resent(tmp_path: Path):
    sent: list[dict] = []
    online = {"value": False}

    def sender(row):
        if not online["value"]:
            raise ConnectionError("offline")
        sent.append(row)

    reporter = SortingLogReporter({}, sender, tmp_path / "outbox.jsonl")
    result = _inspection(_classification(Decision.PASS, record=ORDER))

    assert reporter.report(result) is not None
    assert reporter.pending_count() == 1
    assert sent == []

    online["value"] = True
    assert reporter.report(result, tmp_path / "2026-11-10" / "abc") is None
    assert reporter.pending_count() == 0
    assert len(sent) == 2
    assert sent[0]["evidence_path"] == "2026-11-10/abc"
    assert len({row["inspection_uuid"] for row in sent}) == 2
