"""Status upload for the automatic belt: background sender, cached order lookup (2026-11-25)."""

from __future__ import annotations

import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from machine_vision.models import ClassificationResult, Decision, ParcelData
from machine_vision.repositories.shipments import (
    CachedShipmentRepository,
    LookupUnavailable,
    SupabaseShipmentRepository,
)
from machine_vision.repositories.sorting_logs import SortingLogReporter
from machine_vision.services.classifier import ParcelClassifier


def _result(code="SPXVN76923235515", decision=Decision.PASS):
    parcel = ParcelData(qr_text=code, waybill_code=code)
    record = {"id": 20, "tracking_code": code, "destination_chute_id": 5}
    return SimpleNamespace(classification=ClassificationResult(decision, "5", "ok", parcel, record))


def _wait(predicate, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_report_returns_at_once_and_the_row_is_sent_in_the_background(tmp_path: Path):
    gate, sent = threading.Event(), []

    def slow_sender(row):          # a slow network must not hold the belt cycle
        gate.wait(2.0)
        sent.append(row)

    reporter = SortingLogReporter({}, slow_sender, tmp_path / "outbox.jsonl", background=True)
    started = time.monotonic()
    assert reporter.report(_result()) is None
    assert time.monotonic() - started < 0.2
    assert reporter.status().pending == 1          # durable before it is sent
    gate.set()
    assert _wait(lambda: reporter.status().pending == 0)
    assert sent[0]["tracking_code"] == "SPXVN76923235515" and sent[0]["chute_id"] == 5
    assert reporter.status().state == "ONLINE" and reporter.status().sent == 1
    reporter.close()


def test_offline_rows_wait_and_go_out_in_order_when_the_network_returns(tmp_path: Path):
    online, sent = threading.Event(), []

    def sender(row):
        if not online.is_set():
            raise ConnectionError("network down")
        sent.append(row["tracking_code"])

    outbox = tmp_path / "outbox.jsonl"
    reporter = SortingLogReporter({}, sender, outbox, background=True, retry_s=0.05)
    for code in ("A1", "B2", "C3"):
        reporter.report(_result(code))
    assert _wait(lambda: reporter.status().state == "OFFLINE")
    assert reporter.status().pending == 3 and "network down" in reporter.status().last_error
    reporter.close()
    # Station restarted while still offline: the outbox is kept and sent first.
    online.set()
    again = SortingLogReporter({}, sender, outbox, background=True, retry_s=0.05)
    again.report(_result("D4"))
    assert _wait(lambda: again.status().pending == 0)
    assert sent == ["A1", "B2", "C3", "D4"]
    again.close()


class _FakeTable:
    def __init__(self, client):
        self.client, self.code = client, None

    def select(self, _columns):
        return self

    def eq(self, _column, code):
        self.code = code
        return self

    def limit(self, _n):
        return self

    def execute(self):
        self.client.calls += 1
        if self.client.down:
            raise ConnectionError("timed out")
        rows = self.client.rows if self.code is None else [
            r for r in self.client.rows if r["tracking_code"] == self.code]
        return SimpleNamespace(data=rows)


class _FakeClient:
    def __init__(self, rows):
        self.rows, self.down, self.calls = rows, False, 0

    def table(self, _name):
        return _FakeTable(self)


def test_cached_lookup_survives_a_network_drop_and_unknown_codes_are_not_rejected():
    client = _FakeClient([{"id": 20, "tracking_code": "SPXVN76923235515", "destination_chute_id": 5}])
    repo = CachedShipmentRepository(SupabaseShipmentRepository({}, client), start_thread=False)
    assert len(repo) == 1
    client.down = True
    assert repo.find_by_waybill("spxvn76923235515")["destination_chute_id"] == 5   # from the copy
    with pytest.raises(LookupUnavailable):
        repo.find_by_waybill("SPXVN00000000000")
    classifier = ParcelClassifier({"destination_field": "destination_chute_id", "require_damage_check": False,
                                   "require_logo": False}, repo)
    parcel = ParcelData(qr_text="SPXVN00000000000", waybill_code="SPXVN00000000000")
    result = classifier.classify(parcel)
    assert result.decision == Decision.MANUAL_REVIEW and "Không tra cứu được" in result.reason
    client.down = False
    assert repo.find_by_waybill("SPXVN00000000000") is None                       # really unknown


def test_missing_env_falls_back_to_local_test_mode(monkeypatch):
    from machine_vision import app

    monkeypatch.delenv("SUPABASE_URL", raising=False)
    monkeypatch.delenv("SUPABASE_KEY", raising=False)
    monkeypatch.setattr(app, "_client_cache", [])
    config = {"supabase": {"enabled": True}, "classification": {"local_test_mode": False}}
    warning = app.prepare_supabase(config)
    assert warning and "kiểm tra cục bộ" in warning
    assert config["classification"]["local_test_mode"] is True
    assert config["supabase"]["enabled"] is False
    assert app.build_reporter(config, Path(".")) is None
