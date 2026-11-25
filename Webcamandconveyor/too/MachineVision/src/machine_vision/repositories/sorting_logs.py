"""Upload inspection results to the ``sorting_logs`` table of the data station.

One row is written per inspection. A database trigger (``sorting_logs_apply``)
links the row to its order and moves the order to ``SORTED`` when the decision is
``PASS``, so the station never updates ``orders`` directly.

Rows that cannot be sent (no network, wrong key) are appended to a local JSONL
outbox and retried after the next successful upload. ``inspection_uuid`` is unique
in the database, so resending a row is harmless.

2026-11-25 (automatic belt cycle): ``background=True`` makes ``report()`` return at
once. The row is written to the outbox first (survives a crash or power cut) and a
sender thread uploads the outbox in order, retrying every ``retry_s`` while offline.
The belt never waits for the network (the server is in Tokyo; a slow or dropped
connection used to hold the parcel under the camera). ``status()`` feeds the GUI.
"""

from __future__ import annotations

import json
import os
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from uuid import uuid4

from machine_vision.models import ClassificationResult, Decision, InspectionResult

DEFAULT_STATUS_MAP = {
    Decision.PASS.value: "SORTED",
    Decision.REJECT.value: "REJECTED",
    Decision.MANUAL_REVIEW.value: "MANUAL_REVIEW",
}

Sender = Callable[[dict[str, Any]], None]


def build_sorting_log(
    classification: ClassificationResult,
    settings: dict,
    *,
    inspection_uuid: str,
    destination_field: str = "destination_chute_id",
    evidence_path: str | None = None,
) -> dict[str, Any]:
    """Translate one classification into a ``sorting_logs`` row."""
    parcel = classification.parcel
    record = classification.database_record or {}
    decision = classification.decision.value
    reject_chute = int(settings.get("reject_chute_id", 0))

    # Only a PASS parcel goes to the chute planned for its order; everything else
    # (damaged, bad label, unknown code, uncalibrated detector) goes to the reject chute.
    chute_id = reject_chute
    if classification.decision == Decision.PASS:
        try:
            chute_id = int(record[destination_field])
        except (KeyError, TypeError, ValueError):
            chute_id = reject_chute

    logo = parcel.logo
    damage = parcel.damage
    status_map = {**DEFAULT_STATUS_MAP, **(settings.get("status_map") or {})}
    return {
        "inspection_uuid": inspection_uuid,
        "tracking_code": parcel.waybill_code or str(settings.get("unknown_tracking_code", "UNKNOWN")),
        "order_id": record.get("id"),
        "chute_id": chute_id,
        "status": str(status_map[decision]),
        "decision": decision,
        "reason": classification.reason,
        "qr_ok": bool(parcel.qr_text),
        # None means "not checked / detector not calibrated", never "good".
        "logo_ok": logo.present if logo is not None and logo.calibrated else None,
        "damage_detected": damage.damaged if damage is not None and damage.calibrated else None,
        "damage_types": list(damage.damage_types) if damage is not None else [],
        "station_id": str(settings.get("station_id", "VISION-01")),
        "scanned_from": str(settings.get("scanned_from", "MACHINE_VISION")),
        "scan_attempts": 1,
        "evidence_path": evidence_path,
    }


@dataclass(frozen=True)
class UploadStatus:
    state: str              # "READY" (nothing sent yet), "ONLINE", "OFFLINE"
    pending: int            # rows waiting in the outbox
    sent: int               # rows uploaded since start
    last_error: str | None
    last_sent_at: float | None


class SortingLogReporter:
    """Send inspection rows through ``sender`` and keep failed rows in an outbox."""

    def __init__(
        self,
        settings: dict,
        sender: Sender,
        outbox_path: Path,
        destination_field: str = "destination_chute_id",
        background: bool = False,
        retry_s: float = 10.0,
    ):
        self.settings = settings
        self.sender = sender
        self.outbox_path = outbox_path
        self.destination_field = destination_field
        self.retry_s = retry_s
        self._lock = threading.RLock()        # outbox file + counters
        self._state = "READY"
        self._sent = 0
        self._last_error: str | None = None
        self._last_sent_at: float | None = None
        self._wake = threading.Event()
        self._closing = threading.Event()
        self._thread: threading.Thread | None = None
        if background:
            self._thread = threading.Thread(target=self._run, name="sorting-log-upload", daemon=True)
            self._thread.start()
            self._wake.set()                  # rows left from the last session go first

    def report(self, result: InspectionResult, evidence_folder: Path | None = None) -> str | None:
        """Upload one result. Returns ``None`` on success or an error message."""
        evidence_path = None
        if evidence_folder is not None:
            # "<date>/<inspection id>" is enough to find the folder under captures/.
            evidence_path = "/".join(evidence_folder.parts[-2:])
        row = build_sorting_log(
            result.classification,
            self.settings,
            inspection_uuid=str(uuid4()),
            destination_field=self.destination_field,
            evidence_path=evidence_path,
        )
        if self._thread is not None:
            # Durable first, then upload in the background (in order, after older rows).
            with self._lock:
                self._write_outbox([*self._read_outbox(), row])
            self._wake.set()
            return None
        try:
            self.sender(row)
        except Exception as exc:  # network/auth errors must not stop the station
            with self._lock:
                self._write_outbox([*self._read_outbox(), row])
                self._mark_failed(exc)
            return f"Chưa gửi được kết quả lên Supabase, đã lưu chờ gửi lại: {exc}"
        with self._lock:
            self._mark_sent()
        self.flush_outbox()
        return None

    def pending_count(self) -> int:
        with self._lock:
            return len(self._read_outbox())

    def status(self) -> UploadStatus:
        with self._lock:
            return UploadStatus(
                self._state, len(self._read_outbox()), self._sent, self._last_error, self._last_sent_at
            )

    def flush_outbox(self) -> int:
        """Retry queued rows in order; stop at the first failure. Returns rows sent."""
        with self._lock:
            pending = self._read_outbox()
        sent = 0
        for row in pending:
            try:
                self.sender(row)
            except Exception as exc:
                with self._lock:
                    self._mark_failed(exc)
                break
            sent += 1
            with self._lock:
                self._mark_sent()
        if sent:
            with self._lock:
                # Rows appended meanwhile stay after the ones just sent.
                self._write_outbox(self._read_outbox()[sent:])
        return sent

    def close(self, timeout_s: float = 3.0) -> None:
        """Stop the sender thread (rows not sent stay in the outbox for next time)."""
        self._closing.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout_s)

    def _run(self) -> None:
        while not self._closing.is_set():
            self._wake.wait(self.retry_s)
            self._wake.clear()
            if self._closing.is_set():
                break
            self.flush_outbox()

    def _mark_sent(self) -> None:
        self._state, self._last_error = "ONLINE", None
        self._sent += 1
        self._last_sent_at = time.time()

    def _mark_failed(self, exc: Exception) -> None:
        self._state, self._last_error = "OFFLINE", str(exc) or exc.__class__.__name__

    def _read_outbox(self) -> list[dict[str, Any]]:
        if not self.outbox_path.exists():
            return []
        rows = []
        for line in self.outbox_path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                rows.append(json.loads(line))
        return rows

    def _write_outbox(self, rows: list[dict[str, Any]]) -> None:
        if not rows:
            self.outbox_path.unlink(missing_ok=True)
            return
        self.outbox_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.outbox_path.with_suffix(self.outbox_path.suffix + ".tmp")
        temporary.write_text(
            "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows),
            encoding="utf-8",
        )
        os.replace(temporary, self.outbox_path)


def supabase_sender(client: Any, table: str = "sorting_logs") -> Sender:
    """Build a sender that inserts rows and ignores an already-uploaded inspection."""

    def send(row: dict[str, Any]) -> None:
        client.table(table).upsert(
            row, on_conflict="inspection_uuid", ignore_duplicates=True
        ).execute()

    return send
