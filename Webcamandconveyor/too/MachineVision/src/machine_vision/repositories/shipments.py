from __future__ import annotations

import os
import threading
import time
from typing import Any, Protocol

# Only the routing fields are read; recipient name/phone/address stay in the database
# so they never reach the station's evidence files.
DEFAULT_ORDER_COLUMNS = "id,tracking_code,destination_chute_id,destination_hub,routing_code,status"


class ShipmentRepository(Protocol):
    def find_by_waybill(self, code: str) -> dict[str, Any] | None: ...


class LookupUnavailable(RuntimeError):
    """The order could not be looked up (network / database down): not "not found"."""


class InMemoryShipmentRepository:
    def __init__(self, records: list[dict[str, Any]] | None = None, key: str = "waybill_code"):
        self.key = key
        self.records = {str(row[key]).upper(): row for row in (records or []) if key in row}

    def find_by_waybill(self, code: str) -> dict[str, Any] | None:
        return self.records.get(code.upper())


def create_supabase_client(timeout_s: float = 4.0) -> Any:
    """Create the Supabase client from SUPABASE_URL / SUPABASE_KEY.

    ``timeout_s`` bounds every request: a dead connection must not hold the station.
    """
    try:
        from supabase import ClientOptions, create_client  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "Chưa cài thư viện supabase (pip install supabase python-dotenv)"
        ) from exc
    url = os.environ.get("SUPABASE_URL")
    key = os.environ.get("SUPABASE_KEY")
    if not url or not key:
        raise RuntimeError("Thiếu SUPABASE_URL / SUPABASE_KEY trong file .env")
    return create_client(url, key, options=ClientOptions(postgrest_client_timeout=timeout_s))


class SupabaseShipmentRepository:
    def __init__(self, settings: dict, client: Any | None = None):
        self.client = client if client is not None else create_supabase_client()
        self.table = str(settings.get("table", "orders"))
        self.column = str(settings.get("waybill_column", "tracking_code"))
        self.columns = str(settings.get("select_columns", DEFAULT_ORDER_COLUMNS))

    def find_by_waybill(self, code: str) -> dict[str, Any] | None:
        response = (
            self.client.table(self.table)
            .select(self.columns)
            .eq(self.column, code)
            .limit(1)
            .execute()
        )
        rows = response.data or []
        return rows[0] if rows else None


class CachedShipmentRepository:
    """Order lookup from a local copy of the routing columns (2026-11-25).

    All orders' routing fields (a few dozen rows, no personal data) are loaded at start
    and refreshed every ``refresh_s`` on a background thread, so a lookup during the
    belt cycle is instant and keeps working through a short network drop. A code not in
    the copy (order created since the last refresh) is asked from the database once;
    if that fails, ``LookupUnavailable`` is raised (the classifier then answers
    MANUAL_REVIEW instead of a false "not in database" REJECT).
    """

    def __init__(self, live: SupabaseShipmentRepository, refresh_s: float = 60.0,
                 start_thread: bool = True):
        self.live = live
        self.refresh_s = refresh_s
        self._rows: dict[str, dict[str, Any]] = {}
        self._lock = threading.Lock()
        self.loaded_at: float | None = None
        self.last_error: str | None = None
        self._stop = threading.Event()
        self.refresh()
        if start_thread:
            threading.Thread(target=self._loop, name="orders-cache", daemon=True).start()

    def refresh(self) -> bool:
        try:
            response = self.live.client.table(self.live.table).select(self.live.columns).execute()
        except Exception as exc:  # keep the previous copy
            self.last_error = str(exc) or exc.__class__.__name__
            return False
        rows = {
            str(row[self.live.column]).upper(): row
            for row in (response.data or []) if row.get(self.live.column)
        }
        with self._lock:
            self._rows = rows
        self.loaded_at, self.last_error = time.time(), None
        return True

    def find_by_waybill(self, code: str) -> dict[str, Any] | None:
        key = code.upper()
        with self._lock:
            row = self._rows.get(key)
        if row is not None:
            return row
        try:
            row = self.live.find_by_waybill(code)
        except Exception as exc:
            self.last_error = str(exc) or exc.__class__.__name__
            raise LookupUnavailable(
                f"Không tra cứu được đơn {code} trên Supabase: {self.last_error}"
            ) from exc
        if row is not None:
            with self._lock:
                self._rows[key] = row
        return row

    def __len__(self) -> int:
        with self._lock:
            return len(self._rows)

    def close(self) -> None:
        self._stop.set()

    def _loop(self) -> None:
        while not self._stop.wait(self.refresh_s):
            self.refresh()
