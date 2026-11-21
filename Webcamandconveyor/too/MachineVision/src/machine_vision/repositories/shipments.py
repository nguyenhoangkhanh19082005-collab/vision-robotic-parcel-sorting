from __future__ import annotations

import os
from typing import Any, Protocol

# Only the routing fields are read; recipient name/phone/address stay in the database
# so they never reach the station's evidence files.
DEFAULT_ORDER_COLUMNS = "id,tracking_code,destination_chute_id,destination_hub,routing_code,status"


class ShipmentRepository(Protocol):
    def find_by_waybill(self, code: str) -> dict[str, Any] | None: ...


class InMemoryShipmentRepository:
    def __init__(self, records: list[dict[str, Any]] | None = None, key: str = "waybill_code"):
        self.key = key
        self.records = {str(row[key]).upper(): row for row in (records or []) if key in row}

    def find_by_waybill(self, code: str) -> dict[str, Any] | None:
        return self.records.get(code.upper())


def create_supabase_client() -> Any:
    """Create the Supabase client from SUPABASE_URL / SUPABASE_KEY."""
    try:
        from supabase import create_client  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "supabase is not installed; install the 'database' project extra"
        ) from exc
    url = os.environ.get("SUPABASE_URL")
    key = os.environ.get("SUPABASE_KEY")
    if not url or not key:
        raise RuntimeError("SUPABASE_URL and SUPABASE_KEY are required")
    return create_client(url, key)


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
