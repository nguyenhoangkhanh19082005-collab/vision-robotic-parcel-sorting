from __future__ import annotations

import argparse
import sys
from pathlib import Path

from PyQt6.QtWidgets import QApplication, QMessageBox

from machine_vision.config import load_config, resolve_label_format
from machine_vision.paths import (
    asset_root,
    default_config_path,
    resource_root,
    runtime_root,
)
from machine_vision.readers.logo_reader import TemplateLogoDetector
from machine_vision.readers.qr_reader import QrReader
from machine_vision.repositories.shipments import (
    CachedShipmentRepository,
    InMemoryShipmentRepository,
    SupabaseShipmentRepository,
    create_supabase_client,
)
from machine_vision.repositories.sorting_logs import SortingLogReporter, supabase_sender
from machine_vision.services.classifier import ParcelClassifier
from machine_vision.services.damage import PackageDamageDetector
from machine_vision.services.evidence import EvidenceWriter
from machine_vision.services.pipeline import VisionPipeline
from machine_vision.ui.main_window import MainWindow
from machine_vision.ui.wheel_guard import install_wheel_guard


def build_pipeline(
    config: dict,
    project_root: Path | None = None,
    writable_root: Path | None = None,
) -> tuple[VisionPipeline, str | None]:
    project_root = project_root or resource_root()
    writable_root = writable_root or runtime_root()
    warnings = []
    # Before resolving: the caller passes this same dict to build_reporter / the window.
    supabase_warning = prepare_supabase(config)
    if supabase_warning:
        warnings.append(supabase_warning)
    config = resolve_label_format(config)
    logo_detector = TemplateLogoDetector(config["logo"], project_root, writable_root)
    if logo_detector.template is None:
        warnings.append(f"Logo template not found: {logo_detector.template_path}")
    damage_detector = PackageDamageDetector(config["damage"], project_root, writable_root)
    if damage_detector.production_reference is None:
        warnings.append(f"Empty-belt reference not found: {damage_detector.reference_path}")

    db_cfg = config["supabase"]
    if db_cfg.get("enabled", False):
        repository = CachedShipmentRepository(
            SupabaseShipmentRepository(db_cfg, _supabase_client()),
            refresh_s=float(db_cfg.get("cache_refresh_s", 60.0)),
        )
        if repository.loaded_at is None:
            warnings.append(
                f"Chưa tải được danh sách đơn từ Supabase ({repository.last_error}). "
                "Trạm vẫn chạy và tự thử lại; kết quả được giữ chờ gửi khi có mạng."
            )
    else:
        repository = InMemoryShipmentRepository([])

    classifier = ParcelClassifier(config["classification"], repository)
    pipeline = VisionPipeline(
        config,
        QrReader(config["qr"]),
        logo_detector,
        damage_detector,
        classifier,
    )
    return pipeline, ". ".join(warnings) if warnings else None


_client_cache: list = []


def _supabase_client():
    """One shared Supabase client for order lookup and result upload."""
    if not _client_cache:
        _client_cache.append(create_supabase_client())
    return _client_cache[0]


def prepare_supabase(config: dict) -> str | None:
    """Check that Supabase can be used; otherwise fall back to local test mode.

    Missing library or .env must not stop the station from starting (2026-11-25):
    the window opens in local test mode and says why nothing reaches the web.
    No network request is made here (the client connects lazily).
    """
    db_cfg = config.setdefault("supabase", {})
    classification = config.setdefault("classification", {})
    if not db_cfg.get("enabled", False) or classification.get("local_test_mode", False):
        return None
    try:
        _supabase_client()
    except Exception as exc:  # noqa: BLE001 - any setup problem means "not configured"
        db_cfg["enabled"] = False
        db_cfg["unavailable_reason"] = str(exc)
        classification["local_test_mode"] = True
        return (
            f"Supabase chưa sẵn sàng: {exc}. Trạm chạy chế độ kiểm tra cục bộ, "
            "kết quả KHÔNG được gửi lên web. Xem docs/HUONG_DAN_UPLOAD_TRANG_THAI.md."
        )
    return None


def build_reporter(config: dict, writable_root: Path) -> SortingLogReporter | None:
    """Result upload runs only against the real database, never in local test mode."""
    db_cfg = config["supabase"]
    classification = config.get("classification", {})
    if (
        not db_cfg.get("enabled", False)
        or not db_cfg.get("upload_results", True)
        or classification.get("local_test_mode", False)
    ):
        return None
    outbox = Path(str(db_cfg.get("outbox", "captures/supabase_outbox.jsonl")))
    return SortingLogReporter(
        db_cfg,
        supabase_sender(_supabase_client(), str(db_cfg.get("log_table", "sorting_logs"))),
        outbox if outbox.is_absolute() else writable_root / outbox,
        destination_field=str(classification.get("destination_field", "destination_chute_id")),
        background=True,
        retry_s=float(db_cfg.get("retry_s", 10.0)),
    )


def main() -> int:
    writable_root = runtime_root()
    try:
        from dotenv import load_dotenv  # type: ignore

        load_dotenv(writable_root / ".env")
    except ImportError:
        # dotenv is installed by the database extra; normal environment variables still work.
        pass
    parser = argparse.ArgumentParser(description="Machine Vision parcel sorting station")
    parser.add_argument(
        "--config",
        default=str(default_config_path()),
        help="Path to station YAML configuration",
    )
    args = parser.parse_args()
    config_path = Path(args.config).resolve()
    config = load_config(config_path).raw
    project_root = asset_root(config_path)

    app = QApplication(sys.argv)
    install_wheel_guard(app)
    pipeline, warning = build_pipeline(config, project_root, writable_root)
    evidence = EvidenceWriter(config.get("audit", {}), writable_root)
    reporter = build_reporter(config, writable_root)
    window = MainWindow(
        config, pipeline, evidence, project_root=project_root, reporter=reporter
    )
    window.show()
    if warning:
        QMessageBox.warning(window, "Commissioning required", warning)
    try:
        return app.exec()
    finally:
        if reporter is not None:
            reporter.close()


if __name__ == "__main__":
    raise SystemExit(main())
