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


def build_pipeline(
    config: dict,
    project_root: Path | None = None,
    writable_root: Path | None = None,
) -> tuple[VisionPipeline, str | None]:
    project_root = project_root or resource_root()
    writable_root = writable_root or runtime_root()
    config = resolve_label_format(config)
    warnings = []
    logo_detector = TemplateLogoDetector(config["logo"], project_root, writable_root)
    if logo_detector.template is None:
        warnings.append(f"Logo template not found: {logo_detector.template_path}")
    damage_detector = PackageDamageDetector(config["damage"], project_root, writable_root)
    if damage_detector.production_reference is None:
        warnings.append(f"Empty-belt reference not found: {damage_detector.reference_path}")

    db_cfg = config["supabase"]
    if db_cfg.get("enabled", False):
        repository = SupabaseShipmentRepository(db_cfg, _supabase_client())
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
    pipeline, warning = build_pipeline(config, project_root, writable_root)
    evidence = EvidenceWriter(config.get("audit", {}), writable_root)
    reporter = build_reporter(config, writable_root)
    window = MainWindow(
        config, pipeline, evidence, project_root=project_root, reporter=reporter
    )
    window.show()
    if warning:
        QMessageBox.warning(window, "Commissioning required", warning)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
