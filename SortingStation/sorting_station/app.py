"""Điểm khởi động app hợp nhất.

Chạy:  E:\\UDMT\\.venv\\Scripts\\python.exe -m sorting_station.app  (từ thư mục SortingStation)
"""

from __future__ import annotations

import argparse
import sys

from sorting_station.station_config import (
    DEFAULT_CONFIG,
    install_import_paths,
    load_station_config,
    resolve_paths,
)


def _utf8_console() -> None:
    # Module robot in log tiếng Việt; console Windows cp1252 sẽ lỗi UnicodeEncodeError
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass


def main(argv: list[str] | None = None) -> int:
    _utf8_console()
    parser = argparse.ArgumentParser(description="UDMT - trạm phân loại kiện hàng (Vision + băng tải + robot)")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="Đường dẫn station.yaml")
    args = parser.parse_args(argv)

    station_config = load_station_config(args.config)
    paths = resolve_paths(station_config)
    install_import_paths(paths)

    # Import sau khi đã thêm đường dẫn hai dự án gốc vào sys.path
    from PyQt6.QtWidgets import QApplication, QMessageBox

    from machine_vision.app import build_pipeline
    from machine_vision.config import load_config
    from machine_vision.paths import asset_root, runtime_root
    from machine_vision.services.evidence import EvidenceWriter
    from sorting_station.main_window import IntegratedMainWindow

    try:
        from dotenv import load_dotenv  # type: ignore

        load_dotenv(paths.machine_vision_root / ".env")
    except ImportError:
        pass

    mv_config = load_config(paths.machine_vision_config).raw
    project_root = asset_root(paths.machine_vision_config)
    writable_root = runtime_root()

    app = QApplication(sys.argv if argv is None else ["sorting_station", *argv])
    pipeline, warning = build_pipeline(mv_config, project_root, writable_root)
    evidence = EvidenceWriter(mv_config.get("audit", {}), writable_root)
    window = IntegratedMainWindow(mv_config, pipeline, evidence, project_root, station_config, paths)
    window.showMaximized()
    if warning:
        QMessageBox.warning(window, "Commissioning required", warning)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
