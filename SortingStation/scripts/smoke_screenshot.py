"""Chạy thử GUI hợp nhất (không cần camera/robot/băng tải), chạy 1 chu trình mô phỏng và chụp ảnh các tab.

Dùng:  E:\\UDMT\\.venv\\Scripts\\python.exe scripts\\smoke_screenshot.py [thư_mục_ảnh]
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sorting_station.station_config import install_import_paths, load_station_config, resolve_paths  # noqa: E402


def main() -> int:
    out = Path(sys.argv[1] if len(sys.argv) > 1 else ROOT / "reports" / "screenshots")
    out.mkdir(parents=True, exist_ok=True)
    config = load_station_config()
    paths = resolve_paths(config)
    install_import_paths(paths)

    from PyQt6.QtCore import QTimer
    from PyQt6.QtWidgets import QApplication

    from machine_vision.app import build_pipeline
    from machine_vision.config import load_config
    from machine_vision.paths import asset_root, runtime_root
    from machine_vision.services.evidence import EvidenceWriter
    from sorting_station.main_window import IntegratedMainWindow

    app = QApplication(sys.argv[:1])
    mv_config = load_config(paths.machine_vision_config).raw
    pipeline, _warning = build_pipeline(mv_config, asset_root(paths.machine_vision_config), runtime_root())
    evidence = EvidenceWriter({"enabled": False}, runtime_root())
    window = IntegratedMainWindow(mv_config, pipeline, evidence, asset_root(paths.machine_vision_config),
                                  config, paths)
    window.resize(1600, 960)
    window.show()
    errors: list[str] = []

    def shot(name: str) -> None:
        window.grab().save(str(out / f"{name}.png"))

    def step_operator():
        window.workspace_tabs.setCurrentIndex(0)
        window.sorting_controller.on_inspection("REJECT", "DEFECT", "SPXVN000TEST")
        window.cell.set_process(window.cell.stage.__class__.COMPLETE, "REJECT")
        window.sorting_controller.trigger_last()

    def mid_cycle():
        shot("1_van_hanh_giua_chu_trinh")

    def after_cycle():
        if window.sequencer.state != "DONE":
            errors.append(f"Chu trình mô phỏng không DONE: {window.sequencer.state}")
        shot("2_van_hanh_sau_chu_trinh")
        window.workspace_tabs.setCurrentIndex(1)

    def twin_tab():
        shot("3_digital_twin")
        window.workspace_tabs.setCurrentIndex(window.robot_tab_index)

    def robot_tab():
        shot("4_robot_6_truc")
        window.workspace_tabs.setCurrentIndex(window.robot_tab_index + 1)

    def sorting_tab():
        shot("5_phan_loai")
        window.workspace_tabs.setCurrentIndex(2)

    def conveyor_tab():
        shot("6_bang_tai")
        window.close()
        app.quit()

    schedule = [(800, step_operator), (4500, mid_cycle), (16000, after_cycle), (17500, twin_tab),
                (19000, robot_tab), (20500, sorting_tab), (21500, conveyor_tab)]
    for delay, func in schedule:
        QTimer.singleShot(delay, func)
    app.exec()
    print("ẢNH:", out)
    for error in errors:
        print("LỖI:", error)
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
