# CHANGELOG — SortingStation

## 2026-10-21 — developer + tester (phiên chính)
- Tạo app hợp nhất `SortingStation` ghép GUI MachineVision và GUI robot 6 trục; mã MachineVision không bị sửa.
- Thêm `robot_model.py`: FK/IK PoE từ trục khớp CAD, thay bảng DH lệch tư thế 0 (xem báo cáo, R1).
- Thêm digital twin toàn trạm PyVista (`cell_twin.py`), trạng thái chung (`cell_state.py`).
- Thêm bộ tuần tự phân loại + dạy tư thế (`sorting.py`, `sorting_panel.py`), nút DỪNG TOÀN TRẠM.
- Thêm `robot_bridge.py`: nhúng GUI robot, đọc telemetry không chặn, API điều khiển.
- Tạo venv chung `E:\UDMT\.venv` (cache/temp: `E:\UDMT\.tmp`).
- Test: 16/16 test mới đạt; 47/47 test MachineVision đạt trong venv chung; smoke GUI đạt.
- Báo cáo: `reports\DANH_GIA_VA_DE_XUAT_2026-10-21.md`.
- File bị ảnh hưởng ngoài SortingStation: `robotarm-6-axis\main.py`, `robotarm-6-axis\Changes\main.py` (tham số `urdf_dir`, đường dẫn tuyệt đối).

## 2026-10-21 — developer (sửa giao diện tab ROBOT)
- `robot_bridge.py`: bọc các trang ROBOT CONTROLLER PANEL (JOINTS JOG, CARTESIAN, AUTOMATION, CONNECTION) trong QScrollArea — khi nhúng dưới header/KPI, 6 slider khớp bị ép chồng lên nhau. Test 16/16 đạt; ảnh `reports\screenshots\4_robot_6_truc.png`.

## 2026-10-22 — developer (mô hình 3D băng tải thật)
- Thêm `sorting_station/conveyor_model.py`: mô hình băng tải tham số hóa (mặt băng + 2 rulô, 4 nắp đầu, 2 thanh nhôm, 4 chân chữ L; KHÔNG gồm động cơ/puly/đai theo yêu cầu). Kích thước ước lượng từ `assets/conveyor_photos` (ảnh 05 có thước dây: dài 400 mm, băng rộng 100 mm; ảnh 01: mặt băng cao ≈ 82 mm). `kich_thuoc.txt` người dùng gửi đang trống.
- Thêm `scripts/build_conveyor_model.py` → `assets/conveyor/conveyor.stl` (mm), `conveyor.glb` (m, có màu), `conveyor_preview.png`.
- `cell_twin.py`: thay băng tải dạng hộp bằng mô hình thật; giá camera chỉ vẽ khi `layout.camera.show_mount: true` (chưa in); chỉnh góc nhìn cho cell nhỏ.
- `cell_state.py`: `CellLayout` đọc `layout.conveyor.{center, base_z, model}`; sửa lỗi biến `spec` bị vòng lặp bins ghi đè.
- `config/station.yaml`: bố trí theo băng tải thật (dài 0.9 m → 0.4 m), camera y=-0.20, điểm gắp (220, 0, 140) mm, kiện 110×150×60 mm.
- Thêm `tests/test_conveyor_model.py` (5 test). Kết quả: 21/21 test đạt; smoke GUI đạt.

## 2026-10-22 — developer + tester (tỉ lệ robot so với băng tải)
- Kiểm tra `Roboter.STEP` (trùng `robotarm-6-axis\Roboter.STEP`, đơn vị mm) bằng OpenCascade: hộp bao cụm lắp 467×369×256 mm khớp mesh twin 462×360×248 mm → mesh đúng tỉ lệ CAD.
- Phân tích phối cảnh ảnh `01_mat_ben_trai.jpg`: robot in thật ≈ 0.73–0.87 lần CAD → nguyên nhân lệch là robot thật nhỏ hơn CAD, băng tải (đo bằng thước) giữ nguyên.
- Thêm `robot.scale` (station.yaml, TẠM 0.80): nhân đồng thời cho mesh twin và mô hình động học/IK (`robot_model.build_model_from_robot_project(scale=)`, `cell_twin._load_mesh`); chiều dài đầu hút không nhân.
- Thêm `scripts/render_scale_check.py` → `reports/scale_check.png` (chiếu song song, lưới 50 mm).
- Test: thêm `test_robot_scale_scales_geometry_consistently`; 22/22 đạt (kể cả tầm với pick + 3 thùng ở scale 0.8).
- Môi trường: cài `cadquery-ocp` vào `E:\UDMT\.venv` (kéo VTK 9.7.1 → 9.6.2; chỉ phát sinh DeprecationWarning của NumPy 2.5, không ảnh hưởng chức năng).

## 2026-11-21 — developer (3 thùng theo vùng giao hàng miền Bắc)
- `config/station.yaml`: đổi 3 thùng `PASS` / `REJECT` / `REVIEW` thành `DONG_BANG` / `LOI` / `TRUNG_DU`; giữ nguyên vị trí và tọa độ thả (ĐẠT → ĐỒNG BẰNG, XEM LẠI → TRUNG DU, LỖI giữ nguyên). Thùng TRUNG DU đổi màu vàng → xanh dương.
- `destination_map` nhận số hub từ `orders.destination_chute_id`: 1, 3, 5 → ĐỒNG BẰNG; 2, 4 → TRUNG DU; 0 → LỖI; `LOCAL_TEST` → ĐỒNG BẰNG. `decision_map`: REJECT, MANUAL_REVIEW → LỖI. `fallback_bin` → LỖI.
- `sorting.py` `resolve_bin`: kiện không PASS luôn theo `decision_map` (trước đây `destination_bin` được ưu tiên cho mọi quyết định); kiện PASS có hub lạ → LỖI thay vì thùng ĐẠT. Mặc định `fallback_bin` → `LOI`.
- `tests/test_sorting.py`: cập nhật tên thùng, mở rộng `test_resolve_bin_mapping`; `README.md`: mô tả 3 thùng mới.
- Test: 15/22 đạt. 7 lỗi do IK không với tới điểm tiếp cận của cả 3 thùng và `test_ik_pieper_compatibility` — tọa độ thùng không đổi trong lần sửa này; nghi do thay đổi hình học/đầu hút robot ngày 21/11, chưa xử lý.
## 2026-11-21 — developer — Sửa lỗi robot không với tới thùng (7 test lỗi)
- Nguyên nhân: `robot.tool_length_mm` 30 → 126 (gá giác hút dài, robotarm-6-axis/dong_hoc_step.TOOL_LENGTH_MM) trong khi `robot.scale` = 0.80 (tạm, ước lượng từ ảnh). Ở 0.80, robot thu nhỏ 20 % cộng đầu hút dài nên điểm tiếp cận của cả 3 thùng ngoài vùng với. Hình học mới (cẳng tay 251 mm) với đầu hút 30 mm vẫn đạt 22/22.
- Sửa: `robot.scale` 0.80 → 1.0. Robot in 1:1 từ chính file CAD, vừa động cơ NEMA17 42 mm; GUI robotarm-6-axis cũng dùng 1.0. Tọa độ và tên thùng giữ nguyên.
- Test: pytest 22/22. Đầu hút 106–156 mm vẫn với tới DONG_BANG / LOI / TRUNG_DU (dư biên nếu đo lại đầu hút).
- File: config/station.yaml
