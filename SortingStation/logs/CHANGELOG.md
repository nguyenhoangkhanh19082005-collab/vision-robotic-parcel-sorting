# CHANGELOG — SortingStation

## 2026-11-25 — developer (gửi kết quả lên web + giao diện một màn hình)
- `sorting_station/app.py`: tạo `build_reporter` và truyền vào cửa sổ, đóng khi thoát. Trước bản này
  `run_station.bat` không gửi kết quả nào lên Supabase.
- `sorting_station/main_window.py`:
  - Nhận `reporter`.
  - Twin thu gọn và thẻ robot đặt vào `operator_control_layout` (cột Chạy tự động của MachineVision).
  - Nút Dừng toàn trạm đặt trên `header_actions`.
  - Tab "Robot" và "Phân loại & tư thế".
  - Robot GUI `setStyleSheet("")` để dùng giao diện trạm.
  - Bước 4 của dải hành trình theo trạng thái bộ tuần tự.
- `sorting_station/sorting_panel.py`: chữ thường, màu theo bảng màu trạm.
- Chi tiết thiết kế và kiểm thử: MachineVision `logs/CHANGELOG.md` (2026-11-25). Test 22/22.

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

## 2026-11-22 — developer — IK khi khóa J2
- `sorting_station/robot_model.py`:
  - `lock_joint()`; `max_tool_tilt_deg` (mặc định 15°).
  - `_ik_locked()`: IK ưu tiên 2 cấp. Cấp 1 là vị trí TCP; cấp 2 giữ trục hút thẳng đứng trong không gian null.
  - Trong các nghiệm đủ thẳng, chọn nghiệm gần tư thế hiện tại nhất để không lật cổ tay.
- `main_window.py`: đọc `robot.lock_j2_deg` và `robot.max_tool_tilt_deg`.
- `sorting.py`: HOME và tư thế dạy luôn lấy J2 = góc khóa.
- `config/station.yaml`: thêm `lock_j2_deg: null` (chưa bật), `max_tool_tilt_deg: 15`.
- pytest 22/22.

## 2026-11-22 — developer — Theo tư thế 0 mới DỰNG THẲNG ĐỨNG
- `robot_model.py`: hệ base X lấy theo cẳng tay ở chữ L (J3 +90°) khi tâm cổ tay nằm trên trục J1. Vị trí băng tải và thùng giữ nguyên.
- `tests/test_robot_model.py`: kiểm tra TCP q = 0 dựng đứng (772 mm) và chữ L tại J3 +90°.
- `config/station.yaml`: cập nhật chú thích.
- `home_joints_deg [0…]` giờ là tư thế dựng thẳng đứng.
- pytest 22/22.

## 2026-11-23 — developer — Digital twin: trạm phân loại đầy đủ (gá camera/đèn/cảm biến) + bố trí 3 thùng mới
- `config/station.yaml`:
  - Thùng gom về cuối băng: LỖI ngay sau cuối băng (0.24, 0.185); ĐỒNG BẰNG bên trái băng, phía robot (0.07, 0.22); TRUNG DU-MN bên phải băng (0.37, 0.03).
  - `drop_xyz_mm` cập nhật theo vị trí thùng; IK kiểm tra đủ 3 điểm thả và điểm tiếp cận +60 mm.
  - Camera: y = -0.0275, height_above_belt = 0.137, show_mount = true (theo gá thật của nhóm).
- `sorting_station/cell_twin.py`: vẽ gá + webcam C260 + đèn ring + cảm biến M18 từ `assets/station_equipment/*.stl` (manifest.json, mm, hệ base; xuất bởi robotarm-6-axis/src/cad/tram_step_3_xuat_tram.py). Thiếu thư mục thì vẽ khối hộp như cũ. Vùng nhìn camera tính theo chiều cao (≈ 58° × 46°).
- `scripts/smoke_screenshot.py`: chờ chu trình 50 s (HOME dựng thẳng → chu trình 9 °/s dài ≈ 30–40 s). Chu trình mô phỏng DONE.
- pytest 22/22.

## 2026-11-23 — developer — Kiện nhỏ hơn, mesh cảm biến E18-D80NK, đèn Ø200
- `config/station.yaml`: parcel_size 0.11 × 0.15 × 0.06 → 0.09 × 0.12 × 0.05 m; pick_xyz_mm z 97 → 87.
- `assets/station_equipment/` xuất lại: cảm biến E18-D80NK thật + 2 đai ốc, đèn Ø200. File cũ Cam_bien_quang_M18.stl, Dai_oc_M18_*.stl không còn trong manifest (chưa xóa).
- pytest 22/22; smoke: chu trình DONE.

## 2026-11-24 — developer — Twin trạm: giác hút thật trên khâu 6 + bơm/van AirTAC
- `sorting_station/cell_twin.py`: đầu hút vẽ bằng `link_6_tool_ga.stl` + `link_6_tool_giac_hut.stl` gắn khâu 6 (màu giác đổi theo trạng thái); thiếu file thì dùng khối trụ cũ.
- `assets/station_equipment/` xuất lại, thêm 21 mesh van AirTAC 4V210-08 + bơm hút.
- pytest 22/22; smoke OK.

## 2026-11-24 — developer (tư thế 0 chữ L đầu hút chúc xuống + khóa J2)
- `sorting_station/robot_model.py`: giới hạn J3 [-225, 45], J5 [-20, 200] theo tư thế 0 mới (xem CHANGELOG robotarm-6-axis).
- `config/station.yaml`: `robot.lock_j2_deg: 0.0`; `pick_xyz_mm` 220 → 230 mm (đầu hút thẳng đứng khi J2 khóa);
  điểm thả LỖI → (198, 153, 165) mm (mép thùng gần robot). TRUNG_DU (r ≥ 290 mm) KHÔNG với tới khi J2 khóa 0° — chờ người dùng chọn chỗ đặt mới
  (đề xuất tâm (-0.10, 0.215) m, đã kiểm IK đạt).
- Vùng với khi J2 khóa 0°, đầu hút thẳng đứng: r = 251·cosθ, z = 186.5 − 251·sinθ (mm, θ = J3 = J5).
- `tests/test_robot_model.py` (test tư thế 0 mới), `tests/test_sorting.py` (tư thế dạy trong giới hạn mới). pytest 22/22 đạt.

## 2026-11-24 (lần 3) — developer — Home thật (J2 cúi trước 12.5°): điểm gắp/thả mới
- `config/station.yaml`: pick_xyz_mm [230,0,87] → [250,0,87] (kiện cao 20–150 mm đều hút được, nghiêng 0.3–12°);
  DONG_BANG [100,255,140], LOI [220,170,150], TRUNG_DU [320,30,180] (r 321 mm, nghiêng 12.8°, cách mép thùng 30 mm).
- `sorting_station/robot_model.py`: giới hạn J2 [-102.5,97.5], J3 [-220,20], J5 [-42,178].
- `tests/test_robot_model.py`: test home thật. pytest 22/22 đạt. Chi tiết: robotarm-6-axis/logs/CHANGELOG.md.

## 2026-11-24 (lần 4) — developer — Điểm thả thùng dạy tay
- Người dùng tự dạy vị trí thùng bằng nút "GHI TƯ THẾ ROBOT HIỆN TẠI" (lưu `config/taught_poses.json`, ưu tiên hơn drop_xyz_mm).
- `sorting_station/sorting.py` `plan_cycle`: nếu tư thế thùng ĐÃ DẠY không lên được +approach_height_mm (J2 khóa, rìa tầm với)
  thì thả ngay tại tư thế dạy thay vì báo lỗi hủy chu trình. pytest đạt.
