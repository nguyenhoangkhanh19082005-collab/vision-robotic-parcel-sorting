# CHANGELOG

## 2026-10-21 — Phiên chính (thiết lập)
- Tạo 3 subagent cấp project theo CLAUDE.md mục 3: researcher (haiku), developer (sonnet), tester (opus).
- File: `.claude\agents\researcher.md`, `.claude\agents\developer.md`, `.claude\agents\tester.md`

## 2026-10-21 — developer (tích hợp SortingStation)
- `main.py`, `Changes/main.py`: `WorldClassRoboDKApp.__init__(urdf_dir=DEFAULT_URDF_DIR)`; đường dẫn URDF tuyệt đối theo vị trí `main.py` (trước đây tương đối theo thư mục chạy). Hành vi khi chạy độc lập không đổi.
- Lý do: nhúng GUI robot vào app chung `E:\UDMT\SortingStation`. Đánh giá chi tiết: `E:\UDMT\SortingStation\reports\DANH_GIA_VA_DE_XUAT_2026-10-21.md`.

## 2026-10-22 — developer (sửa lỗi biên dịch firmware)
- `robotarmver2/robotarmver2.ino` (+ bản sao `Changes/robotarmver2.ino`): thêm lại `const float STEPS_PER_REV = 200.0;` bị xóa nhầm ở bản chỉnh dòng điện/gia tốc 16/10 (bản cũ `Changes/robotarmver2/robotarmver2.ino` dòng 100 vẫn có).
- Kiểm tra: `arduino-cli compile --fqbn esp32:esp32:esp32s3` (core esp32 3.3.7) — đạt, 402 286 byte flash (30%), 25 156 byte RAM (7%). Build/cache đặt tại `E:\UDMT\.tmp`. Chưa nạp xuống board.

## 2026-10-22 — developer (chẩn đoán "không điều khiển được từng khớp")
- `robotarmver2.ino` (+ `Changes/robotarmver2.ino`): thêm chẩn đoán, KHÔNG đổi logic điều khiển — khi khởi động in `[DIAG]` trạng thái UART từng TMC2209 (`test_connection`), vi bước/dòng đọc lại, cảnh báo khớp không gắn được bộ phát xung; mỗi 2 s in số gói hợp lệ / sai CRC / MOVE bị bỏ qua (MODE_AUTO) và vị trí bước. Biên dịch đạt (esp32 core 3.3.7, cấu hình board của lần nạp 22/10: CDCOnBoot=default, 4MB, PSRAM tắt).
- Thêm `tests/chan_doan_robot.py`: nghe log/telemetry qua COM (không gửi lệnh); `--jog J --deg D` thử một khớp, bắt buộc gõ YES.
- Đã xác minh: FastAccelStepper 2.0.0 hỗ trợ 8 động cơ trên ESP32-S3 (đủ 6); lần nạp 22/10 có CDC On Boot tắt → Serial ra cổng CH343 (COM8) đúng với GUI.

## 2026-10-22 — developer (dò tự động bus UART TMC2209)
- Kết quả chẩn đoán lần 2 (người dùng gửi): cả 6 driver vẫn "DOC TOAN 0"; người dùng xác nhận không đứt dây/nứt mối hàn.
- `robotarmver2.ino` (+ `Changes/robotarmver2.ino`): thêm `probeUartBus`/`setupTmcUartBus` chạy đầu `setup()` — mỗi bus thử 2 chiều chân (RX/TX gốc và đảo) × 4 địa chỉ, đọc thanh ghi IOIN (VERSION 0x21 = TMC2209, kèm mức chân EN/MS1/MS2); tự dùng chiều chân trả lời được, in hướng dẫn nếu không chiều nào trả lời. Chỉ đọc, không ghi thanh ghi; TX đặt dòng ra thấp nhất khi dò. Biên dịch đạt (404 418 byte flash).

## 2026-10-22 — developer (SỬA LỖI GỐC: Ethernet bị tắt vĩnh viễn)
- Người dùng cho biết firmware gốc (git 120afee) chạy hoàn hảo trên cùng phần cứng. So sánh: firmware gốc CHỈ điều khiển qua LAN/UDP và gọi `Ethernet.begin()` vô điều kiện; bản "Dual-Mode" (Changes, 16/10) gọi `Ethernet.hardwareStatus()` TRƯỚC `begin()` — thư viện Ethernet 2.0.2 chỉ nhận diện chip trong `W5100.init()` (bên trong begin) nên luôn trả `EthernetNoHardware` → Ethernet tắt vĩnh viễn, mọi lệnh LAN bị bỏ qua (log "Khong phat hien W5500 SPI").
- Sửa `robotarmver2.ino` (+ `Changes/robotarmver2.ino`): gọi `Ethernet.begin(mac, ip)` trước rồi mới kiểm tra `hardwareStatus()`. Biên dịch đạt. Chưa nạp.
- Ghi nhận: UART TMC2209 "DOC TOAN 0" nhiều khả năng đã có từ trước (firmware gốc cũng cấu hình qua UART; driver vẫn chạy độc lập bằng STEP/DIR/EN) → không phải nguyên nhân robot đứng yên.
- Khác biệt khác so với bản gốc cần lưu ý khi thử: tỉ số truyền J5 1.0 → 2.5; EN luôn bật (bản gốc tự bật/tắt theo chuyển động); dòng động cơ (không có tác dụng khi UART không thông).

## 2026-10-22 — tester (kết luận chẩn đoán)
- Nguyên nhân "không điều khiển được từng khớp": chưa cấp nguồn động lực cho driver. Có nguồn: 6/6 TMC2209 trả lời UART, vi bước 16, dòng đúng. Đính chính nhận định UART lỗi từ trước (sai).
- Giữ lại trong firmware: sửa `STEPS_PER_REV`, sửa thứ tự `Ethernet.begin()`, phần chẩn đoán `[DIAG]` + dò UART (chỉ in log, không đổi logic điều khiển).

## 2026-10-22 — tester (thử từng động cơ có nguồn động lực)
- Thử ±45° từng khớp: firmware phát đúng số bước cho cả 6 khớp; người dùng quan sát J2, J4, J5 chạy; **J1, J3 không quay; J6 quay liên tục** (firmware báo đã dừng, steps = 0).
- FastAccelStepper 2.0.0 (cài 16/10) gán J1–J4 vào MCPWM/PCNT, J5–J6 vào RMT; tài liệu thư viện mô tả đúng kiểu lỗi "khớp đứng im" và "chạy mãi" trên IDF5 (core esp32 3.3.7 = IDF 5.5).
- Thêm `tests/test_xung_step/test_xung_step.ino`: phát xung bằng digitalWrite (không dùng FastAccelStepper) để tách lỗi phần cứng khỏi lỗi thư viện. Biên dịch đạt. Chờ người dùng đồng ý nạp.

## 2026-10-22 — tester (thử bit-bang từng động cơ)
- Firmware thử `tests/test_xung_step/test_xung_step.ino`: J2, J3, J4, J5 quay đúng; J1, J6 "lắc vài cái rồi dừng" ở 90° trục động cơ → nâng lên 2 vòng/chiều, chậm; thêm lệnh `m J N H`.
- Hạ dòng ra chân TX UART (GPIO_DRIVE_CAP_0) trên cả 2 bus → 6/6 driver trả lời, vi bước 16. Kết quả quan sát J1/J6 lần 2: chưa có.

## 2026-11-03 — tester (GUI Station: J4 quay liên tục, khớp khác không phản hồi)
- Nguyên nhân: board vẫn chạy firmware THỬ XUNG (xác nhận qua COM8: log "[TEST] FIRMWARE THU CHAN STEP"). Firmware này coi mỗi byte nhận được là lệnh chữ; gói nhị phân 30 byte của GUI chứa byte 0x31..0x36 ('1'..'6'), nhiều nhất là 0x34 ('4') → chạy `runTest(J4)` lặp lại (2 vòng thuận, 2 vòng nghịch, dừng). Không phải lỗi phần mềm SortingStation (dùng chung `communication.py`/`main.py` với bản Changes, đã so khớp từng byte).

## 2026-11-03 — developer (thay FastAccelStepper bằng bộ phát xung timer)
- `robotarmver2/robotarmver2.ino` (+ bản sao `Changes/robotarmver2.ino`): bỏ FastAccelStepper (lỗi J1/J3 đứng im trên MCPWM/PCNT, J6 chạy mãi trên RMT). Thêm lớp `SoftStepper` cùng tên hàm (moveTo, setSpeedInHz, setAcceleration, isRunning, getCurrentPosition, forceStopAndNewPosition, setCurrentPosition): ISR hardware timer 40 kHz phát STEP bằng thanh ghi GPIO (DDA Q16), `ProfileTask` 1 kHz tính hình thang vận tốc, đổi DIR chỉ khi đã ngừng phát xung ≥ 1 ms. Phần giao thức, MotorTask, moveSynchronized, Flash giữ nguyên.
- Mô phỏng thuật toán trên PC (4 bộ vmax/gia tốc, đích xa/gần/±1 bước/đảo chiều): luôn dừng đúng đích, không chạy mãi, 0 vi phạm thời gian DIR. Biên dịch đạt (367 KB). Chưa nạp — chờ người dùng xác nhận.

## 2026-11-03 — tester (nạp firmware chính + thử từng khớp, người dùng đồng ý)
- Nạp `robotarmver2.ino` (bộ phát xung timer) lên COM8. Log khởi động: 6/6 TMC2209 OK (UART1 + UART2, addr 0/1/2), vi bước 16, dòng ≈ giá trị đặt; "Bo phat xung: timer 40000 Hz".
- Thêm `tests/thu_tung_khop.py` (thử tuần tự J1..J6: +10° rồi về). Kết quả firmware báo: 6/6 khớp tới đúng +10° (J1/J6 89 bước, J2–J4 461 bước, J5 222 bước), về đúng 0 bước, khớp khác 0.00°, rx_ok tăng đủ, crc_err = 0, không khớp nào chạy tiếp sau khi tới đích. Giờ chạy: J1 13:25:37, J2 13:25:55, J3 13:26:13, J4 13:26:32, J5 13:26:50, J6 13:27:09. Chờ người dùng xác nhận quan sát thực tế.

## 2026-11-03 — developer (J6 quay liên tục / kẹt / sai góc so với mô hình)
- Phân tích CAD `Roboter.STEP` (`research/cad_tim_truyen_dong_j6.py`): cổ tay có 2 Nema17 — một chiếc kèm cặp puly 16/40 (J5, 2.5:1), chiếc còn lại không có puly → J6 dẫn động trực tiếp 1:1, khớp với firmware. Lỗi J6 không do tỉ số truyền.
- Firmware `robotarmver2.ino` (+ `Changes/`):
  - `configureDriver(i)` tách riêng phần cấu hình TMC2209.
  - Giám sát driver xoay vòng 250 ms/driver: đọc GSTAT/DRV_STATUS; tự cấu hình lại khi driver reset/sụt VM/mất vi bước 16; in `[DRV]` khi cờ hở cuộn (ola/olb), chạm mass/nguồn, quá nhiệt thay đổi.
  - Lệnh chữ `?` (ngoài khung gói nhị phân) in trạng thái 6 driver.
  - Dòng J6 800 → 900 mA RMS theo datasheet 17HS3401-S (1.3 A/pha; đỉnh 1.27 A).
  - Biên dịch đạt (368 KB). Chưa nạp: board không cắm USB lúc sửa.
- Thêm `tests/thu_j6.py` (thử chậm 5°/0.25 s, nhanh 360°, đọc DRV_STATUS lúc đang quay; `--joint 5` để thử chéo driver) và `docs/HUONG_DAN_KIEM_TRA_J6.md` (đo cặp cuộn 2.4 Ω, thứ tự chân TMC2209, vít khóa trục D, thử chéo driver).

## 2026-11-03 — researcher/developer/tester (thay động cơ J1–J3 bằng 42STH38-1684A + hộp số 1:27)
- Phân tích Roboter.STEP bằng OCP (chỉ đọc): `research/cad_liet_ke_chi_tiet.py`, `cad_xuat_brep.py` (cache `research/cad_cache/`), `cad_util.py`, `cad_khung_dong_co.py`, `cad_kiem_tra_dong_co_moi.py`, `cad_quet_goc_khop.py`, `cad_chi_tiet.json`.
- Kết quả: J2 lắp thẳng vào `AX-1_6` (Ø37 + 4×Ø3.5 trên Ø28); J3 va `AX-2_2` (đuôi dài hơn 12.3 mm); J1 cần lỗ M2.5 trên `AX-1_11` và thò 37.1 mm dưới đáy đế. Quét góc J2/J3 −180..180°: dải không va chạm giống PG5.
- Phát hiện: CAD có đai ở J1 (≈3:1), J2 (≈4:1), J3 (≈3:1) nhưng firmware đặt 1 / 5.18 / 5.18 (không tính đai) — cần đo kiểm.
- Tạo `src/cad/tao_chi_tiet_dong_co_1_27.py` → `assets/CAD_dong_co_1_27/`: `AX-2_2_sua_J3`, `AX-1_11_sua_J1_M2.5`, `De_nang_J1` (43 mm, thành 6 mm, 4 tai M4), mô hình động cơ mới, lắp ráp `Roboter_dong_co_1_27.step`; kiểm tra va chạm sau sửa: 3/3 động cơ không va chạm. Ảnh kiểm tra `reports/cad_dong_co_1_27/`. Hướng dẫn `docs/THAY_DONG_CO_J1_J3_HOP_SO_1_27.md`.

## 2026-11-04 — developer (GUI "Not Responding" khi kéo slider)
- Nguyên nhân: người dùng kết nối COM12 = cổng USB gốc ESP32-S3 (VID:PID 303A:1001). Firmware xuất Serial qua UART0/CH343 (cổng COM khác) nên cổng USB gốc không đọc dữ liệu → `ser.write()` (không có write_timeout) chặn luồng GUI vô thời hạn; telemetry Act đứng 0.0°.
- `communication.py` (+ `Changes/`): `write_timeout=0.05`; bắt `SerialTimeoutException` (bỏ gói, cảnh báo ≤ 1 lần/3 s, 5 lần liên tiếp → tự ngắt); nhận diện cổng USB gốc ESP32-S3 (VID 0x303A) và trả thông báo cảnh báo khi kết nối; ghi `last_rx_time`/`connect_time`.
- `main.py` (+ `Changes/`): log mức WARNING khi kết nối cổng USB gốc; cảnh báo 1 lần nếu sau 2 s chưa có telemetry.
- Thử với cổng giả không đọc: 5 gói bị bỏ trong 0.25 s rồi tự ngắt, GUI không treo. pytest SortingStation 22/22 đạt.

## 2026-11-04 — developer/tester (động học toàn tay, chiều J3, J1/J5 không chạy, J6 quay vô hạn)
- Board (COM8) đang chạy firmware 03/11 có giám sát driver; lúc kiểm tra cả 6 TMC2209 "KHONG TRA LOI UART" → nguồn động lực (tổ ong) đang tắt → chưa thử động cơ được.
- GUI độc lập `main.py` (+ `Changes/`): thay động học DH (TCP q=0 ở Z = -181 mm, lệch twin) bằng PoE `kinematics_poe.py` (bản sao `SortingStation/sorting_station/robot_model.py`, + `Changes/kinematics_poe.py`), đầu hút 30 mm. FK q=0 = (296, 2, 433) mm khớp twin; IK vòng kín sai số 0.2 mm; GUI khởi động OK.
- Firmware `robotarmver2.ino` (+ `Changes/`): `DIR_SIGNS[2] = -1` (J3 thật quay ngược twin theo người dùng). Biên dịch đạt; CHƯA nạp.
- Chiều dương từng khớp trên twin (tính từ PoE, ảnh `reports/dong_hoc/twin_J{2,3,5}_duong_30.png`): J1 ngược kim đồng hồ nhìn từ trên; J2 cúi ra trước; J3 cẳng tay gập xuống; J4 ngược kim đồng hồ nhìn dọc cẳng tay ra đầu hút; J5 ngóc lên; J6 theo kim đồng hồ.
- Thêm `tests/hieu_chinh_khop.py`: người dùng chạy, trả lời quay/chiều/góc thật từng khớp → đề xuất DIR_SIGNS, GEAR_RATIOS (tỉ số đai CAD chưa tính trong firmware), thử J6 tự quay khi không có xung; lưu JSON vào reports/.

## 2026-11-04 — tester (nạp firmware J3 đảo chiều — người dùng đồng ý, đã bật nguồn động lực)
- Nạp `robotarmver2.ino` (DIR_SIGNS J3 = -1) lên COM8. Khởi động: 6/6 TMC2209 OK, vi bước 16, dòng ≈ đặt (J6 887 mA).
- DRV_STATUS lúc đứng yên: J2, J3, J4, J6 OK; **J1 HO_CUON_B** (hở cuộn B); **J5 HO_CUON_A + HO_CUON_B** (hở cả 2 cuộn) — trùng đúng 2 khớp người dùng báo không quay → nghi dây/jack động cơ J1, J5 hở. (Cờ ola/olb lúc đứng yên chỉ mang tính gợi ý theo datasheet, cần đo điện trở cuộn để xác nhận.)

## 2026-11-04 — tester (kết quả hiệu chỉnh của người dùng: reports/hieu_chinh_khop_20261104_181319.json, _182154.json)
- Lần 2 (đủ J1–J6): J1, J2, J3, J4 quay, ĐÚNG chiều twin (J3 sau khi đảo dấu đúng), về đúng chỗ. J5 không quay (khớp với cờ hở 2 cuộn). Chưa nhập góc thật → chưa hiệu chỉnh được tỉ số truyền.
- J3 lần 1 không quay, lần 2 quay → nghi tiếp xúc chập chờn (jack/dây J3).
- J6: cả 2 lần đều "tự quay sau khi dừng", "không về chỗ cũ", "tự quay khi chỉ J2 chạy"; chiều so với twin lần 1 = cùng, lần 2 = ngược → J6 nhận XUNG NHIỄU, không theo xung firmware → lỗi phần cứng đường STEP J6 (GPIO38→driver), không đổi DIR_SIGNS[5] (đề xuất đảo dấu J6 của script là sai do nhiễu).
- Không sửa firmware ở bước này.

## 2026-11-06 — developer (người dùng đã lắp 42STH38-1684A + hộp số 1:27 vào J1–J3, in lại chi tiết theo assets/CAD_dong_co_1_27)
- Firmware `robotarmver2.ino` (+ `Changes/`), biên dịch đạt, CHƯA nạp:
  - `GEAR_RATIOS` = {81, 108, 81, 5.18, 2.5, 1} (27 × đai 3.0/4.0/3.0 theo CAD — cần đếm răng/hiệu chỉnh xác nhận).
  - Vi bước theo khớp `MICROSTEPS_J` = {8, 8, 8, 16, 16, 16}; dòng J1–J3 1100 mA RMS; tốc độ J1–J3 4800 bước/s (3 vòng/s động cơ → J1/J3 ≈ 13°/s, J2 ≈ 10°/s), gia tốc 8000 bước/s².
  - `moveSynchronized`: đồng bộ theo thời gian (T = khớp chậm nhất) thay cho tỉ lệ số bước.
- Động học: hình học khớp không đổi (động cơ lắp đúng vị trí cũ) → FK/IK giữ nguyên; đế nâng 43 mm → `SortingStation/config/station.yaml`: băng tải & thùng z −0.043 m, pick z 97 mm, drop z 127 mm, joint_speed 9°/s, settle_timeout 8 s. IK điểm gắp/thả đều giải được. pytest 22/22.

## 2026-11-11 — developer (firmware thử riêng J1 hộp số 1:27)
- Thêm `tests/thu_J1_hop_so_1_27/thu_J1_hop_so_1_27.ino`: chỉ điều khiển J1 (STEP 1, DIR 2, EN 14, TMC2209 UART1 addr0, TX yếu CAP_0), 8 vi bước, 1100 mA, tỉ số 81 (27 × đai 3, đổi được bằng lệnh `k`), timer 40 kHz + hình thang 1 kHz. Lệnh chữ qua Serial Monitor (g/r/v/a/k/i/d/z/s/x/e/?) và nhận gói nhị phân 30 byte của GUI (chỉ góc J1). Biên dịch đạt (348 KB). Chưa nạp (board chưa cắm).

## 2026-11-13 — developer (thêm mặt bích 5 mm vào tâm puly lớn J2, J3 theo yêu cầu)
- Mặt bích người dùng gửi (`assets/CAD_mat_bich/mat_bich_5mm_goc.step`): vành Ø22 × 2.1, 4 lỗ M3 trên vòng Ø16, moay-ơ Ø10 cao 11.95, lỗ Ø5, 2 vít trí M3.
- `src/cad/them_mat_bich_J2_J3.py`: đặt mặt bích đồng tâm trên MẶT NGOÀI puly 55 mm (J2) và 40 mm (J3) (phía khâu AX-2_5/AX-3_2 chỉ dày 8–11 mm, không đủ cho moay-ơ 12 mm); khoan 4 lỗ mồi Ø2.5 sâu 8 mm trên vòng Ø16 ở 2 puly; khoét nắp `AX-2_3` (chạm 147 mm3 ở J3). Kiểm tra va chạm sau sửa: không. Xuất STEP/STL + `Mat_bich_va_puly_J2_J3.step`; ảnh `reports/cad_mat_bich/`.

## 2026-11-13 — developer — Cập nhật mặt bích vào file lắp ráp Roboter_dong_co_1_27.step
- Thêm 2 mặt bích 5 mm (J2, J3) vào file lắp ráp đã có 3 động cơ mới; thay puly 55/40 mm bằng bản khoan lỗ mồi, nắp AX-2_3 bằng bản đã khoét. Kiểm tra va chạm mặt bích với toàn bộ 198 chi tiết (kể cả động cơ mới): không va chạm (chỉ kiểm tra trên CAD).
- Bản cũ giữ lại: assets/CAD_dong_co_1_27/Roboter_dong_co_1_27_truoc_mat_bich.step
- File: src/cad/cap_nhat_roboter_mat_bich.py (mới), src/cad/them_mat_bich_J2_J3.py (tách hàm build()), assets/CAD_dong_co_1_27/Roboter_dong_co_1_27.step (ghi đè), assets/CAD_dong_co_1_27/ket_qua_cap_nhat_mat_bich.txt (mới)

## 2026-11-14 — developer — Firmware thử riêng J5 và J6 (động cơ cũ)
- Firmware độc lập điều khiển J5 (STEP40/DIR41, TMC addr 1, ×2.5) và J6 (STEP38/DIR39, TMC addr 2, ×1) qua Serial Monitor; lệnh theo khớp, đọc trạng thái J4/J5/J6. Biên dịch OK (348 KB), CHƯA nạp.
- File: tests/thu_J5_J6_dong_co_cu/thu_J5_J6_dong_co_cu.ino

## 2026-11-14 — developer — Firmware thử 1 động cơ ở ổ J5 (thay bản J5+J6)
- Chỉ điều khiển ổ J5 (STEP40/DIR41), tự dò địa chỉ TMC2209 0..3, lệnh không cần số khớp, thêm lệnh p (xung thô), u (dò lại driver), cảnh báo rõ khi không thấy driver. Biên dịch OK, CHƯA nạp.
- File: tests/thu_1_dong_co_o_J5/thu_1_dong_co_o_J5.ino

## 2026-11-14 — developer — Firmware thử 1 động cơ ở ổ J1
- Bản sao của thu_1_dong_co_o_J5 đổi sang ổ J1: STEP GPIO1, DIR GPIO2, UART1 (RX17/TX18), tự dò địa chỉ TMC 0..3. Biên dịch OK, CHƯA nạp.
- File: tests/thu_1_dong_co_o_J1/thu_1_dong_co_o_J1.ino

## 2026-11-14 — developer — Tăng mô-men 6 khớp (nguồn 24 V, TMC2209)
- configureDriver(): tắt GCONF.i_scale_analog (trước đó dòng bị nhân VREF/2.5 V theo biến trở trên module), đặt tỉ lệ dòng giữ theo từng khớp HOLD_MULT = {0.5, 0.7, 0.7, 0.7, 0.8, 0.5} (thư viện mặc định 0.5), iholddelay 4, TPOWERDOWN 40.
- MOTOR_CURRENTS {1100,1100,1100,500,800,900} → {1500,1500,1500,400,900,900} mA RMS theo datasheet (SY42STH38-1684A 1.68 A; 17HS13-0404S-PG5 0.4 A; 17PM-K4xx ≥ 0.9 A; 17HS3401 1.3 A).
- Dòng [DIAG] lúc khởi động in thêm dòng giữ và trạng thái VREF. Biên dịch OK (368 KB), CHƯA nạp, chưa thử trên robot.
- File: robotarmver2/robotarmver2.ino, Changes/robotarmver2.ino

## 2026-11-14 — tester — Đối chiếu firmware tăng mô-men với Roboter_dong_co_1_27.step
- Đọc 198 chi tiết trong STEP (tên, thể tích, trọng tâm, trục khớp), tính mô-men trọng lực xấu nhất J2–J5 và so với mô-men động cơ theo dòng điện firmware 14/11. Kết quả: động cơ đủ (hệ số an toàn J2 2.2–2.8, J3 3.3–4.4, J4 ≈ 10, J5 3–6); điểm yếu là đai (lực căng J2 ≈ 254 N, J3 ≈ 166 N). Phát hiện: J5 dùng cùng cặp puly 16 mm/40 mm như J1, J3 nhưng firmware đặt 2.5 (đo) so với 3.0 (CAD) → tỉ số đai J1–J3 cần đếm răng. Không sửa firmware.
- File: research/cad_doc_step_lap_rap.py, research/cad_kiem_tra_mo_men.py, reports/kiem_tra_mo_men_theo_CAD.txt

## 2026-11-14 — developer — GUI main.py: Task 1–3 chờ robot tới vị trí thay vì chờ thời gian cố định
- Rà soát GUI với firmware 14/11: giao thức (gói 30 byte, CRC, mã lệnh, baud 115200) không đổi; GUI chỉ gửi góc (độ) nên tỉ số truyền/vi bước/dòng điện không ảnh hưởng. Chỗ phải sửa: Task 1/2/3 dùng sleep 0.05–0.5 s, không đủ khi J1–J3 chỉ còn ≈ 10–13°/s → thêm _wait_robot_reached() (so telemetry Act với slider, sai số 1.5°, quá hạn 40 s, nút DỪNG hủy chờ).
- File: main.py, Changes/main.py. Chỉ kiểm tra cú pháp (py_compile), chưa chạy với robot.

## 2026-11-15 — developer/tester — Tính lại động học từ file STEP, tư thế 0 chữ L, hiệu chỉnh khớp không cần nạp lại
- Phát hiện (đối chiếu mesh twin với Roboter_dong_co_1_27.step): (1) mesh từng khâu của twin trùng chi tiết STEP nhưng bị tịnh tiến lệch nhau 5–14 mm (bản xuất URDF chưa ghép khít) → JOINT_POINTS cũ lệch trục khớp thật 3–13 mm, cẳng tay 252.8 mm thay vì 267.0 mm; (2) tư thế 0 cũ là tư thế vẽ CAD: cánh tay trên nghiêng 5.89°, J3 lệch 11.61°, J4 xoay 13.24°, cổ tay gập 15.51° so với chữ L.
- Mới: src/cad/tinh_dong_hoc_tu_step.py sinh dong_hoc_step.py (JOINT_POINTS/JOINT_AXES ở tư thế 0 = chữ L chuẩn, LINK_MESH_TRANSFORMS đặt lại mesh). robot_simulation.py, kinematics_poe.py và SortingStation/sorting_station/robot_model.py dùng hình học này. Kết quả: J2–J3 154.50 mm, J3–tâm cổ tay 267.00 mm, TCP ở q=0 = (338.5, 0, 354.0) mm (đầu hút 30 mm); IK vòng kín 200 điểm sai ≤ 0.5 mm; pytest SortingStation 22/22 (sửa ngưỡng test tư thế 0: 0.3 → 0.25 m).
- Firmware robotarmver2.ino: DIR_SIGNS J3 −1 → +1 (người dùng báo ngược sau khi thay động cơ); thêm gear_ratio[]/dir_sign[] chỉnh khi đang chạy bằng lệnh $P, $K, $D, $S, $R và lưu Flash NVS. Biên dịch OK (388 KB), CHƯA nạp.
- tests/hieu_chinh_khop.py: đọc giá trị từ board ($P), gửi ngay chiều/tỉ số mới, cho thử lại, lưu Flash. tests/chan_doan_robot.py: Reader lưu các dòng chữ.
- main.py: MoveL/Task gửi góc IK chính xác (không còn int() cắt phần lẻ), HUD hiển thị 0.1°.
- File: dong_hoc_step.py (mới), src/cad/tinh_dong_hoc_tu_step.py (mới), robot_simulation.py, kinematics_poe.py, Changes/kinematics_poe.py, main.py, Changes/main.py, robotarmver2/robotarmver2.ino, Changes/robotarmver2.ino, tests/hieu_chinh_khop.py, tests/chan_doan_robot.py, reports/dong_hoc/dong_hoc_tu_step.txt, reports/dong_hoc/twin_moi_*.png; SortingStation: sorting_station/robot_model.py, tests/test_robot_model.py
- Chưa kiểm trên robot thật. Còn chờ người dùng: chạy hieu_chinh_khop.py để chốt chiều quay và tỉ số đai J1–J3.

## 2026-11-15 — developer — Gá in 3D gắn giác hút chân không L60-M12 + HSP-40-2 vào mặt bích J6
- Khung cổng 46 × 45 × 45 mm (đế 4, khoang ống khí 36 × 36 hở 2 phía, tấm trên 5 có lỗ Ø12.4 cho ren M12×1 và 4 rãnh chữ U thả vít), bắt 4 vít M3×12 vào đai ốc chìm của AX-6 (vòng Ø41.54, góc 60/120/240/300°). In nằm nghiêng, không cần support. Miệng giác cách mặt bích ≈ 99 mm (chưa nén).
- Kiểm tra CAD: không va chạm khi J6 quay đủ vòng; khi quét J5 chỉ chạm cẳng tay (AX-4_1, AX-4_2, động cơ J5) ở 100–110° so với tư thế CAD. Kích thước giác hút lấy từ hình người bán, chưa đo hàng thật; chưa in thử.
- File: src/cad/tao_ga_giac_hut_J6.py, assets/CAD_giac_hut/{Ga_giac_hut_J6.step, Ga_giac_hut_J6_huong_in.stl, Lap_rap_ga_giac_hut_J6.step, ket_qua_kiem_tra.txt}, reports/cad_giac_hut/*.png

## 2026-11-15 — developer — Firmware: J2, J3 lên dòng tối đa của TMC2209
- Người dùng báo lắp động cơ vào tay thì J2/J3 vẫn không đủ mô-men nâng. MOTOR_CURRENTS J2, J3: 1500 → 1770 mA RMS (CS=31, nấc cao nhất với Rsense 0.11 Ω; 105 % định mức động cơ, mô-men ≈ +18 %). HOLD_MULT J2, J3: 0.7 → 0.8 (giữ 1.42 A). JOINT_MAX_SPEED J2, J3: 4800 → 3200 bước/s; JOINT_MAX_ACCEL: 8000 → 4000.
- Biên dịch OK, CHƯA nạp, chưa thử trên robot. Cần tản nhiệt + quạt cho driver J2, J3.
- File: robotarmver2/robotarmver2.ino, Changes/robotarmver2.ino

## 2026-11-16 — developer — Rút ngắn cẳng tay 31,5 mm (trục J3 → tâm cổ tay 267,0 → 235,5 mm), giữ nguyên linh kiện cơ khí
- 3 chi tiết in lại: AX-3_2 (bỏ 6 mm cổ hộp + vát gờ ổ bi 1 mm), AX-3_1 (bỏ 9,5 mm ống, nửa xa xoay 45° để động cơ J4 nằm thẳng và lùi vào hốc trục J3), AX-4_4 (đoạn bầu rỗng 32,5 → 16,5 mm, thành ≥ 2,5 mm). Cánh tay trên J2–J3 KHÔNG đổi (bị khóa bởi chiều dài đai J3).
- Đã kiểm: 3 lưới STL kín, 1 khối; khe hở động cơ J4 sau khi xoay/lùi: thành hộp 1,0 mm, ổ bi và ngõng trục 2,0 mm. CHƯA chạy src/cad/rut_ngan_kiem_tra.py (chồng lấn toàn bộ, quét khớp J3/J4, mô-men trước/sau, ảnh so sánh). Chưa in thử.
- File: src/cad/rut_ngan_lib.py, rut_ngan_xuat_brep.py, rut_ngan_cang_tay.py, rut_ngan_kiem_tra.py (mới); assets/CAD_rut_ngan/{AX-3_1_rut_ngan, AX-3_2_rut_ngan, AX-4_4_rut_ngan}.{step,stl}, Roboter_rut_ngan.step, ket_qua_dung_hinh.txt; reports/rut_ngan/*.png (ảnh hiện trạng)

## 2026-11-16 — tester — Kiểm tra bản rút ngắn cẳng tay (src/cad/rut_ngan_kiem_tra.py)
- Chồng lấn: 163 cặp chi tiết có vị trí tương đối thay đổi, 0 cặp tăng (chồng lấn cũ động cơ J4 × hộp trục J3 65 mm3 → 0). Khe hở động cơ J4: thành hộp 1,00 mm, ổ bi/ngõng trục 2,00 mm, ống 2,36 mm. Quét J3 bước 10°: khoảng quay tự do không đổi so với bản gốc. Mô-men trọng lực (nhựa 60 %, tải 200 g): J3 3,16 → 2,73 N·m (−13,5 %), J2 6,49 → 6,05 N·m (−6,8 %).
- Sửa script kiểm tra: siết dung sai chi tiết gốc trước khi tính giao; AX-4_4 kiểm theo 3 phần (phép giao cả khối với AX-4_1 bị treo); quét J3 dùng lưới giảm mật độ, bỏ cụm quanh trục. Chưa in thử, chưa cập nhật động học twin.
- File: src/cad/rut_ngan_kiem_tra.py, src/cad/rut_ngan_cang_tay.py, src/cad/rut_ngan_lib.py, assets/CAD_rut_ngan/ket_qua_kiem_tra.txt, reports/rut_ngan/so_sanh_*.png, ba_chi_tiet_in_lai.png

## 2026-11-16 — developer + tester: phương án thay động cơ J4 bằng MinebeaMitsumi 17PM-K4xx (lắp trực tiếp) + gá giác hút J6 trong file lắp ráp
- Nội dung: dựng mô hình động cơ 17PM-K4xx (42 × 42 × 48, trục Ø5 × 24, 350 g) thay 17HS13-0404S-PG5 trên bản cẳng tay rút ngắn;
  AX-3_1 rút ngắn KHÔNG xoay 45° + rãnh thoát góc động cơ; tấm gá AX-3_3 thêm 4 lỗ M3 31 × 31 có hạ bậc; khớp nối AX-4_6 lỗ Ø5,1;
  thêm gá + giác hút (mô phỏng) vào mặt bích J6. Kiểm tra chồng lấn/khe hở (0 chỗ mới), tính mô-men J2/J3/J4.
- Kết quả tính (nhựa 60 %, tải 200 g ở miệng giác): J2 7,38 → 6,62 N·m, J3 3,73 → 3,21 N·m so với bản dài gốc; riêng việc đổi động cơ
  chỉ giảm J2 0,27 N·m, J3 0,05 N·m. J4 mất hộp số 5,18:1: mô-men chạy 0,82 → ≈ 0,39 N·m, không đủ khi J5 gập 90° ngang với tải ≥ 200 g.
- Chưa in, chưa lắp, chưa sửa firmware (cần GEAR_RATIOS[3] = 1.0 và dòng J4 khi lắp động cơ mới).
- File: src/cad/thay_dong_co_J4.py (mới); assets/CAD_dong_co_J4_17PM/ (AX-3_1_rut_ngan_khong_xoay, AX-3_3_17PM, AX-4_6_truc_5mm .step/.stl,
  Dong_co_17PM-K4xx.step, Roboter_rut_ngan_17PM_giac_hut.step, ket_qua_kiem_tra.txt); reports/dong_co_J4_17PM/*.png

## 2026-11-16 — developer: tăng tốc độ / gia tốc J2, J3 (firmware, CHƯA nạp)
- J2: 6,7 °/s → 30 °/s, 8,3 °/s² → 60 °/s²; J3: 8,9 °/s → 40 °/s, 11,1 °/s² → 60 °/s² (cả hai = 14400 xung/s = 540 v/ph trục động cơ).
- Giới hạn J2/J3 khai báo theo độ (JOINT_SPEED_DEG / JOINT_ACCEL_DEG), đổi ra xung bằng updateJointLimits() lúc khởi động và sau lệnh $K,
  kẹp ở trần bộ phát xung 18000 xung/s; in "[DIAG] Jx gioi han" khi khởi động. J1, J4–J6 không đổi.
- Lý do dừng ở mức này: 42STH38 ở 24 V / 1,77 A bắt đầu tụt mô-men từ ≈ 480 v/ph (≈ 85 % ở 540, ≈ 70 % ở 630 v/ph, ước tính lý tưởng).
- Biên dịch OK (388752 byte). File: robotarmver2/robotarmver2.ino, Changes/robotarmver2.ino

## 2026-11-17 — developer — Firmware J4 cho động cơ 17PM-K4xx nối thẳng
- Người dùng báo J4 (17PM mới) rung lắc tại chỗ; đổi chéo 2 dây giữa thì đứng im (hở mạch → cách đấu ban đầu đúng cặp). Nguyên nhân: firmware còn thông số động cơ cũ.
- `GEAR_RATIOS[3]` 5.181818 → 1.0; `MOTOR_CURRENTS[3]` 400 → 900 mA; `HOLD_MULT[3]` 0.7 → 0.9; J4 đặt theo độ 45 °/s, 90 °/s² (`JOINT_SPEED_DEG/JOINT_ACCEL_DEG[3]`).
- `loadFromFlash()`: tỉ số J4 > 2.0 lưu trong NVS (của hộp số cũ) bị bỏ qua, dùng mặc định 1.0.
- Biên dịch esp32:esp32:esp32s3 đạt (388 816 byte). CHƯA nạp.
- File: robotarmver2/robotarmver2.ino, Changes/robotarmver2.ino

## 2026-11-18 — developer — Khối trụ căng đai J2 (AX-1_3) và J3 (AX-2_1)
- Người dùng đang chèn mẩu đũa (J2 31–33 mm, J3 35–37 mm) ép vào lưng đai để căng. Thay bằng khối trụ rỗng Ø11 mọc từ thành vỏ về phía nhánh đai, lỗ xuyên Ø4.4 (que Ø4 / vít M4 luồn từ ngoài), hốc đai ốc M4 (7.3 × sâu 3.5) ở mặt ngoài, đầu khối trụ cách lưng đai 3 mm, chân đỡ đặc xuống đáy (in không cần support). Mỗi khớp 2 khối trụ (2 nhánh đai).
- J2: f = 0.28 dọc nhánh (đoạn thành thẳng, dưới 2 trụ đai ốc M3 nắp), que nghiêng ±15° so với pháp tuyến đai (vuông góc thành); chạm lưng đai ở 21.9 / 22.1 mm tính từ mặt ngoài vỏ. Giữa nhánh J2 không đặt được: vướng 2 lỗ vít nắp M3 (37,32)/(−32,37), 4 lỗ bắt tấm động cơ ở đáy, thành cong/đôi.
- J3: giữa nhánh (f = 0.5), que vuông góc đai, trục que w = 5.0 (mép vỏ w = 10); chạm lưng đai ở 29.3 / 29.1 mm từ mặt ngoài.
- Kiểm tra: phần thêm không chạm chi tiết nào trong lắp ráp, không lòi ra ngoài vỏ, cách lỗ vít ≥ 1.6 mm (J2) / 5.8 mm (J3); que ép sâu 10 mm không chạm puly/ổ bi; STL kín (0 cạnh hở), B-rep hợp lệ.
- File: src/cad/them_cang_dai_J2_J3.py (mới); assets/CAD_cang_dai_J2_J3/ (AX-1_3_cang_dai_J2.step/.stl, AX-2_1_cang_dai_J3.step/.stl, Roboter_rut_ngan_17PM_giac_hut_cang_dai.step, mat_cat_cang_dai.png, anh_3d_J2.png, anh_3d_J3.png, ket_qua_kiem_tra.txt); assets/Gui_xuong_in/C_cang_dai_J2_J3/ (2 STL).

## 2026-11-19 — developer — Firmware: J4 trở lại động cơ cũ 17HS13-0404S-PG5, tăng tốc J4
- Người dùng bỏ phương án 17PM (đã sửa được J2/J3 bằng căng đai), lắp lại động cơ J4 cũ; J4 quay "chậm, lờ đờ" vì firmware còn tỉ số 1.0 / 900 mA của 17PM.
- `GEAR_RATIOS[3]` 1.0 → 5.181818; `MOTOR_CURRENTS[3]` 900 → 400 mA (100 % định mức 0.4 A); `HOLD_MULT[3]` 0.9 → 0.7.
- Tăng tốc J4: 43 °/s (2000 xung/s cũ) → `JOINT_SPEED_DEG[3]` 90 °/s (4146 xung/s, 78 v/ph động cơ), gia tốc 17 → 180 °/s².
- `loadFromFlash()`: tỉ số J4 < 2.0 lưu trong NVS (từ $K 4 1.0) bị bỏ qua, dùng 5.18.
- Biên dịch esp32s3 đạt (388 828 byte). CHƯA nạp.
- File: robotarmver2/robotarmver2.ino, Changes/robotarmver2.ino

## 2026-11-19 — developer — Guốc trượt căng đai chụp đầu vít M4 (J2, J3)
- Người dùng báo: vít M4 tì thẳng vào lưng đai GT2 6 mm không giữ được — đai xê dịch ngang khi khớp quay và tuột khỏi đầu vít.
- Thêm guốc nhựa 18 × 12 × 9 mm: rãnh 7 mm (đai 6 + 1), 2 gờ chặn cao 2 mm (19/11 nâng lên 3.5 mm), đáy rãnh cong R20 dọc đai (hai đầu thấp 2.1 mm để dẫn đai), lỗ Ø4.5 sâu 5.5 ở mặt dưới chụp đầu vít, tự xoay để đai kéo thẳng hàng. Bề rộng đai là tham số dòng lệnh.
- STL kín (0 cạnh hở), B-rep hợp lệ.
- File: src/cad/tao_guoc_truot_dai.py (mới); assets/Guoc_truot_dai/ (Guoc_truot_dai_6mm_M4.step/.stl, anh_guoc_truot.png); assets/Gui_xuong_in/D_guoc_truot_dai/Guoc_truot_dai_6mm_M4.stl.

## 2026-11-20 — developer — Rãnh trượt căng đai lệch tâm (J2, J3)
- Đai thật cao hơn lỗ vít căng 0.5–1 con ốc M4 (puly nhỏ nâng bằng mặt "con thoi" AX-1_6). Thêm con trượt chụp đầu vít M4 có rãnh 7 mm, gờ chặn 1.6 × 3.0 mm, đáy rãnh cong R20, tâm rãnh LỆCH LÊN 2.0 / 3.0 mm so với trục vít → dùng lại lỗ vít trên vỏ đang có, không in lại vỏ.
- File: src/cad/tao_ranh_truot_cang_dai.py; assets/Ranh_truot_cang_dai/ (2 STEP + 2 STL + ảnh); assets/Gui_xuong_in/F_ranh_truot_cang_dai/ (2 STL).
- Song song đã dựng (chưa kiểm tra xong, người dùng yêu cầu dừng): src/cad/them_cang_dai_J2_J3_v2.py, src/cad/cap_nhat_mat_bich_v2.py — mặt bích 5 mm chuyển sang phía khâu (AX-2_5_mat_bich_J2, AX-3_2_mat_bich_J3: đế Ø23 + 4 lỗ mồi Ø2.5, bu lông M5×35), puly nhỏ +3 mm; đầu ra assets/CAD_v2_dai_that_mat_bich/, assets/Gui_xuong_in/E_cang_dai_v2_mat_bich/.

## 2026-11-20 — tester — Rãnh trượt J2 va vành puly lớn: phân tích + phương án (chưa sửa CAD)
- Điểm tì J2 chỉ cách điểm đai rời puly lớn 19.6 mm; puly 55 mm có 2 vành r 27.5 (khe 8 mm). Rãnh trượt rộng 10.2–11.6 mm, nhô 8 mm về phía puly → 2 gờ đâm vào vành. Chỗ trống phía puly: 3.4 mm khi chưa ép, ≈ 0 khi ép ≥ 5 mm.
- Đề xuất 4 phương án, chờ người dùng chọn. Ảnh: assets/Ranh_truot_cang_dai/phuong_an_J2_va_cham_puly.png

## 2026-11-20 — tester — Kiểm tra PA4 (in lại vỏ J2, dời trụ căng) — chưa sửa CAD
- Quét 2 nhánh đai J2 (f 0.36–0.66, vít nghiêng ±30°) trên vỏ gốc AX-1_3 bản lắp 1:27: rãnh trượt (ép 0–10 mm) phải cách vành puly 55 ≥ 1 mm; vít M4 phải luồn sạch (chỉ qua 1 lớp thành) giữa trụ vít nắp (Ø12, có hốc đai ốc) và trụ vít ghép vỏ.
- Kết quả: f ≤ 0.46 rãnh trượt vẫn chạm vành; f ≈ 0.52 (cách điểm rời puly 36.7 mm), vít chếch ≈ 5–6° luồn được giữa 2 trụ, rãnh trượt cách vành ≥ 5 mm khi ép 10 mm; thành trong cách lưng đai ≈ 25 mm, đáy vỏ dưới trục ≈ 18 mm.
- Ảnh: assets/Ranh_truot_cang_dai/PA4_vi_tri_tru_cang_moi_J2.png. Script quét: E:\UDMT\.tmp\dai\j2_pa4_quet2.py, j2_pa4_matran.py, j2_pa4_chitiet.py.

## 2026-11-20 — researcher/tester — Con lăn ổ bi căng đai J2: nghiên cứu + kiểm tra vị trí (chưa sửa CAD)
- Quy tắc: con lăn tì lưng đai ≥ puly nhỏ nhất (≈ Ø12.7) → F624ZZ (Ø13) tối thiểu; J2 đảo chiều → 2 con lăn đối xứng.
- Không đặt sát puly lớn như ảnh tham khảo được (vành Ø55 cùng mặt phẳng với vành ổ bi). 2×F624ZZ đạt từ f = 0.44; đề xuất f = 0.50, trụ đỡ trên đáy vỏ, rãnh trục 8 mm (ép 4–12 mm).
- File: research/con_lan_o_bi_cang_dai_J2.md; assets/Ranh_truot_cang_dai/con_lan_o_bi_J2_de_xuat.png.

## 2026-11-20 — tester — Kiểm tra 3D khoảng trống 2 trụ con lăn J2
- Thêm ổ bi chính J2 (Lager 65x50x7), nắp AX-1_4, khâu 2 AX-2_5, con thoi vào kiểm tra. f = 0.46 trụ chạm ổ bi chính; f = 0.50–0.52 đạt (hở nhỏ nhất 1.9–3.1 mm tới ổ bi chính, ≥ 3.8 mm tới thành, ≥ 4.1 mm tới nắp). Chọn f = 0.52.
- File: research/con_lan_o_bi_cang_dai_J2.md (bổ sung bảng), assets/Ranh_truot_cang_dai/con_lan_J2_khong_gian_3D.png.

## 2026-11-20 — developer — Vỏ J2 in lại: trụ căng đai ra giữa nhánh cho rãnh trượt chữ U (PA4)
- Người dùng chọn rãnh trượt chữ U như J3 (bỏ phương án con lăn). Vỏ nền AX-1_3 gốc (không có 2 trụ căng cũ).
- 2 trụ Ø11 mới ở f = 0.54 (cách điểm đai rời puly lớn 37.8 mm), trục vít w = −4 (cao độ lỗ cũ), vít chếch +8.5° / −9° luồn giữa trụ vít nắp và trụ ghép vỏ (nhựa quanh lỗ ≥ 2.4 mm). Lỗ Ø4.4 xuyên thành; khe thả đai ốc M4 từ miệng vỏ (cách lưng đai 12–15.5 mm); chân đỡ xuống đáy; gân vuông 7.2 mm nối trụ vào thành.
- Kiểm tra: 1 solid hợp lệ, 98.83 → 102.33 cm3, STL kín; phần thêm không chạm chi tiết nào; không che lối nhét đai ốc M3 của trụ vít nắp; rãnh trượt e = 2/3 mm ép 0/5/10 mm không chạm, cách vành puly lớn 7.1–10.9 mm; thân vít không chạm. Vít M4×40.
- Lỗi OCC đã xử lý: chân đỡ rộng đúng Ø trụ (mặt tiếp tuyến) → thu còn 9.6 mm; mặt cuối trụ trong vùng thành xiên → dò lùi; đầu gân ngay sau mặt trong thành → lưới hở → gân dài tới mặt trong + 1.4 mm (nhô ≤ 0.2 mm ở 1 góc).
- File: src/cad/tao_vo_J2_cang_dai_giua_nhanh.py; assets/CAD_vo_J2_cang_dai_giua_nhanh/ (STEP, STL, ket_qua_kiem_tra.txt, anh_vo_J2_moi.png); assets/Gui_xuong_in/G_vo_J2_cang_dai_giua_nhanh/AX-1_3_cang_dai_giua_nhanh_J2.stl.

## 2026-11-21 — developer — Gá giác hút J6 bản dài + file lắp ráp v3 (cấu hình robot thật)
- Gá giác hút: ống khí cứng bị gập 90° vì đầu đuôi khí chỉ cách mặt đế 21 mm (11 mm khi lò xo nén hết). Bản dài giữ nguyên mọi mặt lắp (đế 46 × 45, 4 lỗ M3 ở 60/120/240/300°, tấm trên có lỗ Ø12.4 cho ren M12 và 4 rãnh chữ U); khoang ống 36 → 66 mm, vát góc trong 4 → 6 mm; cao 45 → 75 mm.
  - Đầu đuôi khí cách mặt đế 51 mm (41 mm khi nén hết). Bán kính uốn tối đa của ống Ø10 khi ra cửa sổ: 38 mm khi nén hết (cũ 12 mm).
  - Miệng giác cách mặt bích J6 99 → 129 mm (tool_length ≈ 129 mm). Thể tích 49.9 cm3. STL kín, in nằm nghiêng, không cần support.
- Lắp ráp v3 (lấy từ v2, đổi theo người dùng):
  - Vỏ J2 = AX-1_3_cang_dai_giua_nhanh_J2; vỏ J3 = AX-2_1_cang_dai_J3 (bản 18/11); nắp J3 AX-2_3 gốc.
  - J4 cũ chưa rút ngắn (AX-3_1/AX-3_2 gốc, 17HS13-0404S-PG5); bầu AX-4_4_rut_ngan → 64 chi tiết phía cổ tay lùi 16 mm. Trục J3 → tâm cổ tay 267.0 → 251.0 mm.
  - Gá giác hút dài thay gá cũ.
- Kiểm tra:
  - 2 đầu bầu ngắn trùng 100 % với AX-4_4 gốc.
  - Chi tiết mới và nhóm lùi 16 mm không có chồng lấn mới; còn AX-1_6 × vỏ J2/J3 35.7/245 mm3, có sẵn từ bản gốc.
  - Quét J5: gá dài không chạm từ −125° tới +95° so với tư thế CAD, giống gá cũ.
- Chưa cập nhật động học twin (dong_hoc_step.py) theo cẳng tay 251 mm và đầu hút 129 mm.
- File:
  - src/cad/tao_ga_giac_hut_J6_dai.py, src/cad/cap_nhat_lap_rap_v3.py, src/cad/ve_anh_lap_rap_v3.py (mới).
  - assets/CAD_giac_hut_dai/ (Ga_giac_hut_J6_dai.step, Ga_giac_hut_J6_dai_huong_in.stl, Lap_rap_ga_giac_hut_J6_dai.step, ket_qua_kiem_tra.txt, 2 ảnh).
  - assets/Gui_xuong_in/H_ga_giac_hut_J6_dai/Ga_giac_hut_J6_dai_huong_in.stl.
  - assets/CAD_lap_rap_v3/ (Roboter_v3_J4_cu_bau_ngan_giac_hut_dai.step, ket_qua_kiem_tra.txt, anh_lap_rap_v3.png).

## 2026-11-21 — developer/tester — Động học twin theo lắp ráp v3 (bầu ngắn + gá giác hút dài)
- src/cad/tinh_dong_hoc_tu_step.py:
  - Trục J5, J6 và mesh khâu 5, 6 lùi 16 mm dọc cẳng tay.
  - Mesh khâu 4 co đoạn bầu s 154.5..187 → 16.5 mm. Mesh gốc cất ở assets/ROBOTARM2_description/meshes/goc_truoc_v3/; script luôn đo độ lệch trên mesh gốc.
  - Thêm TOOL_LENGTH_MM = 126.0: miệng giác cách mặt bích AX-6 129 mm; mặt bích cách điểm J6 4.41 mm; trừ flange_offset 7.4 của kinematics_poe.
- Kết quả:
  - J1–J4 không đổi; J5, J6 dời 16.00 mm; trục khớp không đổi hướng.
  - J3 → tâm cổ tay 267.0 → 251.0 mm.
  - TCP tư thế 0: (338.5, 0, 354.0) với đầu hút 30 → (418.5, 0, 354.0) mm với đầu hút 126.
  - IK vòng kín 200 điểm: 200/200, sai 0.000 mm. GUI chạy 25 s không lỗi.
- main.py (+ Changes/main.py): tool_length lấy từ dong_hoc_step.TOOL_LENGTH_MM (trước cố định 30 mm).
- SortingStation/config/station.yaml:
  - robot.tool_length_mm 30 → 126.
  - robot.scale 0.80 (tạm, ước lượng từ ảnh) → 1.0: robot in 1:1 từ file CAD. Ở 0.80 với đầu hút 126 mm, cả 3 thùng ngoài vùng với.
  - pytest SortingStation 22/22. Đầu hút 106–156 mm vẫn với tới DONG_BANG / LOI / TRUNG_DU.
- File:
  - dong_hoc_step.py, src/cad/tinh_dong_hoc_tu_step.py, main.py, Changes/main.py.
  - assets/ROBOTARM2_description/meshes/link_4_1.stl (co bầu) + meshes/goc_truoc_v3/ (bản gốc 7 mesh).
  - reports/dong_hoc/dong_hoc_tu_step.txt, reports/dong_hoc/twin_v3_tu_the_0.png.
  - ../SortingStation/config/station.yaml.

## 2026-11-21 — developer — Chuẩn bị đưa toàn hệ thống lên GitHub (chưa push)
- `.gitignore` robotarm: thêm `.tmp/`, `.claude/`, `research/cad_cache*`, `.pytest_cache/`, `*.bak` và 7 file lắp ráp `assets/**/Roboter*.step` (70–76 MB, vượt ngưỡng 50 MB của GitHub).
- Lắp ráp cuối nén thành `assets/CAD_lap_rap_v3/Roboter_v3_J4_cu_bau_ngan_giac_hut_dai.step.zip` (15 MB).
- `SortingStation/.gitignore` (mới).
- Hướng dẫn tự upload và README repo tổng: `docs/github/HUONG_DAN_UPLOAD_GITHUB.md`, `docs/github/README_repo_tong.md`.
- Đã thử: lọc robocopy (MachineVision 120 file, ảnh còn lại đều bị .gitignore chặn trừ logo và sơ đồ mạch), `git reset` trên repo chưa có commit, các luật ignore không làm rơi file mã nguồn.
