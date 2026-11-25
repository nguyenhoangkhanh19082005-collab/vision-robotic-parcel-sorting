# CHANGELOG — MachineVision

## 2026-11-25 — developer (giao diện: một màn hình vận hành cho cả trạm)
Vấn đề của giao diện cũ (dựng ngầm ở 1920×1080 để soát):
- Nút Chạy tự động và Đã lấy kiện nằm ở tab "Băng tải / ESP32" (trang gần như trống). Trang Vận hành không
  có nút chạy, không thấy trạng thái băng tải hay web.
- Thẻ Phân loại / Robot lặp ở 2 tab. Robot GUI có giao diện riêng (cyan).
- Góc đẹp nhất của trang dành cho chỉ số kỹ thuật (Laplacian, Tenengrad).
- Tiêu đề chỉ nói về camera ("MV-01 / PARCEL INSPECTION CELL"). Lẫn Anh và Việt, chữ in hoa khắp nơi.
- Không thấy kiện đang ở bước nào trong trạm.

Thiết kế (theo skill frontend-design; không dùng superdesign vì đó là công cụ canvas HTML, không áp vào Qt được):
- Màu lấy từ chính trạm:
  - Nền xanh xám như mặt băng tải (`#131b1a`, khối nội dung `#1b2523`).
  - Nâu kraft của thùng carton (`#d49b4e`) cho kiện đang xử lý và nút chính.
  - Màu trạng thái: đạt / loại / xem lại / web.
- Chữ: Bahnschrift (DIN, có sẵn trong Windows, có dấu tiếng Việt) cho tiêu đề và số; Segoe UI cho nội dung.
  Viết hoa đầu câu, không dùng chữ in hoa trang trí.
- Điểm nhấn duy nhất: dải hành trình kiện đánh số 1–5:
  Camera kiểm tra → Tới điểm gắp → Chờ gắp → Robot thả vào thùng → Lưu lên web.
  - Bước đang chạy sáng màu kraft, bước đã qua có dấu ✓, kiện lỗi chuyển sang màu đỏ, sự cố tô đỏ đậm.
  - Bước "Lưu lên web" theo trạng thái gửi.

Thay đổi:
- Thêm `ui/station_theme.py`: PALETTE, STYLE, STATE_TEXT/COLORS, DECISION_TEXT/COLORS, `ProcessRoute`.
- `ui/main_window.py`, `_build_ui` viết lại:
  - Thanh tiêu đề: "Trạm phân loại kiện hàng", các ô trạng thái tiếng Việt, đồng hồ, `header_actions`.
  - Trang "Vận hành", 3 cột:
    - Camera (ảnh lớn; Kết nối camera / Kiểm tra tay / Chụp nền trống).
    - Chạy tự động (Bắt đầu / Đã lấy kiện, phím Space; dòng pha; PWM 40 cố định). Đây là
      `operator_control_layout`.
    - Kiện vừa kiểm tra, Ca làm việc (5 số), Các kiện trước / Cảnh báo (rê chuột vào dòng để xem lý do).
  - Tab "Mô hình 3D".
  - Tab "Băng tải & camera": băng tải chạy tay, chỉ số độ nét, lấy mẫu logo, chế độ bàn thử, focus peaking.
  - Các widget giữ nguyên tên biến và logic; chỉ đổi vị trí, chữ và kiểu dáng.
  - `_on_auto_phase` cập nhật dải hành trình; `_refresh_upload_status` cập nhật bước web.
  - Kết quả hiện "Đạt, sọt 5" / "Loại" / "Cần xem lại".
- `ui/conveyor_panel.py`: chữ thường (Chạy tiến / Chạy lùi / Dừng / Kiểm tra kết nối), câu trạng thái rõ
  nghĩa.
- SortingStation:
  - Mô hình 3D thu gọn và thẻ Robot 6 trục nằm trong cột Chạy tự động.
  - Dừng toàn trạm nằm trên thanh tiêu đề.
  - Tab "Robot", tab "Phân loại & tư thế".
  - Robot GUI dùng chung giao diện trạm.
  - Robot sáng ở bước 4 khi đang gắp.

Kiểm thử:
- Dựng ngầm ở 1920×1080 và 1366×768, có kết quả thật (burst `station_runs/20261125_121056/p05`) và pha "Tới
  điểm gắp"; cả bản chạy riêng MachineVision.
- Smoke GUI tích hợp:
  - Có robot mô phỏng: 2/2 PASS, robot thả 2 kiện.
  - Không robot, có bấm Đã lấy kiện: 2/2 PASS, băng đứng yên tới lúc bấm.
- MachineVision 183/183 (gồm 52 ca ảnh thật), SortingStation 22/22.
- Chưa xem trên màn hình thật với OpenGL: khung mô hình 3D trống khi dựng ngầm, cùng widget như trước.
- Sao lưu: `.bak_2026-11-25e/`.

## 2026-11-25 — developer (upload trạng thái đơn lên web: sẵn sàng cho chạy tự động)
Kiểm tra hiện trạng:
- Database Supabase đã đủ: `sorting_logs`, trigger `trg_sorting_logs_apply` (PASS → đơn `SORTED`), 3 view
  theo dõi.
- 3 mã tem đang dùng đều có trong `orders`: SPXVN76923235515 → sọt 5, SPXVN06275784488 → sọt 5,
  SPXVN67182584388 → sọt 4.
- Máy trạm chưa gửi được gì vì 4 lỗi:
  1. `supabase` chưa cài trong `E:\UDMT\.venv`.
  2. `run_station.bat` (SortingStation) không truyền reporter vào cửa sổ.
  3. Tra cứu và gửi chạy đồng bộ trong chu trình băng tải: chờ mạng (máy chủ ap-northeast-1), mạng treo
     thì kiện đứng dưới camera.
  4. `enabled: true` mà thiếu `.env` thì phần mềm không khởi động được.

Sửa:
- Cài `supabase` 2.34.1 và `python-dotenv` (pip cache ở `E:\UDMT\.tmp\pip`).
- `repositories/sorting_logs.py`: `SortingLogReporter(background=True)`.
  - Ghi outbox trước, luồng nền gửi theo thứ tự, mất mạng thì thử lại mỗi `retry_s`.
  - Thêm `status()` (READY/ONLINE/OFFLINE, số dòng chờ gửi, lỗi cuối) và `close()`.
- `repositories/shipments.py`:
  - `CachedShipmentRepository`: bản sao cột định tuyến của `orders`, làm mới nền mỗi `cache_refresh_s`.
  - `LookupUnavailable`.
  - Client có thời gian chờ 4 s (`postgrest_client_timeout`).
- `services/classifier.py`: không tra cứu được đơn thì trả MANUAL_REVIEW, không REJECT "mã lạ".
- `app.py`:
  - `prepare_supabase()`: thiếu thư viện hoặc `.env` thì tự chạy chế độ cục bộ và báo lý do.
  - Reporter chạy nền, đóng khi thoát.
- `ui/main_window.py`: ô SUPABASE cập nhật mỗi 2 s theo `reporter.status()`, có số dòng chờ gửi và
  tooltip.
- SortingStation `app.py` và `main_window.py`: tạo và truyền reporter, đóng khi thoát.
- `config/default.yaml`: `supabase.enabled: true`, `local_test_mode: false`, `cache_refresh_s: 60`,
  `retry_s: 10`.

Kiểm thử:
- `tests/test_upload_background.py` (4 test):
  - Gửi nền không chặn chu trình.
  - Mất mạng rồi có lại, kể cả khởi động lại: gửi đủ, đúng thứ tự.
  - Bản sao đơn vẫn tra được khi mất mạng; mã lạ lúc mất mạng thành MANUAL_REVIEW.
  - Thiếu `.env` thì chạy cục bộ.
- Database thật (giao dịch có rollback): dòng PASS SPXVN76923235515 tự gắn đơn 20, đơn → `SORTED` +
  `sorted_at`. Sau rollback: 0 log, đơn vẫn `READY_TO_SHIP`.
- MachineVision 117 test (không tính test phát lại), SortingStation 22/22.

Còn lại:
- Người dùng tạo `.env` (URL và key máy trạm).
- RLS đang TẮT trên cả 7 bảng (cảnh báo critical của Supabase). Chưa tự bật vì bật mà không có policy sẽ
  chặn cả web lẫn máy trạm.

## 2026-11-25 — tester (kiểm lại lần 4 trên băng thật: kiện sát nhau, chạy lùi về camera)
- Lần chạy `logs/runs/20261125_123942`: PWM 40, GUI đã mở lại (.pyc 12:30).
- 9 thư mục kiện, 7 lần kiểm tra, **7/7 PASS** (QR 100%):
  - 4 kiện được camera dừng khi chạy tiến.
  - 3 kiện bị cuốn qua camera được chạy lùi về (p04→p05, p06→p07, p08→p09), camera dừng khi đang lùi,
    kiểm tra PASS, rồi tới điểm gắp.
  - Không có MANUAL_REVIEW, không FAULT.
- Thời gian chạy lùi: lần đầu khoảng 8,2 s (kiện chỉ chạy tiến khoảng 2,4 s sau khi ra khỏi khung, chưa rõ
  nguyên nhân: có thể băng trượt khi mới đổi chiều). Hai lần sau khoảng 3 s, đối xứng với chạy tiến.
  `backup_max_s` 12 → 15.
- Đưa vào `dataset/station_runs/` (9 ca, burst của p05 bắt khi chạy lùi). Tổng 52 ca, phát lại 0 lỗi.
- MachineVision: toàn bộ test đạt.

## 2026-11-25 — developer (lần chụp 4–5: kiện sát nhau bị cuốn qua camera; cố định PWM 40)
Dữ liệu: `logs/runs/20261125_121056` (lần 4, 11 kiện), `_121357` (PWM 30), `_121500` (PWM 50).
- Mọi kiện camera đã dừng đều cho kết quả đúng:
  - Tem SPXVN76923235515 và SPXVN67182584388 (tem mới): PASS, QR 100%.
  - ND-18 (QR nhòe), CG53 (QR rách), UB20 (QR mờ): REJECT.
- 5 kiện MANUAL_REVIEW (kiện tốt rơi xuống sọt lỗi):
  - Lần 4 p02, p11; PWM 50 p03: kiện sau bị cuốn qua vùng camera trong lúc kiện trước đang chuyển tới
    điểm gắp. Lúc chu trình theo dõi lại, kiện đã ở mép dưới khung (0,56–1,00).
  - Lần 4 p03; PWM 30 p02: dừng ma ở cảm biến (camera không thấy kiện nào). Bản sửa lần 3 xử lý được
    ca này nhưng GUI chưa được mở lại: .pyc từ 11:49, code sửa từ 11:54–11:59, nhật ký không có dòng
    "bỏ qua".

Sửa:
1. `services/station_cycle.py`: thêm `_back_up()`. Kiện tới điểm gắp mà chưa được kiểm tra ("missed",
   camera đã thấy kiện qua mép dưới) thì:
   - Băng chạy lùi (REV, PWM chạy) tối đa `backup_max_s` (12 s), tới khi camera dừng được kiện.
   - Sau đó căn lại vị trí bằng xung FWD, kiểm tra, và đi tiếp như kiện bình thường.
   - Chỉ chạy lùi khi vùng camera trống (`view_clear`), để không đẩy kiện vừa đặt ở đầu băng rơi ra.
   - Không bắt được kiện thì quay lại cách cũ (MANUAL_REVIEW, rơi sọt lỗi).
   - `_recenter(direction)` căn lại được cả hai chiều.
2. `config/default.yaml`: `station_cycle.speed: 40`. Chạy tự động luôn ở PWM 40, không lấy theo thanh
   trượt, để thời gian camera → điểm gắp khớp với `pick_stop_delay_ms`. Thêm `backup_max_s: 12`.
3. `services/arrival.py` (quy tắc kiện dài):
   - `overlong_stop_at` 0,91 → 0,93. Ở PWM 30, kiện dài 0,91 khung bị dừng sớm 2 khung; lúc chạy thật
     nó nằm trọn khung.
   - Thêm dừng muộn `lead_was_in`: đầu kiện đã thấy trong khung rồi vọt ra ngoài thì dừng ngay, để căn
     lại xử lý phần lố.
4. `ui/run_recorder.py`: lưu thêm khoảng 0,4 s ảnh sau lúc dừng (luồng riêng, `tail_frames` trong
   meta), để phát lại kiểm được trường hợp dừng muộn 1 khung.
   - `run_replay.replay_arrival` chấp nhận "dừng ở khung kế tiếp" cho bản ghi cũ không có phần đuôi.
5. `controllers/fake_esp32.py`: giả lập băng chạy lùi.

Test:
- `dataset/station_runs/`: 43 ca thật (516 MB), phát lại 0 lỗi.
- Thêm test: kiện bị cuốn qua được lùi về và PASS, lưu ảnh sau lúc dừng, kiện dài dừng muộn.
- MachineVision 169/169, SortingStation 22/22.
- Smoke GUI không robot: PWM 40, 2/2 PASS.
- Chưa kiểm trên băng thật: chạy lùi đưa kiện về camera (hướng REV, PWM 40).
- Sao lưu: `.bak_2026-11-25c/`.

## 2026-11-25 — developer (lần chụp 3: kiện lỗi, kiện dài hơn khung, lần dừng ma ở cảm biến)
Dữ liệu: `logs/runs/20261125_114818` (GUI còn code cũ) và `_114924` (7 kiện).
- 4 kiện lỗi đều REJECT đúng:
  - HN-29-02-TX..: tem rách, mất QR.
  - HN-29-02-CG53: QR rách hoặc xước.
  - HP-16-04-UB20: QR mờ.
  - ND-18-01-XT66: QR in nhòe.
- Kiện tốt đặt chéo: quy tắc kiện dài chạy đúng trên băng thật, PASS, QR 100%.

Lỗi tìm thấy và cách sửa:
1. **Kiện QR mờ đặt chéo (dài hơn khung) không được dừng.** Đầu kiện đọc được 0,92, rồi 0,85, rồi
   1,00 (góc kiện chạm vệt sáng dưới), nên khung "sắp ra khỏi khung" bị trượt qua.
   - `arrival.py`: kiện dài được dừng khi vùng phát hiện phủ >= 0,86 khung, đuôi bị mép trên cắt, và
     đầu kiện >= `overlong_stop_at` (0,91, vị trí đo thật, không cộng bù trễ).
   - Kiện vẫn nằm trọn được (khoảng 0,87 khung, lần 3 p07) vẫn dừng theo quy tắc nằm trọn khung như
     lúc chạy thật.
2. **Lần dừng ma.** Cảm biến điểm gắp bị che (tay, kiện vừa nhấc) trong khi kiện thật còn dưới camera.
   Chu trình ghi MANUAL_REVIEW, rồi chính kiện đó lại bị ghi thêm một lần nữa.
   - `arrival.py`: thêm `passed_downstream`, bật khi một vùng đang chạy chạm mép dưới khung; xóa ở
     `rearm()`.
   - `station_cycle.py`: thêm tham số `parcel_passed`. Băng dừng ở cảm biến mà camera chưa thấy kiện
     nào đi qua thì bỏ qua và chạy tiếp, không ghi kết quả.
3. **Vận tốc bị kéo về gần 0** khi một đầu kiện kẹt ở vệt sáng (lần 3 p03, phát lại không dừng):
   `_end_velocity` lấy trung vị riêng cho từng đầu kiện, chọn đầu chạy nhanh hơn.
4. Sửa lỗi chèn lặp đoạn chú thích và trường `overlong` trong `arrival.py` (script vá lần 2 bị chạy 2
   lần).
5. Manifest hỗ trợ dạng "OVERLONG PASS" (`run_replay.parse_expectation`).

Test:
- `dataset/station_runs/`: 26 ca thật (353 MB), phát lại 0 lỗi.
- Thêm test: đầu kiện dài nhảy giá trị, `passed_downstream`, tay che cảm biến bị bỏ qua.
- MachineVision 146/146, SortingStation 22/22.
- Smoke GUI không robot: 2/2 PASS, băng đứng yên tới lúc bấm ĐÃ LẤY KIỆN.
- Sao lưu: `.bak_2026-11-25b/`.

## 2026-11-25 — developer (lần chụp 2: kiện PASS bị đẩy rơi xuống sọt lỗi, kiện đặt chéo bị bỏ sót)
Dữ liệu: 5 lần chạy `logs/runs/20261125_1112..1117`, 16 kiện.
- Tem SPXVN76923235515: 9/9 PASS, QR đọc 100%.
- Tem ND-18-01-XT66 là tem mẫu "QR in nhòe" (chữ sắc nét, phần dữ liệu QR nhòe): 2/2 REJECT, đúng.

Lỗi tìm thấy và cách sửa:
1. **Kiện PASS bị đẩy rơi xuống sọt lỗi.** Firmware v2.1 trễ 2200 ms nên kiện dừng khi đã qua khỏi
   cảm biến. Không có robot thì chu trình chỉ chờ cố định 8 s rồi chạy băng tiếp.
   - `config/default.yaml`: `manual_pick_s: 0`, nghĩa là chờ người vận hành xác nhận.
   - `services/station_cycle.py` `_wait_pick`: `manual_pick_s <= 0` thì không có hẹn giờ.
   - `ui/main_window.py`: thêm nút **ĐÃ LẤY KIỆN (phím Space)**, chỉ bật ở pha PICKING khi không có
     robot.
   - `ui/auto_cycle.py`: thêm `notify_manual_pick()`; `_pick_done` trả True khi đã xác nhận.
2. **Kiện đặt chéo không được dừng (3 kiện thành MANUAL_REVIEW).** Xoay khoảng 35° thì kiện dài khoảng
   0,95 khung, không bao giờ nằm trọn khung.
   - `services/arrival.py`: thêm quy tắc kiện dài (overlong). Đuôi kiện còn bị mép trên cắt, đầu kiện
     sắp ra khỏi khung, và vùng phát hiện dài >= 0,86 khung thì dừng ngay lúc đó.
   - Ngưỡng 0,86 tránh dừng sớm khi tay buông kiện làm vận tốc vọt lên.
3. **FAULT "Băng tải dừng ngoài chu trình" (2 lần).** Firmware dừng AUTO (ASTOP=1) khi vật đã qua khỏi
   cảm biến.
   - `_wait_for_camera` trả về `missed_past`.
   - `_eject(past_sensor=True)` không chờ cảm biến nữa: băng đang chạy sẽ đẩy kiện rơi.
4. Phát lại burst giống chu trình: thêm `station_cycle.inspection_failure()` và
   `run_replay.replay_inspection()`. Trước đây lỗi "QR anchor" làm script phát lại dừng hẳn.
5. Recorder ghi thêm nhật ký các pha (`phases` có giờ) và cờ `overlong` lúc dừng.

Test:
- `dataset/station_runs/` thêm 16 kiện lần 2 (5 kiện có burst), 207 MB. manifest có PASS/REJECT/
  NO_STOP/OVERLONG.
- Phát lại 18 ca: 0 lỗi.
- Thêm test kiện dài hơn khung, kiện bị bỏ sót đã qua cảm biến, không robot chờ xác nhận.
- MachineVision 131/131, SortingStation 22/22.
- Smoke GUI tích hợp: có robot mô phỏng 2/2 PASS; không robot có bấm ĐÃ LẤY KIỆN 2/2 PASS, băng đứng
  yên tới lúc bấm.
- Sao lưu: `.bak_2026-11-25/`.

## 2026-11-25 — developer (băng trống bị dừng nhầm do vệt phản chiếu đèn chạy theo băng)
- Lần chụp 1 (băng trống, 2 lần chạy `logs/runs/20261125_1059*`, `_1102*`): sau khoảng 5 s bộ dò
  báo có kiện, băng dừng, kiểm tra ra REJECT "không đọc được QR".
- Nguyên nhân (ảnh trong bản ghi): đèn phản chiếu trên mặt băng bóng thành một vệt sáng. Mặt băng
  gợn sóng nên khi băng chạy vệt sáng di chuyển và đổi hình, qua được quy tắc "chỉ dừng vật đang
  di chuyển".
- Đo trên ảnh thật (trung vị theo vùng):
  - Vệt sáng: L 18–82, độ chi tiết 4,5–17,8.
  - 9 kiện thật 23/11: L 130–150, độ chi tiết 14–27.
- `services/arrival.py`:
  - Một vùng chỉ là kiện khi trung vị L >= `object_min_lightness` (100) hoặc độ chi tiết >=
    `object_min_texture` (30).
  - Điểm ảnh tối (< 80) và trơn (< 8) bị bỏ khỏi mặt nạ trước khi tách vùng, để không dính vào kiện.
  - Thêm `blob_lightness`, `blob_texture`, `glare_skipped` để chẩn đoán.
- `ui/auto_cycle.py`: log CSV thêm cột `blob_l,blob_tex,glare_skipped`; `meta.json` lưu L và độ chi
  tiết lúc dừng.
- `config/default.yaml`: thêm 4 ngưỡng trên.
- Thêm `services/run_replay.py`, dùng chung cho `scripts/replay_station_runs.py` và test.
- Bộ dữ liệu `dataset/station_runs/` (manifest.json): 2 lần chạy băng trống, NO_STOP, chỉ lưu ảnh lúc
  phát hiện (15 MB).
- Test:
  - Thêm `tests/test_station_runs.py` (phát lại bản ghi thật).
  - Thêm test vệt sáng tổng hợp chạy và đổi hình.
  - Phát lại 2 lần chạy băng trống: 0 lần dừng (trước là 1 lần mỗi lần chạy).
  - Diễn tập 3/3. MachineVision 107 test đạt, 1 bỏ qua (chưa có ca kiện có burst).

## 2026-11-25 — developer (ghi lại mọi lần chạy tự động để kiểm thử bằng ảnh thật)
- Thêm `src/machine_vision/ui/run_recorder.py`: khi chạy tự động bằng camera, mỗi kiện được lưu vào
  `logs/runs/<giờ chạy>/pNN/`, gồm:
  - `arrival/`: 150 khung cuối trước lúc dừng, rộng 1280 px, kèm `stamps.json` (thời điểm chụp từng
    khung).
  - `burst/`: 15 khung chụp lúc kiểm tra, độ phân giải đầy đủ.
  - `meta.json`: khung dừng, trạng thái bộ dò, kết quả, QR, lý do.
  - Mỗi lần chạy lưu thêm `background.png` và `settings.json`.
  - Ghi ở luồng riêng nên không ảnh hưởng thời điểm dừng. Tự ngừng ghi khi ổ đĩa còn dưới 400 MB.
  - Tắt bằng `station_cycle.record_runs: false`.
- `src/machine_vision/ui/auto_cycle.py`: nối recorder vào `camera_arrival`, burst, kết quả, pha
  SETTLING/FAULT và `rearm`.
- `scripts/replay_station_runs.py`: phát lại các lần chạy đã ghi qua code hiện tại (bộ dò và pipeline
  kiểm tra), so với kết quả lúc chạy thật và với manifest kỳ vọng.
- `services/arrival.py`: điểm ảnh khác nền mà đứng yên quá `static_absorb_s` được học vào ảnh nền
  theo từng điểm ảnh, kể cả khi vùng đó dính liền với kiện (trước đây chỉ học khi cả vùng đứng yên).
- Test:
  - Thêm `tests/test_run_recorder.py` (2 test). MachineVision 103/103.
  - Smoke GUI tích hợp (ESP32 giả lập): 2/2 kiện dừng giữa khung (0,17–0,81), PASS, QR 100%.
  - Phát lại cho đúng khung dừng 44 và 715, đúng kết quả PASS.

## 2026-11-24 — developer (camera chỉ dừng được kiện đầu tiên: bật lại theo từng kiện, học vùng đứng yên)
- Hiện tượng: kiện lỗi đặt lên băng không được dừng để kiểm tra.
- Log `logs/arrival/arrival_20261124_1948..1955.csv`: mỗi lần chạy chỉ dừng được 1 kiện. Sau đó bộ dò
  bị khóa 30–40 s vì nó đợi thấy băng tải trống mới bật lại, nhưng khung hình không lần nào trống
  (luôn còn một vùng thay đổi đứng yên ở nửa dưới, có lúc phủ cả khung).
- `src/machine_vision/services/arrival.py`:
  - `rearm()`: chu trình bật lại bộ dò trước mỗi kiện, không đợi băng tải trống.
  - Chỉ vùng thay đổi **có di chuyển** (|v| >= `moving_speed`) mới được dừng. Bỏ quy tắc "kiện
    nằm yên trọn khung thì dừng", vì lúc bộ dò chạy thì băng luôn đang chạy.
  - Vùng mà >= 90% điểm ảnh không đổi quá 1 s (vệt sáng, bóng đổ, ray, camera bị xê dịch) bị
    bỏ qua. Sau 4 s thì vùng đó được học vào ảnh nền.
  - `light_fault()`: độ sáng ngoài 0,5–2 lần ảnh nền quá 2 s (đèn tắt) thì báo lỗi.
  - `background_mismatch` tính cả vùng đã bị bỏ qua.
- `src/machine_vision/ui/auto_cycle.py`:
  - `rearm` gọi `detector.rearm()`.
  - FAULT "đèn tắt" có hướng dẫn xử lý. FAULT lệch nền chờ 6 s (vùng lệch đứng yên đã được học
    trước đó).
  - Log CSV thêm cột `moved,gain,static_ratio,absorbed`.
- `config/default.yaml` (`station_cycle.arrival`):
  - ROI thu vào mặt băng giữa 2 thanh ray: [0.35, 0, 0.30, 1]. Ray nhôm trắng đổi sáng theo đèn.
  - Thêm các tham số `moving_speed`, `static_still_s`, `static_absorb_s`, `light_gain_range`,
    `light_fault_s`.
- `scripts/rehearse_camera_cycle.py`:
  - Ảnh nền lấy trung vị các ảnh băng trống 23/11, cùng buổi với ảnh kiện. Ảnh nền 24/11 chụp sau
    khi camera bị dịch nên không khớp.
  - Dùng `detector.rearm`.
- Test:
  - Thay 2 test theo hành vi mới.
  - Thêm 3 test: vệt sáng đứng yên suốt 3 kiện liên tiếp (tái hiện lỗi log 19:50), camera xê dịch
    được tự học rồi vẫn dừng kiện sau, đèn tắt.
  - MachineVision 101/101, SortingStation 22/22.
  - Diễn tập 3/3: kiện thứ 2 (REJECT) cũng dừng, lệch tâm +7 / +10 px (trước là +14 / +19 px).
  - Xử lý 12 ms mỗi khung 2K.
- Sao lưu: `.bak_2026-11-24b/` (`arrival.py`, `auto_cycle.py`, `default.yaml`, `test_arrival.py`).
- Chưa kiểm trên băng thật. Khung chụp thử 24/11 lúc đèn tắt có độ sáng x0,40 so với ảnh nền, và
  trường hợp này giờ sẽ báo FAULT đèn.

## 2026-11-24 — developer (trễ dừng điểm gắp 1200 → 2200 ms)
- Người vận hành yêu cầu kiện chạy thêm 1 s trước khi dừng để gắp.
- `firmware/conveyor_sensor/conveyor_sensor.ino`: `DEFAULT_AUTO_DELAY_MS` 1200 → 2200.
  Build lại `export/conveyor_sensor.ino.merged.bin`, SHA-256 9163989B…C67AB4.
- `config/default.yaml`: `station_cycle.pick_stop_delay_ms` 1200 → 2200. PC gửi số này qua
  `AUTO:1:<ms>` và đè lên giá trị trong firmware.
- `firmware/conveyor_sensor/NAP_FIRMWARE.md`: cập nhật SHA-256 và số ms.
- Theo yêu cầu: không chạy test mô phỏng; chỉ biên dịch.

## 2026-11-24 — developer (camera không dừng kiện nào: ảnh nền lệch do camera bị xê dịch)
- Hiện tượng: kiện tốt hay lỗi đều không được camera dừng. Log `logs/arrival/arrival_20261124_17*.csv`:
  mọi khung có một khối "thay đổi" phủ kín từ mép trên tới mép dưới (0,00..1,00), nên không kiện nào
  "nằm trọn khung".
- Nguyên nhân: camera đã bị dịch hoặc chỉnh góc sau khi chụp ảnh nền 23/11. Băng tải to hơn và lệch
  phải; so với nền cũ, 32% khung trống bị coi là thay đổi.
- Sửa:
  - Chụp lại `config/calibration/empty_belt.png` (trung vị 30 khung, băng tải trống). Bản cũ lưu ở
    `.bak_2026-11-24/calibration/empty_belt_2026-11-23.png`.
  - Kiểm tra trên băng tải trống: không kích hoạt lần nào, kể cả với đèn ×0,7 và ×1,4.
  - Mô phỏng kiện thật trên nền mới ở 0,2 / 0,4 / 0,7 khung/s: dừng trọn khung.
  - Kiện mẫu thật vừa đặt: phát hiện đúng thùng; QR SPXVN76923235515 đọc 100% khung, PASS.
- `services/arrival.py`: thêm `background_mismatch()`. Cả khung khác ảnh nền liên tục quá 3 s thì
  `ui/auto_cycle.py` báo FAULT, dừng băng tải và nhắc bấm CHỤP NỀN TRỐNG, thay vì chạy âm thầm mà
  không dừng kiện nào. Thêm 1 test; MachineVision 98/98.

## 2026-11-24 — developer (firmware v2.1: dừng trễ ở điểm gắp)
Người vận hành ước lượng từ lúc cảm biến quang thấy kiện tới lúc kiện tới điểm gắp là 1–1,5 s
(cảm biến đặt trước điểm gắp). Trước đây ESP32 dừng ngay (`pick_stop_delay_ms: 0`).
- `firmware/conveyor_sensor/conveyor_sensor.ino` v2.1:
  - `DEFAULT_AUTO_DELAY_MS` = 1200, dùng cho `AUTO:1` không kèm số ms và khi khởi động.
  - Kiện thứ hai chạm cảm biến khi đang đếm trễ không làm lùi thời điểm dừng của kiện đầu.
  - STATUS thêm `ASTOP=<0|1>`, cho biết lần dừng gần nhất do AUTO (lúc dừng kiện có thể đã qua
    khỏi cảm biến).
  - Lệnh REV/STOP hủy lệnh dừng đang chờ.
  - Dòng MONITOR in thêm số ms trễ.
  - Biên dịch bằng arduino-cli (core 3.3.7), 24% flash.
  - `export/conveyor_sensor.ino.merged.bin`, SHA-256 4CB946D4…66679E.
- `config/default.yaml`: `station_cycle.pick_stop_delay_ms` 0 → 1200, thêm `manual_pick_s: 8`.
- `src/machine_vision/controllers/conveyor_udp.py`: đọc `ASTOP` (tùy chọn, vẫn hiểu v2.0),
  `ConveyorStatus.autostopped`.
- `src/machine_vision/services/station_cycle.py`:
  - `_transfer_to_pick` nhận dừng hợp lệ khi cảm biến có vật, hoặc ASTOP=1, hoặc PC đã thấy cảm
    biến trước đó.
  - Dự phòng dừng từ PC tính từ lần đầu thấy cảm biến, kể cả khi kiện đã qua khỏi cảm biến.
  - `_wait_pick` khi không có robot và kiện dừng ngoài cảm biến: chờ `manual_pick_s`.
- `src/machine_vision/controllers/fake_esp32.py`: mô phỏng giống v2.1 (ASTOP, không đếm lại trễ,
  robot lấy kiện vừa qua cảm biến).
- Test: thêm 2 test chu trình (dừng trễ, kiện nằm ngoài cảm biến, có và không có robot) và kiểm tra
  đọc STATUS v2.1. MachineVision 97/97, SortingStation 22/22, diễn tập 3/3.
- Sao lưu: `.bak_2026-11-24/` (firmware v2.0 nguồn + merged.bin, các file PC cũ).
- Chưa kiểm trên phần cứng: chưa nạp v2.1; 1200 ms là giá trị ước lượng, cần chỉnh theo vị trí
  dừng thật.

## 2026-11-23 — developer (camera dừng kiện không kịp trên băng tải thật)
Lỗi khi chạy thật: kiện tốt đặt vào thì camera không kịp ra lệnh dừng, kiện chạy tới tận cảm biến
(ghi MANUAL_REVIEW, rơi vào sọt lỗi). Nguyên nhân tìm được:
1. Luồng camera trong GUI chỉ đạt ~12 khung/s (đo trên camera thật): mỗi khung tính độ nét trên
   ảnh 2K mất 67 ms ngay trong vòng đọc ảnh.
2. Bộ dò kiện tính vận tốc theo tâm khối; khi tay rút khỏi kiện (đầu băng tải nằm trong khung),
   tâm khối nhảy, bị hiểu là kiện chạy rất nhanh → dự đoán dừng sẽ lố → không bao giờ bắn.
3. Kiểm tra "cơ hội cuối" giả định 30 khung/s.
4. PWM 150 quá nhanh.
Sửa:
- `src/machine_vision/ui/camera_worker.py`: tách luồng đọc ảnh (chỉ đọc + đưa vào bộ đệm, 29,9
  khung/s trên camera thật) và luồng xem trước (đo nét + khung kiện trên khung mới nhất, ~7
  khung/s khi GUI vẽ 30 ms).
- `src/machine_vision/services/arrival.py`: vận tốc = trung vị bước dịch của hai đầu kiện (bỏ đầu
  đang chạm mép); đo khoảng cách khung thật; bắn khi tâm tới giữa hoặc khi khung sau sẽ muộn;
  thêm `overshoot()` và `state.heading`, `state.blob`, `state.frame_interval`.
- `src/machine_vision/services/station_cycle.py`: `CycleSettings.speed_min/speed_max` (kẹp PWM
  chu trình về 30–50, `speed_range` trong cấu hình); `_recenter`: dừng lố thì lùi REV từng nhịp
  (`recenter_pulses` 3, `recenter_speed` 30, `recenter_pulse_s` 0,15 s) tới khi kiện nằm trọn khung.
  Thông báo RUNNING hiện PWM đang chạy.
- `src/machine_vision/ui/auto_cycle.py`: `camera_overshoot` (đo trên 2 khung mới sau khi dừng);
  ghi log từng khung của bộ dò ra `logs/arrival/arrival_<ngày_giờ>.csv` để chỉnh `latency_s`.
- `src/machine_vision/ui/main_window.py`: truyền thư mục `logs` cho chu trình.
- `config/default.yaml`: `controller.default_speed` 150 → 40; `station_cycle.speed_range`
  [30, 50], `recenter_*`.
- `scripts/rehearse_camera_cycle.py`: dùng ảnh trạm 23/11 (ảnh 10/11 chụp ở vị trí camera cũ, lệch
  ảnh nền mới nên cả bộ dò cũ cũng chỉ 1/3).
- Test: thêm 7 (tay rút khỏi kiện ở 30/12/8 khung/s, camera chậm + kiện dài, đo lố, lùi REV, kẹp
  tốc độ, đo lố trên khung mới). MachineVision 95/95, SortingStation 22/22, diễn tập 3/3.
- Sao lưu bản trước: `.bak_2026-11-23b/`.
- Chưa kiểm trên băng tải thật: `latency_s`, nhịp lùi REV (chiều và quãng lùi mỗi nhịp).

## 2026-11-23 — developer (tem A7 bản QR 28 mm + logo lớn)
- `outputs/tem_in/phong_to_qr_logo_tem.py` (mới): dựng lại tem web testqr (QR 20 mm) từ chính nét
  vector gốc: QR ×1,4 → 28 mm tại (44,25; 40,87) mm; logo ×1,35; mã vạch co ngang 0,78; cột
  "Nội dung hàng" co 0,87, dải giữa cao thêm 8 mm (lấy vào khoảng trống trên chân tem).
  Bản in: `outputs/tem_in/tem_A7_6_tem_testqr_QR28_logo_A4.pdf` (2 tờ A4, 6/6 QR đọc được).
- QR mới đọc được từ 24 dpi (bản 20 mm: 34 dpi); làm mờ σ = 1 px: 36 dpi (bản 20 mm: 50 dpi).
- `outputs/tem_in/ghep_tem_a4.py`: không tìm thấy khung trên tem đã dựng lại thì giữ khung mặc định.
- `config/default.yaml`: thêm khổ `A7_QR28` (QR ROI + mẫu/hiệu chỉnh logo riêng). CHƯA bật:
  `label_format` vẫn là "A7". Khi thử tem này: đổi `label_format: "A7_QR28"` rồi LẤY MẪU LOGO.
  88/88 test đạt.

## 2026-11-22 — developer (QR tem A7 phóng to 20 mm)
- Tem web A7 cũ: QR là ảnh JPEG 58×58 px in 15,4 mm (≈0,62 mm/ô), webcam khó đọc.
- `outputs/tem_in/phong_to_qr_tem.py` (ngoài MachineVision): tạo lại 10 tem thử với QR vector
  20×20 mm, phiên bản 1, mức sửa lỗi M (≈0,95 mm/ô), cùng nội dung mã vận đơn; khối
  "Ngày đặt hàng" thu nhỏ 80 % dưới QR. Bản ghép in: `outputs/tem_in/tem_A7_10_tem_A4_QR20mm.pdf`.
- So sánh render (zxing + OpenCV, upscale 1–3×): QR mới đọc được từ 36 dpi (cũ 50–60 dpi), có
  làm mờ σ = 1 px đọc được từ 60–72 dpi (cũ 85–100 dpi).
- `config/default.yaml`: `label_format` "A6" → "A7"; `label_formats.A7` thêm
  `label.qr_roi`, `label.rectification.qr_target_roi`, `package_detection.label_qr_roi` theo
  vị trí QR mới. Kiểm tra: nắn nhãn từ QR mới khớp khung tem; 88/88 test đạt.
- Chưa làm: mẫu logo A7 vẫn là mẫu tạm sinh từ A6, không khớp tem web → phải LẤY MẪU LOGO trên
  tem thật. Ngưỡng hư hỏng tính theo cạnh QR nay ứng với QR 20 mm (gần mức 21 mm của tem A6
  lúc hiệu chỉnh). Mẫu in trên web (server) chưa đổi.
