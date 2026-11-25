# Chu trình tự động và tem A7 (2026-11-21)

## 1. Yêu cầu chức năng (đính chính 2026-11-10)
1. Cảm biến quang thấy kiện → băng tải **dừng**.
2. Webcam kiểm tra **logo**, **mã QR** (đối chiếu bảng `orders` trên Supabase) và **tình trạng bề mặt** (tốt / móp méo / thủng lỗ).
3. Ghi kết quả lên trạm dữ liệu Supabase (`sorting_logs`): kiện đạt → sọt của đơn hàng; kiện lỗi hoặc không xác định → sọt loại.
4. Cho băng tải chạy tiếp tới điểm gắp của robot.

Tem vận đơn hiện là A6 (105×148 mm), sắp đổi sang A7 (74×105 mm).

## 2. Thiết kế
```
            ┌──────────── RUNNING: FWD, hỏi STATUS mỗi 0,25 s (nuôi watchdog 3 s) ─────────┐
            │                                                                              │
 cảm biến 1 ▼  ESP32 tự STOP sau auto_stop_delay_ms (AUTO mode, không phụ thuộc mạng)      │
            │  (mất gói AUTOSTOP → PC gửi STOP sau delay + stop_grace_s)                    │
 SETTLING   │  chờ settle_s cho hết rung                                                   │
 INSPECTING │  xóa bộ đệm, chụp loạt ảnh MỚI → QR + logo + bề mặt + tra Supabase           │
            │  ảnh mờ/chói → chụp lại tối đa quality_retries lần → vẫn lỗi: MANUAL_REVIEW   │
 RECORD     │  lưu bằng chứng + 1 dòng sorting_logs (mất mạng → outbox, gửi lại sau)       │
 RELEASING  │  FWD tới điểm gắp, chờ cảm biến về 0 trong release_timeout_s ────────────────┘
 FAULT      │  WATCHDOG/OVERCURRENT/LINK_DOWN, băng tải dừng ngoài chu trình, kiện kẹt
            ▼  → gửi STOP + AUTO:0, báo lỗi trên GUI, chờ người vận hành
```

| Thành phần | File |
|---|---|
| Máy trạng thái (không phụ thuộc Qt, test được) | `src/machine_vision/services/station_cycle.py` |
| Kênh lệnh 8120 + sự kiện 8121, `STATUS`, `AUTO` | `src/machine_vision/controllers/conveyor_udp.py` (`ConveyorLink`) |
| Luồng Qt, chụp loạt ảnh mới | `src/machine_vision/ui/auto_cycle.py` |
| Nút **CHẠY TỰ ĐỘNG** | tab **BĂNG TẢI / ESP32** trong `ui/main_window.py` |
| ESP32 giả lập (đúng giao thức firmware v2) | `src/machine_vision/controllers/fake_esp32.py` |
| Tham số | `station_cycle:` trong `config/default.yaml` |

Quy tắc an toàn:
- `STATUS` là nguồn sự thật, sự kiện UDP chỉ để phản ứng nhanh, vì UDP có thể mất gói.
- Nút **DỪNG** của tab băng tải và việc tắt camera đều kết thúc chu trình. Khi kết thúc, chu trình luôn gửi `STOP:0` và `AUTO:0:0`.
- Không thay thế nút dừng khẩn phần cứng.

Quyết định sọt (`repositories/sorting_logs.py`):

| Kết quả | Trạng thái | Sọt |
|---|---|---|
| PASS (QR có trong `orders`, logo đúng, không lỗi) | `SORTED` | `orders.destination_chute_id` |
| REJECT (thủng, móp viền, rách nhãn, sai logo, mã không có trong `orders`) | `REJECTED` | `reject_chute_id` (0) |
| MANUAL_REVIEW (ảnh mờ, không thấy QR, chưa hiệu chuẩn) | `MANUAL_REVIEW` | `reject_chute_id` (0) |

Kiện vẫn được cho chạy tới điểm gắp; robot đọc sọt từ dòng `sorting_logs`.

## 3. Tem A7
- `label_format: "A6"` hoặc `"A7"` trong `config/default.yaml`. Mỗi khổ có mẫu logo và file hiệu chỉnh riêng (`label_formats`), nên lấy mẫu A7 không ghi đè A6.
- Nắn nhãn neo theo QR. Mọi ngưỡng (rách nhãn, lỗ, móp viền) tính theo **cạnh QR**, nên không phụ thuộc khổ tem hay độ cao camera.
- Mẫu logo A7 hiện là **mẫu tạm** sinh từ mẫu A6 (thu 0,705 lần rồi phóng lại). **Khi có tem A7 thật:** đặt `label_format: "A7"`, mở GUI, đặt kiện, bấm **LẤY MẪU LOGO**, kéo chọn logo.
- Nếu bố cục A7 khác A6 (QR đặt ở chỗ khác so với logo), bước lấy mẫu logo tự tính lại vùng tìm logo quanh vị trí vừa chọn.

## 4. Kết quả kiểm thử (2026-11-21, không có phần cứng, không có tem A7)

### 4.1 Test tự động
`pytest`: 75/75 đạt, trong đó có 5 test chu trình chạy qua UDP thật trên localhost với ESP32 giả lập:
- 3 kiện liên tiếp: mỗi lần kiểm tra đều diễn ra khi băng tải đã dừng và kiện đang ở cảm biến.
- Ảnh mờ: được ghi MANUAL_REVIEW, kiện vẫn được cho chạy tiếp.
- Mất AUTOSTOP: PC tự gửi STOP.
- Sự kiện quá dòng: chu trình chuyển FAULT và dừng băng tải.

### 4.2 Diễn tập đầu-cuối `scripts/rehearse_auto_cycle.py`
Thành phần thật: pipeline vision, ảnh trạm, chu trình, bộ tạo dòng `sorting_logs`. Thành phần giả lập: ESP32 và bảng `orders`.

| Tình huống | Kết quả | Sọt |
|---|---|---|
| Kiện tốt, có trong `orders` | PASS | 2 |
| Kiện tốt, không có trong `orders` | REJECT | 0 |
| Nhãn rách (ảnh trạm 04/11) | REJECT `label_tear` | 0 |
| Ảnh mờ | MANUAL_REVIEW | 0 |
| Vật không có nhãn | MANUAL_REVIEW | 0 |

Kết quả: **5/5 đúng**. Băng tải dừng ở cả 5 lần kiểm tra; cuối phiên ở trạng thái STOP với AUTO=0. Mỗi lần kiểm tra mất 0,7–0,9 s.

GUI chạy ẩn (offscreen) với camera phát lại và ESP32 giả lập: bấm **CHẠY TỰ ĐỘNG**, 3 kiện đều PASS, bấm dừng thì băng tải dừng và nút kiểm tra tay hoạt động lại.

### 4.3 Mô phỏng tem A7 `scripts/simulate_a7_label.py`
Cách dựng ảnh: lấy ảnh trạm thật, thu nhãn còn 0,705 lần, phần mặt thùng mà nhãn A6 từng che được thay bằng vân carton thật (lấy từ thùng trong dataset, màu chỉnh theo đèn trạm). Các lỗi được vẽ lên phần carton lộ ra.

| Ca (20 khung + 5 nhãn rách) | Đúng |
|---|---|
| Kiện tốt → PASS (QR, logo, viền) | 20/20 |
| Thủng lỗ 0,15–0,30 cạnh QR | 18/20: 1 ca bị loại dưới dạng `dent_or_edge_tear`, 1 ca sót. Cả hai là lỗ sát mép kiện |
| Móp/khuyết viền 0,8 cạnh QR | 20/20 |
| Rách nhãn | 5/5 |
| Logo A7: nhận đúng / nhận nhầm khi xóa logo | 20/20, điểm 0,17–0,24 (ngưỡng 0,12) / 0/20, tối đa 0,02 |

Lỗi đã sửa nhờ mô phỏng: một lỗ nằm trước nền cũng tối (đầu băng tải) bị mất khỏi mask kiện, nên bị bỏ qua. Bước tìm lỗ giờ quét trong đường viền ngoài đã lấp kín của kiện (`services/damage.py`).

Tem A6 hiện tại sau các sửa đổi: QR 20/20, logo 20/20, xóa logo 0/20 nhận nhầm, không báo hỏng nhầm 20/20.

## 5. Giới hạn còn lại
- **Chưa thử trên phần cứng thật:** ESP32, motor, cảm biến và Supabase thật. Cần chỉnh `auto_stop_delay_ms` sao cho kiện dừng giữa khung hình, và `stop_grace_s` phải ngắn hơn thời gian kiện đi qua cảm biến.
- **Tem A7 chỉ mô phỏng**, giả định bố cục là A6 thu nhỏ. Cần lấy mẫu logo trên tem thật và chạy lại `scripts/simulate_a7_label.py` hoặc kiểm tra thật.
- **Móp mặt trên không làm đổi đường viền** (lõm giữa nắp) chưa phát hiện được bằng ảnh 2D từ trên xuống. Hướng xử lý: DINOv2 so với thùng lành (`scripts/ml/parcel_anomaly.py`, `docs/ml_pretrained_eval.md`) khi đã có 20–50 ảnh thùng A7 thật. Công cụ này chạy offline, chưa đưa vào GUI.
- **Lỗ sát mép kiện** (cách mép dưới khoảng 0,12 cạnh QR) có thể bị coi là khuyết viền, hoặc bị sót nếu quá nông.
- **Ảnh nhãn chứa tên, địa chỉ, số điện thoại**: không đưa `captures/`, `dataset/station/` hay `tmp/` lên repo công khai.

## 6. Chế độ camera: webcam dừng kiện, cảm biến ở điểm gắp (2026-11-22)
Lý do: chỉ có một cảm biến quang, mà điểm gắp nằm ngoài khung hình (cách camera 1–2 s chạy). Chạy theo giờ (`conveyor_transfer_s`) bị lệch vì motor DC không có encoder, cộng thêm độ trễ lệnh. Vì vậy hai thiết bị đổi vai:

| Điểm dừng | Thiết bị | Độ chính xác cần |
|---|---|---|
| Kiểm tra | **Webcam**: tâm kiện tới giữa khung hình → PC gửi STOP | Chỉ cần kiện nằm trọn trong khung |
| Gắp | **Cảm biến quang** tại điểm gắp → ESP32 tự STOP (firmware v2, `AUTO:1:0`) | Vài mm, không phụ thuộc mạng |

```
FWD ─► camera: tâm kiện qua giữa khung (bù trễ latency_s) ─► STOP ─► kiểm tra + ghi Supabase
    ─► FWD ─► cảm biến điểm gắp: ESP32 tự STOP ─► robot gắp (ứng dụng tổng) ─► robot xong
    ─► (cảm biến phải trống) ─► FWD kiện sau
```

| Thành phần | File |
|---|---|
| Phát hiện kiện trên ảnh (so với nền trống, theo dõi tâm, bù trễ) | `services/arrival.py` (`ArrivalDetector`) |
| Chu trình hai điểm dừng | `services/station_cycle.py` (`CameraStationCycle`) |
| Nối camera và robot | `ui/auto_cycle.py` (`camera_arrival`, tín hiệu `parcel_at_pick`, `notify_pick_done`) |
| Robot gắp khi kiện tới cảm biến | `SortingStation/sorting_station/main_window.py`, `sorting_panel.py`, `sorting.py` (`start_cycle(..., at_pick=True)`) |
| Tham số | `station_cycle.trigger: "camera"`, `pick_stop_delay_ms`, `transfer_timeout_s`, `pick_timeout_s`, `station_cycle.arrival` |

An toàn và xử lý lệch:
- **Camera bỏ sót kiện**: kiện vẫn bị cảm biến chặn ở điểm gắp và được ghi **MANUAL_REVIEW**, robot đưa vào thùng LỖI. Không bao giờ dùng kết quả của kiện khác.
- **Kiện đặt quá sát nhau**: khoảng cách giữa hai kiện nhỏ hơn quãng camera → điểm gắp thì kiện sau bị cuốn qua camera và xử lý như trên. **Kiện phải cách nhau xa hơn quãng camera → điểm gắp.**
- **Robot báo xong mà cảm biến vẫn thấy kiện** (hút trượt): FAULT, băng tải không chạy.
- **Robot không chạy được** (chưa lưu gốc, lập kế hoạch lỗi): chu trình dừng.
- **Không bật robot tự động**: chu trình chờ người lấy kiện khỏi cảm biến rồi mới chạy tiếp.
- **Kiện không tới cảm biến** trong `transfer_timeout_s`, hoặc không được gắp trong `pick_timeout_s`: FAULT và STOP.
- **Đầu chu trình đã có kiện nằm ở cảm biến**: FAULT; lấy kiện ra rồi chạy lại.

### Kiểm thử (không có phần cứng)
- **`pytest` 85/85.** Trong đó có:
  - 4 test bộ phát hiện kiện: hai chiều chạy, bù trễ, hạt chói nhỏ, tự bật lại.
  - 6 test chu trình hai điểm dừng với ESP32 giả lập qua UDP: 3 kiện liên tiếp, camera bỏ sót, không có robot, quá hạn gắp, kiện đặt sát, hút trượt.
- **`scripts/rehearse_camera_cycle.py`**:
  - Kiện thật cắt từ ảnh trạm, di chuyển trên ảnh nền thật ở 30 khung/s; pipeline thật.
  - **3/3 đúng**: kiện có trong đơn → PASS sọt 2; không có trong đơn → REJECT; đặt quá sát → MANUAL_REVIEW.
  - Kiện dừng dưới camera lệch tâm +14 / +19 px (khoảng 1,5–2 mm). Dừng đúng tại cảm biến điểm gắp cả 3 lần.
- **Ứng dụng tổng chạy ẩn** (camera mô phỏng, ESP32 giả lập, robot mô phỏng, bật tự động): 2 kiện PASS → robot gắp vào thùng DONG_BANG → băng tải chỉ chạy tiếp sau khi robot xong.

### Khi lắp thật
1. Dời cảm biến quang tới điểm gắp. **Chụp lại ảnh nền băng tải trống**, vì cảm biến không còn trong khung hình.
2. Kiểm tra `station_cycle.arrival.roi` phủ đúng vùng băng tải trong ảnh và `axis` đúng chiều chạy (`"y"` là dọc ảnh).
3. Đo trễ dừng thực tế: chạy vài kiện, xem độ lệch tâm (ảnh kiểm tra), rồi chỉnh `latency_s`.
4. Chỉnh vị trí lắp cảm biến, hoặc `pick_stop_delay_ms`, sao cho kiện dừng đúng `pick_xyz_mm` của robot.
5. Có thể bỏ `sorting.conveyor_transfer_s` trong `SortingStation/config/station.yaml`: ở chế độ camera, giá trị này không còn được dùng.

## 7. Chỉ xét QR (2026-11-23)
Yêu cầu của nhóm, thay cho các mục kiểm tra cũ:
- Kiện chỉ cần **đọc được QR** → tra bảng `orders` trên Supabase → robot gắp vào sọt của đơn.
- **QR rách hoặc mờ đến mức khó quét** → vào sọt lỗi: chạy qua cảm biến, rơi khỏi cuối băng tải.
- Rách tem ở chỗ khác, logo, móp hay thủng thùng: **không xét**.

| Quy tắc | Cấu hình |
|---|---|
| Tắt kiểm tra logo | `logo.enabled: false`, `classification.require_logo: false` |
| Tắt kiểm tra rách viền tem và bề mặt thùng | `damage.enabled: false`, `classification.require_damage_check: false` |
| QR "khó quét": cùng mã phải đọc được ở **≥ 50%** khung trong loạt ảnh | `classification.min_qr_read_ratio: 0.5` (`ParcelData.qr_read_ratio`) |
| Không đọc được QR ở mọi khung (sau 1 lần chụp lại) | REJECT "Không đọc được mã QR (tem mờ, rách hoặc bị che)" (`station_cycle.py`) |
| Khung nét nhất không đọc được → thử thêm 2 khung | `qr.extra_frame_attempts: 2` |
| Không phóng to khung 2K khi đọc QR (vùng cắt nhỏ vẫn phóng) | `qr.max_upscaled_pixels: 12000000`, `qr.try_inverted: false` |

Muốn bật lại các kiểm tra cũ thì đặt lại các giá trị trên. Mã nguồn kiểm tra rách viền tem vẫn được giữ: khía nằm trong vùng chữ in hay mã vạch (gồ ghề, xám) không bị tính là vết rách, nên tem mới có mã vạch sát mép không còn bị báo rách nhầm.

### Tem mới (QR 28 mm, 21 ô) trên camera hiện tại
Tem cũ có QR 20 mm (khoảng 175 px trên ảnh, 6 px/ô) mờ quá, không đọc được 0/12 khung; khử mờ bằng phần mềm cũng không cứu được. Tem mới QR 28 mm đọc **12/12 khung**.

Kết quả trên ảnh trạm và các ca giả lập:

| Ca | Kết quả | Thời gian |
|---|---|---|
| Tem mới, QR nguyên | PASS, đọc 100% khung | 0,5 s |
| QR rách 25% / 35% cạnh | PASS, đọc 100% khung (QR tự sửa lỗi) | 0,5 s |
| QR rách 45% / 60% cạnh | REJECT, không đọc được | 2,2–2,5 s mỗi loạt |
| QR mờ σ 2 / σ 4 | PASS | 0,5 s |
| QR mờ σ 6 | REJECT | 2,5 s mỗi loạt |
| QR chập chờn (đọc được 2/6 khung) | REJECT "chỉ đọc được 33% số khung" | 1,1 s |
| Tem cũ QR nhỏ, mờ | REJECT | 2,5 s mỗi loạt |
| Tem rách viền, QR nguyên | PASS (không xét rách viền) | 0,5 s |
| Băng tải trống | REJECT | 2,4 s mỗi loạt |

Với kiện không đọc được, chu trình chụp lại 1 lần (`quality_retries: 1`), tổng khoảng 5 s rồi mới đẩy kiện ra.

### Lấy mẫu tại trạm (2026-11-23, tem QR 28 mm, ảnh nền mới sau khi dời cảm biến)
| Mẫu | Thư mục `dataset/station/` | Camera: vị trí kiện trong khung | Dừng | QR |
|---|---|---|---|---|
| Băng tải trống (nền mới) | `empty_belt_sensor_moved_20261123T170053` | — | không kích hoạt (12/12; cả khi ánh sáng ×0,6 / ×1,6) | — |
| Cỡ 1 | `good_parcel_qr28_pos2_20261123T171012` | 0,09–0,55 | khung thứ 2 | 12/12, PASS |
| Cỡ 2 (đặt lấn mép trên) | `good_parcel_qr28_size2_20261123T171131` | 0,00–0,58 | không dừng (đúng: chưa trọn khung) | 12/12 |
| Cỡ 2 | `good_parcel_qr28_size2_full_20261123T171540` | 0,13–0,78 | khung thứ 2 | 12/12, 10,5 px/ô, PASS |
| Cỡ 3 (lớn nhất) | `good_parcel_qr28_size3_20261123T172450` | 0,15–0,94 | khung thứ 2 | 12/12, 9,5 px/ô, PASS |

Ghi chú:
- Kiện lớn nhất chiếm khoảng 79% chiều dọc khung hình. Khi băng tải chạy, quãng kiện nằm trọn khung chỉ còn khoảng 17% khung hình, nên không được chạy băng tải quá nhanh. Đo lại `latency_s` khi chạy thật.
- Giữa buổi chiều, camera hoặc tấm nền đã xê dịch khoảng 60 px. Mỗi lần như vậy phải bấm **CHỤP NỀN TRỐNG**; nút này nay dùng được cả ở chế độ băng tải.
- Ảnh nền cũ được sao lưu ở `tmp/_review/backup_2026-11-23/`.

### Kiểm thử các ca lỗi do người vận hành đặt (2026-11-23, hộp kích thước ngẫu nhiên)
Mỗi ca được kiểm tra đúng như khi vận hành: camera phát hiện kiện nằm trọn khung → `inspect` → nếu không đọc được QR thì chụp lại 1 lần → kết luận.

| Ca | Thư mục `dataset/station/` | Camera dừng | QR đọc được | Kết luận | Thời gian |
|---|---|---|---|---|---|
| Hộp tốt cỡ 1 / 2 / 3 | `good_parcel_qr28_pos2_*`, `..._size2_full_*`, `..._size3_*` | khung thứ 2 | 12/12 | PASS → robot gắp | 0,5 s |
| QR rách + mờ | `bad_parcel_qr_torn_blur_20261123T173219` | khung thứ 2 | 0/12 | REJECT → sọt lỗi | 4,4 s |
| Không có QR (tem mất mảng có QR) | `bad_parcel_no_qr_20261123T173606` | khung thứ 2 | 0/12 | REJECT → sọt lỗi | 4,3 s |
| QR mờ (in nhòe) | `bad_parcel_qr_blur_20261123T175024` | khung thứ 3 | 0/12 | REJECT → sọt lỗi | 4,7 s |

**Thay đổi rút ra:** hộp mất QR có Tenengrad 255, thấp hơn ngưỡng cổng chất lượng 280, trong khi hộp tốt chỉ đạt 340–420, quá sát ngưỡng. Vì vậy `acquisition.quality_gate_blocks: false`: khung dưới ngưỡng nét vẫn được đem đọc QR, và việc đọc được QR mới quyết định kết luận.

## 8. Sửa lỗi camera dừng kiện không kịp (2026-11-23, sau lần chạy thật đầu tiên)
Hiện tượng: kiện tốt đặt lên băng tải thì camera không dừng kịp, kiện chạy tới cảm biến điểm gắp.

| Nguyên nhân | Sửa |
|---|---|
| Luồng camera chỉ ~12 khung/s vì đo độ nét ảnh 2K (67 ms) ngay trong vòng đọc ảnh | Tách luồng: đọc ảnh 29,9 khung/s; đo nét/vẽ GUI ở luồng riêng (`ui/camera_worker.py`) |
| Đầu băng tải nằm trong khung, kiện đặt tay ngay dưới camera; tay rút ra làm tâm khối nhảy → bị hiểu là kiện chạy rất nhanh → dự đoán lố → không dừng | Vận tốc đo từ **hai đầu kiện** (trung vị, bỏ đầu chạm mép) (`services/arrival.py`) |
| "Cơ hội cuối" giả định 30 khung/s | Dùng khoảng cách khung đo được; khung sau sẽ muộn thì dừng ngay |
| PWM 150 quá nhanh | Mặc định 40; chu trình tự động kẹp PWM trong `station_cycle.speed_range: [30, 50]` |
| Dừng lố do quán tính/trễ | Sau khi dừng đo lại trên ảnh mới; đầu kiện ra khỏi khung thì lùi REV 30 từng nhịp 0,15 s (tối đa 3 nhịp) |

Mô phỏng (kiện thật cắt từ ảnh trạm, chạy vào từ ngoài khung, trễ dừng 0,15 s): cỡ 0,65 và 0,70 khung, tốc độ 0,2–1,0 khung/s, 30 và 12 khung/s đều dừng trọn khung (tâm 0,51–0,58). Kiện đặt tay dưới camera, cỡ 0,67 khung: bộ cũ không ra lệnh dừng ở 0,56 khung/s (12 khung/s) và 0,33 khung/s (8 khung/s); bộ mới vẫn ra lệnh dừng ở hai trường hợp đó, lố 27 px và 5 px, phần lố được lùi chỉnh.

**Khi chạy thật**: mỗi lần bật tự động ghi `logs/arrival/arrival_<ngày_giờ>.csv` (mỗi khung: có kiện, trọn khung, hai đầu, vận tốc, khoảng cách khung, bắn). Gửi file này nếu kiện còn dừng lệch để chỉnh `latency_s`.

## 9. Dừng trễ ở điểm gắp: firmware v2.1 (2026-11-24)
Cảm biến quang đặt **trước** điểm gắp. Kiện chạy thêm khoảng 1–1,5 s sau khi cảm biến thấy mép trước rồi mới tới đúng chỗ robot gắp.

| Mục | Giá trị / hành vi |
|---|---|
| Trễ dừng | `station_cycle.pick_stop_delay_ms: 1200`, gửi xuống ESP32 qua `AUTO:1:1200`. Chỉnh không cần nạp lại firmware |
| Lúc dừng kiện có thể đã qua khỏi cảm biến | Firmware báo `ASTOP=1` trong STATUS; chu trình coi đó là "kiện đã ở điểm gắp" |
| Kiện thứ hai chạm cảm biến khi đang đếm trễ | Không làm lùi thời điểm dừng của kiện đầu |
| Không có robot, kiện dừng ngoài cảm biến | Cảm biến không thấy lúc người lấy kiện ra, nên băng tải chờ `manual_pick_s` (8 s) rồi chạy tiếp |
| Có robot, kiện dừng ngoài cảm biến | Không còn phát hiện được ca "hút trượt" bằng cảm biến (chỉ phát hiện được khi kiện còn che cảm biến) |
| Firmware v2.0 cũ | PC vẫn đọc được STATUS; nhưng v2.0 không có ASTOP nên kiện dừng ngoài cảm biến chỉ được nhận nếu PC kịp thấy cảm biến có vật |

## 10. Camera bị xê dịch: phải chụp lại ảnh nền (2026-11-24)
Bộ dò kiện so mọi khung với `config/calibration/empty_belt.png`. Mỗi lần camera, đèn hay băng tải bị dịch, phải chụp lại ảnh nền:
1. Lấy hết kiện khỏi vùng camera.
2. Bấm **CHỤP NỀN TRỐNG** trên GUI.

Nếu quên bước này, chu trình tự động báo FAULT sau 3 s với nội dung "Camera thấy cả khung hình khác ảnh nền băng tải trống…" và dừng băng tải. Trước bản sửa này, băng tải vẫn chạy nhưng không dừng kiện nào.

## 11. Mỗi kiện đều được dừng: bật lại theo kiện, học vùng đứng yên (2026-11-24)
| Trước | Bây giờ |
|---|---|
| Sau lần dừng đầu, bộ dò đợi băng tải **trống hoàn toàn** mới bật lại. Một vệt sáng hay bóng đứng yên cũng làm nó khóa luôn | Chu trình gọi `rearm()` trước mỗi kiện |
| Vùng khác ảnh nền mà nằm yên trọn khung cũng bị dừng | Chỉ vùng **đang di chuyển theo băng** mới được dừng |
| Vệt sáng, bóng hay ray đổi màu dính vào kiện làm sai hai đầu kiện | Vùng đứng yên bị bỏ qua sau 1 s và được học vào ảnh nền sau 4 s |
| Đèn tắt thì băng vẫn chạy nhưng không dừng kiện nào | Báo FAULT "Ánh sáng khác hẳn lúc chụp ảnh nền…" sau 2 s |

Lưu ý khi vận hành:
- Đặt kiện lên băng **đang chạy** rồi buông tay ngay. Kiện bị giữ yên hơn 1 s sẽ bị coi là vật đứng yên.
- Đèn vẫn phải bật như lúc chụp ảnh nền. Nếu đổi đèn hẳn: lấy hết kiện khỏi camera rồi bấm **CHỤP NỀN TRỐNG**.
- Log `logs/arrival/*.csv` có thêm các cột: `moved` (kiện đã được thấy di chuyển), `gain` (độ sáng so với ảnh nền), `static_ratio`, `absorbed`.

## 12. Ghi lại lần chạy, kiện dài, nút ĐÃ LẤY KIỆN (2026-11-25)
- **Ghi lại:** mỗi kiện khi chạy tự động được lưu ở `logs/runs/<giờ>/pNN/`, gồm ảnh lúc phát hiện, ảnh kiểm tra, `meta.json` và nhật ký các pha. Phát lại bằng:
  `python scripts/replay_station_runs.py logs/runs/<giờ>`
- **Bộ test ảnh thật:** `dataset/station_runs/` cùng `manifest.json`, được chạy trong `tests/test_station_runs.py`. Thêm ca mới: chép thư mục lần chạy vào đây rồi ghi kết quả mong đợi vào manifest.
- **Không có robot:** kiện PASS dừng ở điểm gắp, đã qua khỏi cảm biến (trễ 2200 ms). Băng **đứng chờ** tới khi bấm **ĐÃ LẤY KIỆN** (phím Space). Nếu bật lại `manual_pick_s > 0` thì băng chạy tiếp sau ngần ấy giây.
- **Kiện dài hơn khung** (ví dụ đặt chéo): camera dừng lúc đầu kiện sắp ra khỏi khung, khi phần lớn kiện đang trong khung. Nên đặt kiện thẳng theo băng để kiện nằm trọn khung.
- **Vệt phản chiếu đèn** trên băng không bị coi là kiện: vùng phải có trung vị L >= 100 hoặc độ chi tiết >= 30.

## 13. Kiện sát nhau: chạy lùi đưa về camera; PWM cố định 40 (2026-11-25)
- Chạy tự động luôn ở **PWM 40** (`station_cycle.speed`). Thanh trượt trên GUI chỉ còn tác dụng với nút chạy tay.
- Trong lúc kiện trước được chuyển tới điểm gắp, camera không dừng băng. Kiện đặt quá sát phía sau sẽ bị cuốn qua vùng camera.
- Khi kiện đó tới điểm gắp mà chưa được kiểm tra, băng **chạy lùi** đưa kiện về dưới camera, kiểm tra, rồi chạy tiếp như bình thường. Thời gian lùi tối đa là `backup_max_s` giây.
- Chu trình chỉ chạy lùi khi vùng camera đang trống. Vì vậy **không đặt kiện mới khi GUI đang báo "chạy lùi"**.

