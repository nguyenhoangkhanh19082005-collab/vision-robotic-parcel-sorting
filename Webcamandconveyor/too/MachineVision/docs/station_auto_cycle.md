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
