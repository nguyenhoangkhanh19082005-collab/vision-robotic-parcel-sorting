# Bàn giao để đóng gói MachineVision lên GitHub (2026-11-21)

> **Dành cho phiên chat/agent nhận file này.**
> - Nhiệm vụ là đưa phần việc Vision + GUI + Digital Twin + firmware băng tải của dự án *Parcel Vision System* lên GitHub, để nhóm trưởng viết báo cáo tổng hợp.
> - Thư mục gốc là `E:\UDMT\Webcamandconveyor\too\MachineVision`. Đây **chưa phải git repository**.
> - Trả lời người dùng bằng tiếng Việt.
> - **Hỏi người dùng trước khi push**: URL repository của nhóm, repo public hay private, đẩy lên nhánh nào.
> - Không tự tạo repository mới nếu nhóm đã có repo.
> - File `E:\UDMT\CLAUDE.md` thuộc một dự án khác (cánh tay robot 6 trục), **không áp dụng** cho thư mục này.

## 1. Hệ thống làm gì
Trạm phân loại kiện hàng trên băng tải, dùng webcam 2K (Stafor, 2560×1440) đặt nhìn từ trên xuống, đèn livestream và ESP32 + W5500 điều khiển băng tải. Luồng hoạt động:

1. Cảm biến quang thấy kiện → ESP32 dừng băng tải.
2. Vision kiểm tra:
   - mã QR, đối chiếu với bảng `orders` trên Supabase;
   - logo Shopee Xpress;
   - bề mặt: rách nhãn, móp hoặc khuyết viền, thủng lỗ.
3. Ghi một dòng vào bảng `sorting_logs` trên Supabase: kiện PASS vào sọt của đơn hàng, kiện lỗi hoặc không xác định vào sọt loại (0).
4. Băng tải chạy tiếp tới điểm gắp của robot.

| Phần | Vị trí | Ghi chú |
|---|---|---|
| Pipeline vision | `src/machine_vision/services/`, `readers/`, `imaging/` | Ảnh được nắn theo QR (phép đồng dạng); QR đọc bằng zxing-cpp; logo khớp bằng SIFT + RANSAC; rách nhãn đo khía viền giấy theo cạnh QR; kiện tách khỏi ảnh nền trống |
| Chu trình tự động (mới) | `services/station_cycle.py`, `controllers/conveyor_udp.py`, `ui/auto_cycle.py` | Nút **CHẠY TỰ ĐỘNG** trong tab BĂNG TẢI / ESP32 |
| GUI vận hành (PyQt6) | `src/machine_vision/ui/main_window.py` và các file cùng thư mục | Gồm tab VẬN HÀNH, DIGITAL TWIN 3D, BĂNG TẢI / ESP32 |
| Digital Twin | `ui/digital_twin.py`, `ui/qml/DigitalTwinView.qml`, `assets/digital_twin/scene_manifest.json` | Cảnh dựng bằng phần mềm, robot 6 trục chỉ là mô phỏng |
| Supabase | `repositories/shipments.py`, `repositories/sorting_logs.py`, `docs/supabase_bo_sung_2026-11-10.sql`, `docs/HUONG_DAN_UPLOAD_TRANG_THAI.md` | Mặc định tắt; khóa đọc từ `.env` |
| Firmware ESP32 | `firmware/conveyor_sensor/conveyor_sensor.ino` (v2), `firmware/sketch_sep19a.ino` (bản gốc) | Giao thức mô tả ở `docs/esp32_contract.md` |
| Sơ đồ mạch | `hardware/esp32_conveyor_schematic.png` | |
| Công cụ ML (offline) | `scripts/ml/parcel_anomaly.py` | Dùng DINOv2 / SAM2 / Grounding DINO, chạy trong `.venv-ml` riêng. **Chưa nối vào GUI** |
| Cấu hình | `config/default.yaml` | Chứa `label_format: A6/A7`, `station_cycle`, các ngưỡng |
| Tài liệu | `README.md`, `RESOURCES.md`, `docs/*.md` | Xem mục 6 |

Phiên bản mã nguồn **0.3.0**, gồm khoảng 10.000 dòng Python trong 75 file `.py` (`src`, `tests`, `scripts`).

## 2. Trạng thái và kết quả kiểm thử (để đưa vào báo cáo)
**Test tự động:**
- `pytest`: **75/75 đạt** (2026-11-21).
- Chạy lại bằng: `.\.venv\Scripts\python.exe -m pytest -q`.

**Diễn tập chu trình tự động** (`scripts/rehearse_auto_cycle.py`):
- Dùng ESP32 giả lập qua UDP thật trên localhost, ảnh trạm thật, pipeline thật và bảng `orders` giả.
- **5/5 tình huống đúng**: kiện tốt có trong DB → PASS sọt 2; kiện tốt không có trong DB → REJECT; nhãn rách → REJECT; ảnh mờ → MANUAL_REVIEW; vật không nhãn → MANUAL_REVIEW.
- Mỗi lần kiểm tra mất 0,7–0,9 s.
- GUI chạy ẩn với camera phát lại: 3 kiện tự động đều PASS, dừng chu trình an toàn.

**Trên kiện thật ở trạm, tem A6** (20 khung): QR 20/20, logo 20/20, xóa logo thì 0/20 nhận nhầm, không báo hỏng nhầm 20/20.

**Mô phỏng tem A7** (`scripts/simulate_a7_label.py`, chưa có tem thật):

| Ca | Kết quả |
|---|---|
| Kiện tốt | 20/20 PASS |
| Thủng lỗ | 18/20 (2 ca lỗ sát mép kiện: 1 ca vẫn bị loại, 1 ca sót) |
| Móp viền | 20/20 |
| Rách nhãn | 5/5 |
| Logo | 20/20; khi xóa logo thì 0/20 nhận nhầm |

**Đánh giá model Hugging Face** (`docs/ml_pretrained_eval.md`):
- Chế độ chỉ chấm carton: 0/12 báo nhầm, phát hiện 36/36 lỗi tổng hợp.
- Không dùng được để chấm vùng nhãn, vì nhãn khác nội dung bị báo lỗi.

**Chưa nghiệm thu:**
- Phần cứng thật: ESP32, motor, cảm biến.
- Supabase thật.
- Tem A7 thật.
- Móp giữa nắp mà không làm đổi đường viền.

Chi tiết: `docs/station_auto_cycle.md`.

## 3. Đưa lên / KHÔNG đưa lên
`.gitignore` đã được cập nhật cho các mục dưới đây. **Vẫn phải kiểm tra lại bằng `git status` trước khi commit.**

**Đưa lên** (dung lượng nhỏ, khoảng 1,3 MB):
- `src/`, `tests/`, `scripts/` (gồm `scripts/ml/`), `docs/`, `firmware/`, `hardware/`
- `config/default.yaml`, `config/calibration/logo_calibration*.json`, `config/calibration/.gitkeep`
- `assets/logo/spx_template.png`, `spx_template_a7.png` (mẫu tạm), `spx_stacked_template.png`, `spx_template_40cm.png`; `assets/digital_twin/scene_manifest.json`
- `README.md`, `RESOURCES.md`, `pyproject.toml`, `.gitignore`, `.env.example`, `camera_diag.py`
- File bàn giao này (tùy nhóm; có thể để trong `docs/`)

**KHÔNG đưa lên:**

| Đường dẫn | Lý do |
|---|---|
| `.env` | Khóa Supabase |
| `.venv/`, `.venv-ml/` (1,1 GB), `.runtime/`, `__pycache__/`, `.pytest_cache/`, `.ruff_cache/` | Môi trường và cache |
| `models/hf/` (~0,9 GB), `models/*.npz` | Trọng số tải từ Hugging Face; tải lại được |
| `tmp/` (3,3 GB) | Ảnh thử, bản sao lưu, kết quả phân tích |
| `captures/`, `dataset/station/` (133 MB), `dataset/raw/`, `dataset/analysis/` | **Ảnh nhãn có họ tên, địa chỉ, số điện thoại người nhận**: dữ liệu cá nhân |
| `config/calibration/*.png` | Ảnh nền riêng của trạm; nếu cần thì chia sẻ qua Drive nội bộ |
| `assets/logo/*_backup_*.png` | Bản sao lưu |
| `exports/` | Các file EXE/ZIP cũ ngày 29/09 và 05/10; đã lỗi thời |
| `build/`, `work/` | Thư mục build, file nháp |
| `E:\UDMT\Webcamandconveyor\too\work\` (27 MB) | Ảnh chụp UI và ảnh cắt nhãn. Ảnh chụp UI có thể dùng cho báo cáo, **cần người dùng chọn**; ảnh cắt nhãn có thông tin cá nhân |

**Lưu ý thêm:**
- `docs/HUONG_DAN_UPLOAD_TRANG_THAI.md` có **URL project Supabase**, không có khóa. Nếu repo để public, hỏi người dùng có muốn thay bằng URL mẫu không.
- Dù để public hay private, phải bật RLS cho các bảng.
- Báo cáo đánh giá nằm ngoài thư mục dự án: `E:\UDMT\Webcamandconveyor\review\danh_gia_vision_2026-10-28.md`. Hỏi người dùng có muốn chép vào `docs/` không.

## 4. Các bước gợi ý
```powershell
cd E:\UDMT\Webcamandconveyor\too\MachineVision
git init -b main          # hoặc clone repo nhóm rồi chép source vào, làm trên branch riêng
git add .
git status                # rà: không có .env, ảnh nhãn, tmp/, .venv*, models/hf, exports/
git count-objects -vH     # sau commit: kích thước phải cỡ vài MB
```

**Rà soát trước khi push:**
1. Quét khóa: `git grep -nE "eyJ[A-Za-z0-9_-]{20,}|sb_secret|service_role"` phải không trả về gì.
2. Không có file ảnh nào trong `git ls-files` ngoài `assets/logo/*.png` và `hardware/*.png`.
3. Không file nào vượt 5 MB.
4. Chạy `pytest` trên một bản clone sạch. Các test tự sinh ảnh, không cần dữ liệu ngoài repo.

**Gợi ý chia commit**, nếu muốn lịch sử dễ đọc:
1. Mã nguồn vision, GUI, Digital Twin, kèm test.
2. Firmware v2, giao thức, chu trình tự động.
3. Hỗ trợ tem A7 và các script mô phỏng/diễn tập.
4. Tài liệu (README, docs, báo cáo đánh giá).

**Với repo nhóm đã có lịch sử:** tạo branch, ví dụ `vision-station`, rồi mở Pull Request. **Không force-push**.

## 5. Nhóm trưởng chạy thử sau khi clone
```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[dev,database]"
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe scripts\rehearse_auto_cycle.py   # cần ảnh trạm (không có trong repo)
.\.venv\Scripts\python.exe -m machine_vision.app --config config\default.yaml
```

- **Không có ảnh nền** (`config/calibration/empty_belt.png`): GUI báo "Commissioning required". Cần chụp lại bằng `scripts/capture_empty_belt_reference.py` tại trạm.
- **Script diễn tập** cần các thư mục `dataset/station/...` và `captures/...`, vốn không có trong repo. Có thể chia sẻ qua Drive nội bộ, hoặc chỉ ghi kết quả vào báo cáo.

## 6. Tài liệu cho người viết báo cáo
| Tài liệu | Nội dung |
|---|---|
| `README.md` | Tổng quan, cài đặt, vận hành, xử lý sự cố |
| `RESOURCES.md` | Lý thuyết, thuật toán, tài liệu tham khảo (cập nhật lần cuối 06/10, chưa có các phần mới) |
| `docs/station_auto_cycle.md` | **Mới**: chu trình tự động, tem A7, kết quả 21/11 |
| `docs/vision_redesign.md` | Thiết kế lại vision sau khi lắp trạm (rách nhãn, phân vùng kiện, QR) |
| `docs/ml_pretrained_eval.md` | Đánh giá SAM2 / Grounding DINO / DINOv2 |
| `docs/phase2_plan.md` | Kế hoạch giai đoạn 2: YOLO11n-seg, phân cỡ kiện |
| `docs/esp32_contract.md` | Giao thức UDP, firmware v2, chân kết nối, an toàn |
| `docs/HUONG_DAN_UPLOAD_TRANG_THAI.md`, `docs/supabase_bo_sung_2026-11-10.sql` | Ghi kết quả lên Supabase |
| `docs/architecture.md`, `docs/hmi_digital_twin.md`, `docs/3d_asset_handoff.md` | Kiến trúc, HMI, Digital Twin |

## 7. Thay đổi trong phiên 2026-11-10 → 11-21
- **Mới:**
  - `services/station_cycle.py`
  - `ui/auto_cycle.py`
  - `controllers/fake_esp32.py`
  - `tests/test_station_cycle.py` (5 test)
  - `scripts/rehearse_auto_cycle.py`
  - `scripts/simulate_a7_label.py`
  - `scripts/capture_station_samples.py`
  - `docs/station_auto_cycle.md`
  - `assets/logo/spx_template_a7.png`
  - `config/calibration/logo_calibration_a7.json`
- **Sửa:**
  - `controllers/conveyor_udp.py`: thêm `ConveyorLink`, `STATUS`, `AUTO`, sự kiện.
  - `ui/main_window.py`: thêm nút CHẠY TỰ ĐỘNG, tách hàm `_record_result`.
  - `config.py`: thêm `resolve_label_format`.
  - `app.py`
  - `services/damage.py`: gộp giấy nhãn vào mask kiện; tìm lỗ trong đường viền đã lấp kín.
  - `config/default.yaml`:
    - `bench_mode_default: false`
    - `min_template_sharpness 3.0`
    - thêm `station_cycle`
    - thêm `label_format`/`label_formats`
  - `README.md`, `.gitignore`
  - Phiên bản 0.3.0 (`pyproject.toml`, `__init__.py`)
- **Dữ liệu trạm (không đưa lên):**
  - Ảnh nền mới.
  - Mẫu logo mới, chụp trên kiện thật.
  - `dataset/station/`: 12 ảnh băng tải trống và 32 ảnh kiện tốt.
