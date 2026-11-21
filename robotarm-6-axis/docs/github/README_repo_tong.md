# Hệ thống phân loại kiện hàng: Vision + băng tải + cánh tay robot 6 trục

Repository gồm 3 phần. Giữ nguyên cấu trúc thư mục, vì SortingStation tìm 2 phần còn lại theo đường dẫn tương đối trong `SortingStation/config/station.yaml`.

| Thư mục | Nội dung |
|---|---|
| `Webcamandconveyor/too/MachineVision/` | **Parcel Vision**: webcam 2K đọc QR và đối chiếu Supabase, nhận logo Shopee Xpress, phát hiện rách nhãn, móp, thủng; GUI vận hành PyQt6; firmware ESP32 + W5500 điều khiển băng tải. Xem `README.md` và `HANDOFF_GITHUB_2026-11-21.md` trong thư mục này. |
| `robotarm-6-axis/` | **Cánh tay robot 6 trục**: firmware ESP32-S3 + TMC2209 (`robotarmver2/`), GUI điều khiển + digital twin (`main.py`), động học PoE lấy từ file CAD (`dong_hoc_step.py`, `kinematics_poe.py`), toàn bộ CAD in 3D (`assets/`, `src/cad/`), nhật ký thay đổi (`logs/CHANGELOG.md`). |
| `SortingStation/` | **Trạm hợp nhất**: một ứng dụng ghép Vision, băng tải và robot, có digital twin toàn trạm và bộ tuần tự gắp – thả vào 3 thùng theo vùng giao hàng. |

## Cài đặt (Windows, Python 3.12)

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r robotarm-6-axis\requirements.txt
.\.venv\Scripts\python.exe -m pip install -e "Webcamandconveyor\too\MachineVision[dev,database]"
```

Đặt `.venv` ở thư mục gốc repo: `SortingStation\run_station.bat` gọi `..\.venv`.

## Chạy

| Việc | Lệnh (từ thư mục gốc repo) |
|---|---|
| Trạm hợp nhất | `SortingStation\run_station.bat` |
| GUI robot riêng | `cd robotarm-6-axis` rồi `..\.venv\Scripts\python.exe main.py` |
| GUI Vision riêng | `cd Webcamandconveyor\too\MachineVision` rồi `..\..\..\.venv\Scripts\python.exe -m machine_vision.app --config config\default.yaml` |
| Test | `cd SortingStation` rồi `..\.venv\Scripts\python.exe -m pytest -q` (22 test); MachineVision: `python -m pytest -q` (75 test) |

## Không có trong repo

| Thứ | Lý do |
|---|---|
| `.env` (khóa Supabase) | Bảo mật. Tạo từ `.env.example`. |
| Ảnh kiện, ảnh nhãn, dataset | Có họ tên, địa chỉ, số điện thoại người nhận. |
| Trọng số model Hugging Face | Tải lại được. |
| File lắp ráp STEP trung gian 70–76 MB | Vượt giới hạn file của GitHub. |

Lắp ráp robot cuối cùng có ở dạng nén: `robotarm-6-axis/assets/CAD_lap_rap_v3/Roboter_v3_J4_cu_bau_ngan_giac_hut_dai.step.zip`.
