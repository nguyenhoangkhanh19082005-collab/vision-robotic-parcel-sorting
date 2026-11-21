# Bàn giao MachineVision cho nhóm — 05/10/2026

## Các file dùng cho lần bàn giao này

- `MachineVision_Core_Source_2026-10-05.zip`: mã Vision, GUI, Digital Twin, cấu hình, logo đã lấy mẫu, firmware ESP32 gốc, schematic, tests, scripts, README và RESOURCES.
- `MachineVision_Private_Calibration_2026-10-05.zip`: ảnh nền bench/băng tải và hiệu chuẩn thực tế; chỉ lưu trong Drive riêng của nhóm.
- `RESOURCES.md`: lý thuyết, phương pháp, thuật toán, trạng thái từng phần, kết quả test và các giới hạn chưa nghiệm thu.

Gói core loại môi trường Python, cache, các bản build/EXE/ZIP cũ, dữ liệu raw, ảnh vận đơn, log và khóa Supabase. Bản gốc và dữ liệu thử tại workspace vẫn được giữ để có thể phục hồi hoặc đánh giá sau. Firmware gốc không có watchdog tự dừng motor khi mất mạng; đọc `docs/esp32_contract.md` trước khi thử phần cứng.

## Upload GitHub của nhóm

1. Giải nén **gói Core Source** vào một thư mục mới.
2. Tạo repository private của nhóm, hoặc clone repository đã được nhóm trưởng cung cấp. Không tạo repository trùng nếu nhóm đã có repo.
3. Với repo trống, mở PowerShell ở thư mục `MachineVision` vừa giải nén:

```powershell
git init -b main
git add .
git status
git commit -m "Vision GUI, Digital Twin and manual ESP32 UDP integration"
git remote add origin https://github.com/TEN_NHOM/TEN_REPOSITORY.git
git push -u origin main
```

Thay URL bằng URL thật. Trước commit, kiểm tra `git status` chỉ có source/tài liệu, không có `.env`, ảnh nền, ảnh nhãn, môi trường Python hoặc EXE. Với repository đã có lịch sử, đưa các file core vào clone trên branch riêng rồi tạo Pull Request theo quy trình nhóm; không force-push hoặc ghi đè lịch sử.

GitHub giữ file sau giải nén; không cần commit file ZIP vào source repo. README và RESOURCES nằm ở root để nhóm trưởng đọc ngay.

## Upload Drive nhóm

1. Tạo thư mục mốc `MachineVision/2026-10-05/` trong Drive nhóm được giới hạn quyền.
2. Upload ZIP Core Source, ZIP Private Calibration và RESOURCES.md.
3. Nếu nhóm trưởng cần đánh giá trên ảnh thật, thêm dataset đã duyệt vào thư mục riêng; ảnh có tên/địa chỉ/điện thoại cần quyền truy cập nội bộ.
4. Ghi chú: phiên bản source 0.2.0 có tab BĂNG TẢI / ESP32; bộ test 47 bài; phần cứng ESP32/motor và độ chính xác vision trên băng tải chưa được nghiệm thu trong phiên bàn giao.

Không upload `.venv`, `.runtime`, cache hoặc khóa/tài khoản thật. Các EXE 29/09/2026 trong workspace là bản cũ và không có phần tích hợp mới.

## Nhóm trưởng chạy source để kiểm tra

Cài Python 3.11 hoặc 3.12 trên máy đích, giải nén Core Source. Nếu cần khôi phục hiệu chuẩn tại hiện trường, giải nén thêm Private Calibration đè vào cùng thư mục `MachineVision`. Khi chuyển camera/đèn sang vị trí khác phải chụp nền chuẩn lại.

```powershell
cd "DUONG_DAN\MachineVision"
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e ".[dev]"
python -m pytest -q -p no:cacheprovider --basetemp=build\pytest-temp
python -m machine_vision.app --config config\default.yaml
```

Nếu PowerShell chặn Activate.ps1, dùng trực tiếp `.\.venv\Scripts\python.exe` cho các lệnh trên.

Test băng tải: laptop cùng subnet với ESP32, mở tab BĂNG TẢI / ESP32, nhấn PING rồi DỪNG và đợi ACK trước khi cho chạy. Slider thay PWM cho lệnh tiếp theo; đổi slider không tự gửi lệnh. Cần STOP và ACK trước đảo chiều. Quan sát tại chỗ, chuẩn bị ngắt nguồn motor vật lý. GUI không tự nối QR hoặc phân loại với motor.
