# Chạy MachineVision bằng file EXE trên Windows

## Yêu cầu máy đích

- Windows 10 hoặc Windows 11 64-bit;
- webcam USB/UVC;
- không cần cài Python;
- nên đặt ứng dụng trong thư mục người dùng có quyền ghi, ví dụ Desktop hoặc Documents.

## Chạy bản portable

1. Giải nén toàn bộ `MachineVision_Windows_Portable_2026-09-29.zip`.
2. Không chạy trực tiếp EXE từ bên trong file ZIP.
3. Mở thư mục vừa giải nén và chạy `MachineVision.exe`.
4. Nếu Windows SmartScreen xuất hiện, kiểm tra nguồn file rồi chọn **More info → Run anyway**.
5. Cho phép truy cập webcam nếu Windows yêu cầu.

Lần mở đầu có thể chậm hơn do bản one-file phải giải nén thư viện vào thư mục tạm.

## Nội dung thư mục portable

- `MachineVision.exe`: ứng dụng chính;
- `config/default.yaml`: cấu hình camera và thuật toán có thể chỉnh sửa;
- `assets/`: logo template, Digital Twin manifest và tài nguyên đi kèm;
- `README.md`: hướng dẫn nhanh.

Ảnh kết quả được ghi vào thư mục `captures` bên cạnh EXE. Không đặt ứng dụng trong
`Program Files` nếu tài khoản không có quyền ghi.

## Lỗi thường gặp

### Không mở được camera

- đóng Zoom, Teams hoặc phần mềm khác đang dùng webcam;
- kiểm tra `camera.index` trong `config/default.yaml`;
- thử đổi `camera.backend` từ `dshow` sang `msmf`;
- bật quyền camera trong Windows Settings → Privacy & security → Camera.

### Bị antivirus chặn

Bản EXE chưa được ký số nên SmartScreen hoặc antivirus có thể cảnh báo. Chỉ chạy file
đúng checksum SHA-256 được cung cấp cùng gói portable.

## Build lại từ mã nguồn

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e ".[build]"
python scripts\build_windows_exe.py
```
