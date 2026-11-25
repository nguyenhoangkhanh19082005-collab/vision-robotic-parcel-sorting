# Nạp firmware băng tải + cảm biến (v2.1)

Bản build ngày 24/11/2026:
- Board **DOIT ESP32 DEVKIT V1**, Arduino-ESP32 core 3.3.7.
- Dùng 24% flash, 6% RAM.
- SHA-256 của `export/conveyor_sensor.ino.merged.bin`: `9163989B2357CC68D81B67243E731EA4967501276CC68A4EAE14939312C67AB4` (bản trễ 2200 ms, build 24/11 17:57).
- Bản v2.0 cũ (dừng ngay khi cảm biến thấy kiện) lưu ở `.bak_2026-11-24/firmware/conveyor_sensor_v2.0.merged.bin`.

## Thay đổi của v2.1: dừng trễ ở điểm gắp
Cảm biến quang đặt **trước** điểm gắp. Từ lúc cảm biến thấy mép trước của kiện tới lúc kiện tới đúng điểm gắp mất khoảng 1–1,5 s, nên băng tải phải chạy thêm rồi mới dừng.

| Mục | v2.0 | v2.1 |
|---|---|---|
| Trễ dừng mặc định (`AUTO:1` không kèm số ms) | 0 ms | **2200 ms** (`DEFAULT_AUTO_DELAY_MS`; 1200 ms trước đó, người vận hành yêu cầu chạy thêm 1 s) |
| Kiện thứ hai chạm cảm biến khi đang đếm trễ | Đếm lại từ đầu, kiện đầu trôi qua điểm gắp | Bỏ qua, vẫn dừng theo kiện đầu |
| `STATUS` | `...:UP=<s>` | `...:UP=<s>:ASTOP=<0|1>`, trong đó 1 nghĩa là lần dừng gần nhất do AUTO |
| Đang đếm trễ mà nhận lệnh REV/STOP | Có thể dừng nhầm khi đang chạy lùi | Hủy lệnh dừng đang chờ |

Phần mềm PC gửi trễ thật qua `AUTO:1:<ms>` lấy từ `config/default.yaml` → `station_cycle.pick_stop_delay_ms` (đang đặt 2200). **Chỉnh vị trí dừng thì sửa số này rồi mở lại GUI, không cần nạp lại firmware.**

## Thông số đang nạp
| Mục | Giá trị |
|---|---|
| IP ESP32 / subnet | **192.168.2.100** / 255.255.255.0 |
| Cổng nhận lệnh / cổng đẩy sự kiện | UDP 8120 / UDP 8121 |
| W5500 | MOSI 23, MISO 19, SCK 18, CS 5, RST 4 |
| BTS7960 | RPWM 25, LPWM 26, R_EN 27, L_EN 14, R_IS 34, L_IS 35 |
| Cảm biến quang | GPIO 13, kích mức LOW (NPN, kiểu E18-D80NK) |
| Watchdog | Motor dừng nếu 3 s không nhận lệnh |

Muốn đổi IP, mức kích của cảm biến hay chân nối thì sửa đầu file `conveyor_sensor.ino` (dòng 31–70), rồi nạp bằng **Cách 1**.

## Trước khi nạp
1. Rút nguồn động lực của motor (nguồn cấp cho BTS7960), chỉ cắm cáp USB vào ESP32.
2. Đo ngõ ra cảm biến khi không có vật. Nếu đo được khoảng 5 V thì phải có cầu chia áp, vì chân GPIO của ESP32 chỉ chịu tối đa 3,6 V.

## Cách 1: Arduino IDE (nạp từ mã nguồn)
1. Mở `conveyor_sensor.ino` trong Arduino IDE.
2. Chọn **Tools → Board → esp32 → DOIT ESP32 DEVKIT V1**, rồi chọn cổng COM của ESP32.
3. Thư viện cần có: **Ethernet** của Arduino (Library Manager → "Ethernet").
4. Bấm **Upload**. Nếu IDE dừng ở dòng `Connecting....`, giữ nút **BOOT** trên ESP32 cho tới khi bắt đầu nạp.

## Cách 2: nạp thẳng file .bin (không cần biên dịch)
Chỉ dùng **một** file: `export/conveyor_sensor.ino.merged.bin`, nạp ở **địa chỉ 0x0**.

- **Trình duyệt** (Chrome/Edge): mở https://espressif.github.io/esptool-js/ → Connect → chọn cổng COM → Flash Address `0x0` → chọn file merged.bin → Program.
- **Dòng lệnh**: thay `COM5` bằng cổng của bạn.
  ```powershell
  & "$env:LOCALAPPDATA\Arduino15\packages\esp32\tools\esptool_py\4.5.1\esptool.exe" --chip esp32 --port COM5 --baud 460800 write_flash 0x0 "E:\UDMT\Webcamandconveyor\too\MachineVision\firmware\conveyor_sensor\export\conveyor_sensor.ino.merged.bin"
  ```

## Kiểm tra sau khi nạp
1. Mở **Serial Monitor** ở tốc độ 115200.
   - Khi khởi động phải thấy dòng báo trạng thái cảm biến `Cam bien GPIO13: …`.
   - Sau đó cứ mỗi 3 s có một dòng `[MONITOR …]`.
2. Đưa tay qua cảm biến. Serial phải in `EVT:SENSOR:1…`, bỏ tay ra thì in `EVT:SENSOR:0…`.
3. Mạng: card Ethernet của laptop phải có một IP thuộc dải **192.168.2.x**. Hiện laptop chỉ có 192.168.1.100. Chạy lệnh dưới đây trong PowerShell **Administrator**; IP 192.168.1.100 của robot vẫn được giữ:
   ```powershell
   New-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress 192.168.2.10 -PrefixLength 24
   New-NetFirewallRule -DisplayName "ESP32 conveyor events" -Direction Inbound -Protocol UDP -LocalPort 8121 -Action Allow
   ping 192.168.2.100
   ```
4. Kiểm tra cảm biến qua mạng mà không cần GUI:
   ```powershell
   cd E:\UDMT\Webcamandconveyor\too\MachineVision
   .\.venv\Scripts\python.exe scripts\conveyor_sensor_monitor.py
   ```
5. Trong GUI: PING → DỪNG (đợi ACK) → cắm lại nguồn motor → CHẠY TIẾN ở PWM thấp.
6. Kiểm tra v2.1: Serial in `FW v2.1` khi khởi động. Dòng `[MONITOR …]` có thêm `auto ON 2200 ms`. `STATUS` trả về có đuôi `:ASTOP=0`.
7. Chỉnh vị trí dừng: chạy tự động một kiện PASS.
   - Kiện dừng **trước** điểm gắp thì tăng `pick_stop_delay_ms`, dừng **quá** thì giảm.
   - Ở PWM 40, mỗi 100 ms tương ứng một quãng ngắn. Nên chỉnh từng bước 100–200 ms.
