# Giao thức băng tải ESP32/W5500 đang dùng

Nguồn: firmware nhóm cung cấp tại [`../firmware/sketch_sep19a.ino`](../firmware/sketch_sep19a.ino) và sơ đồ [`../hardware/esp32_conveyor_schematic.png`](../hardware/esp32_conveyor_schematic.png). Đây là **giao thức thực tế**, thay thế bản phác thảo TCP/JSON trước đây.

## Mạng và lệnh

- ESP32/W5500 IP tĩnh `192.168.2.100/24`, UDP port `8120`. Laptop cần IP cùng subnet (ví dụ `192.168.2.10/24`); không cần Internet hay Supabase.
- Mỗi lệnh là một datagram ASCII; phản hồi về đúng source IP/port của gói vừa nhận.

| GUI gửi | ESP32 trả | Ý nghĩa |
|---|---|---|
| `PING` | `PONG` | Kiểm tra kênh UDP, **không** báo trạng thái motor |
| `FWD:150` | `ACK:FWD:150 (State: FWD Speed: 150)` | Chạy thuận, PWM 150/255 |
| `REV:150` | `ACK:REV:150 (State: REV Speed: 150)` | Chạy nghịch |
| `STOP:0` | `ACK:STOP:0 (State: STOP Speed: 0)` | Ngắt enable và PWM |

GUI xác minh IP/port người trả và ACK trùng action/speed. Timeout mặc định 600 ms; trạng thái sau timeout là `UNKNOWN`, **không** suy diễn motor đã dừng. Các thao tác dùng worker QThread để GUI vẫn phản hồi. Nút STOP vẫn nhấn được khi chưa PING hoặc có lệnh đang chờ; phản hồi cũ bị bỏ qua theo thứ tự lệnh tại GUI. PING định kỳ 1 giây chỉ kiểm tra liên lạc; firmware gốc không có watchdog từ PING.

Sau PING đầu tiên, nhấn DỪNG và đợi ACK trước khi GUI cho chạy. Đảo chiều yêu cầu STOP đã ACK. Khi có request đang chờ, STOP được ưu tiên gửi ngay sau request đó (timeout tối đa 600 ms). Địa chỉ bị khóa sau lệnh chạy cho tới ACK STOP. Khi đóng ứng dụng đã gửi lệnh chạy, GUI thử STOP và báo nếu chưa có xác nhận. UDP gốc vẫn không bảo đảm thứ tự/giao nhận ở ESP32; đây là một hạn chế giao thức cần nâng cấp.

Digital Twin chỉ phản ánh **lệnh gần nhất được ACK** (FWD/REV/STOP), không có encoder hay công tắc phản hồi để chứng minh tốc độ/trạng thái cơ học thực. Camera bật/tắt và kết quả vision không tự phát lệnh motor. Chưa có điều khiển phân loại tự động.

## Đấu nối từ schematic

| Khối | Chân |
|---|---|
| W5500 SPI | MOSI 23, MISO 19, SCLK 18, CS 5, RST 4 |
| BTS7960 | RPWM 25, LPWM 26, R_EN 27, L_EN 14 |
| BTS7960 sensor | R_S 34, L_S 35 trên sơ đồ nhưng **firmware chưa đọc** |
| Nguồn | 24 V tới BTS7960/motor; LM2596 tạo 5 V cho ESP32/BTS7960 logic; W5500 3,3 V; GND chung |

Xác minh đúng điện áp module thực tế, công suất và cực tính trước khi cấp nguồn. Không lấy 24 V trực tiếp vào ESP32 hoặc W5500. PWM trong firmware là LEDC 5 kHz, 8 bit, dùng API Arduino-ESP32 core 3.x.

## Giới hạn an toàn bắt buộc xử lý trước khi vận hành không giám sát

Firmware gốc **không tự dừng khi mất gói/đứt cáp LAN/GUI treo**; khi motor đã chạy, lệnh cũ có thể tiếp tục. Nó cũng ACK lệnh lạ mà không báo lỗi. Vì vậy đây mới là điều khiển thử nghiệm có người giám sát, không phải bộ điều khiển an toàn. Cần:

1. Nút dừng khẩn/công tắc ngắt nguồn động lực vật lý, tiếp điểm an toàn phù hợp và bảo vệ dòng.
2. Watchdog trong ESP32: chỉ duy trì chạy khi nhận heartbeat/lệnh hợp lệ từ host được cho phép; timeout hoặc link down phải gọi `stopMotor()`.
3. Thêm trạng thái/sequence ID và cơ chế kiểm tra phản hồi motor bằng sensor/encoder nếu muốn gọi là telemetry thực; test mất nguồn, mất mạng, reset ESP32.
4. Không dùng `STOP` GUI làm E-stop. Không kích hoạt auto-routing/robot cho tới khi commissioning liên động đầy đủ.

## Cách kiểm tra an toàn

Thử UDP giả lập không có động cơ trước (`tests/test_conveyor_udp.py`). Khi dùng phần cứng: tháo cơ cấu tải hoặc giữ vùng nguy hiểm trống, chuẩn bị ngắt nguồn motor vật lý, PING trước, chọn PWM thấp, chỉ nhấn FWD/REV khi giám sát trực tiếp, STOP và quan sát motor dừng thật. Mất ACK → ngắt nguồn động lực bằng phần cứng và kiểm tra tại chỗ.

## Firmware v2.0: băng tải + cảm biến quang (2026-11-03)

Mã nguồn: [`../firmware/conveyor_sensor/conveyor_sensor.ino`](../firmware/conveyor_sensor/conveyor_sensor.ino). Bản gốc `sketch_sep19a.ino` được giữ nguyên. Đã biên dịch với Arduino-ESP32 core 3.3.7, board *DOIT ESP32 DEVKIT V1*: 24% flash, 6% RAM, không cảnh báo. **Chưa chạy thử trên phần cứng.**

### Chân theo sơ đồ "BĂNG TẢI VÀ CẢM BIẾN"
| Khối | Chân ESP32 |
|---|---|
| W5500 | MOSI 23, MISO 19, SCK 18, CS 5, RST 4 |
| BTS7960 | RPWM 25, LPWM 26, R_EN 27, L_EN 14; R_IS 34, L_IS 35 (đọc mV) |
| Cảm biến quang (net `Proximity`, nguồn 5 V) | GPIO 13, `INPUT_PULLUP`, lọc nhiễu 15 ms |

Mặc định là cảm biến NPN thường mở (kiểu E18-D80NK): ngõ ra xuống LOW khi có vật. Nếu cảm biến của bạn ngược lại (PNP), đặt `SENSOR_ACTIVE_LOW = false`.

> **Kiểm tra điện áp trước khi cắm vào ESP32:** cấp 5 V cho cảm biến, không có vật, đo ngõ ra so với GND. Nếu đo ra khoảng 5 V (cảm biến có điện trở kéo lên 5 V bên trong) thì phải thêm cầu chia áp, ví dụ 10 kΩ nối tiếp và 20 kΩ xuống GND, hoặc diode Schottky nối ngược chiều về chân ra. GPIO của ESP32 chỉ chịu tối đa 3,6 V.

### Giao thức
Bốn lệnh cũ giữ **nguyên định dạng**, nên GUI hiện tại không cần sửa: `PING`→`PONG`, `FWD:n`, `REV:n`, `STOP:0`→`ACK:… (State: … Speed: …)`.

| Lệnh mới (cổng 8120) | Phản hồi |
|---|---|
| `STATUS` | `STATUS:<FWD/REV/STOP>:<pwm>:SENSOR=<0/1>:COUNT=<n>:AUTO=<0/1>:DELAY=<ms>:ISR=<mV>:ISL=<mV>:UP=<s>` |
| `AUTO:<0/1>:<ms>` | `ACK:AUTO:…`. Khi bật, băng tải đang chạy FWD sẽ tự dừng `<ms>` sau khi cảm biến thấy kiện |
| `RESETCOUNT` | `ACK:RESETCOUNT` |
| Lệnh sai hoặc PWM ngoài 1–255 | `ERR:…`. Bản cũ trả ACK cả với lệnh sai |

Sự kiện được đẩy tới **cổng 8121** của IP đã gửi lệnh hợp lệ gần nhất:
- `EVT:SENSOR:<1 = có vật / 0 = hết vật>:<đếm>:<ms>`
- `EVT:AUTOSTOP:<đếm>`
- `EVT:WATCHDOG_STOP`
- `EVT:OVERCURRENT_STOP:<mV>`

### Cơ chế an toàn đã thêm (không thay thế nút dừng khẩn vật lý)
- **Watchdog 3 s:** motor đang chạy mà không nhận gói hợp lệ nào trong 3 s thì tự dừng. GUI ping mỗi 1 s và ngừng ping khi mất phản hồi.
- **Rút cáp LAN** khi đang chạy → dừng.
- **Đảo chiều trực tiếp** FWD↔REV: ngắt cả hai nhánh 150 ms trước khi đảo.
- **Quá dòng:** tắt mặc định (`OVERCURRENT_MV = 0`). Đọc giá trị `ISR`/`ISL` khi băng tải chạy có tải rồi mới đặt ngưỡng.
- Khi khởi động, motor được tắt **trước mọi thao tác khác**.

### Kiểm tra cảm biến không cần GUI
```
.\.venv\Scripts\python.exe scripts\conveyor_sensor_monitor.py
```
Script in `STATUS` mỗi khi trạng thái đổi, và in mọi `EVT:*`. Các gói `STATUS` cũng nuôi watchdog, nên không chạy song song với tab băng tải của GUI. Serial Monitor (115200 baud) in trạng thái mỗi 3 s. Khi chưa nhận được W5500, Serial vẫn in mỗi lần cảm biến đổi trạng thái (`EVT:SENSOR…`), nên có thể thử cảm biến chỉ với cáp USB.
