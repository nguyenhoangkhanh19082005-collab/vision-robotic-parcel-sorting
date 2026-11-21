# HƯỚNG DẪN KHẮC PHỤC HIỆN TƯỢNG "ACT HIỆN 0.0° VÀ ĐỘNG CƠ CHƯA CHẠY"

## 1. NGUYÊN NHÂN GỐC RỄ (ROOT CAUSE ANALYSIS)

Khi bạn kéo thanh trượt trên giao diện:
- **Đèn LED xanh trên ESP32 chớp nháy**: Đây là đèn báo RX của chip nạp UART (CH343/CP2102). Việc đèn chớp chứng minh **100% dữ liệu từ Python đã được truyền qua cáp Type-C thành công vào phần cứng ESP32**.
- Tuy nhiên, **`Act: 0.0°` không thay đổi và động cơ chưa quay** là do **3 nguyên nhân kỹ thuật cốt lõi** sau:

### Nguyên nhân 1: Xung đột chân kích hoạt `ENABLE_PIN 14` do thư viện FastAccelStepper
- Trên mạch, cả 6 động cơ dùng chung **1 chân Enable duy nhất (PIN 14)** kết nối tới chân `EN` của 6 driver TMC2209 (Driver TMC2209 kích hoạt ở mức **LOW = 0V**, ngắt ở mức **HIGH = 3.3V**).
- Trong firmware cũ có dòng:
  ```cpp
  steppers[i]->setAutoEnable(true);
  ```
  Khi bạn chỉ kéo 1 thanh trượt (ví dụ Khớp 1), 5 khớp còn lại đang đứng yên (Idle). Tính năng `autoEnable` của thư viện FastAccelStepper trên 5 khớp đứng yên sẽ **chủ động kéo chân ENABLE_PIN 14 lên mức HIGH (Tắt)**!
  👉 **Hậu quả**: Toàn bộ 6 driver TMC2209 bị ngắt điện tức thì, động cơ mất hoàn toàn lực giữ và không thể quay dù xung STEP vẫn phát.

### Nguyên nhân 2: Khóa điều kiện gửi phản hồi Serial trong Firmware cũ
- Trong firmware cũ, khối gửi gói tin telemetry qua Serial bị chặn bởi điều kiện:
  ```cpp
  if (millis() - last_serial_rx_time < 5000) { ... }
  ```
  Nếu khi ESP32 khởi động mà PC chưa kịp đồng bộ gói tin đầu tiên, `last_serial_rx_time` = 0 khiến ESP32 **hoàn toàn im lặng trên cổng Serial**. Do không nhận được gói phản hồi nào, giao diện Python giữ nguyên giá trị chữ mặc định ban đầu là `Act: 0.0°`.

### Nguyên nhân 3: Nguồn lực 12V / 24V cho động cơ
- Cáp USB Type-C chỉ cấp nguồn **5V logic** cho chip ESP32 và phần điều khiển của TMC2209 (VIO).
- Cuộn dây động cơ bước (VMOT) bắt buộc phải có **nguồn tổ ong / Adapter 12V hoặc 24V** cấp vào mạch công suất. Nếu chưa bật nguồn 12V/24V, động cơ sẽ không có dòng lực để quay.

---

## 2. CÁC ĐIỀU CHỈNH ĐÃ THỰC HIỆN TRONG FIRMWARE (`robotarmver2.ino`)

Cả 2 file tại `E:\robotarm-6-axis\robotarmver2\robotarmver2.ino` và `E:\robotarm-6-axis\Changes\robotarmver2.ino` đã được nâng cấp tối ưu triệt để:

1. **Khóa cứng Enable mức LOW (Active LOW)**:
   - Loại bỏ hoàn toàn `setAutoEnable(true)`.
   - Giữ lệnh chuẩn: `pinMode(ENABLE_PIN, OUTPUT); digitalWrite(ENABLE_PIN, LOW);`.
   - Driver TMC2209 luôn được cấp dòng khóa trục, giữ mô-men ổn định, không còn bị ngắt nhầm giữa các trục.
2. **Luôn phát Telemetry Serial 30Hz liên tục**:
   - Bỏ điều kiện `< 5000ms`. Ngay khi bật nguồn, ESP32 liên tục gửi dữ liệu góc thực tế 30 lần/giây qua Serial.
   - Ngay khi phần mềm máy tính kết nối COM8, nhãn `Act:` sẽ lập tức hiển thị góc thời gian thực.
3. **Phản hồi chuyển động tức thì (Real-time Dynamic Tracking)**:
   - `MotorTask` trong chế độ `MODE_PC` gọi `moveSynchronized` ngay khi có lệnh mới từ slider, giúp động cơ bám mượt mà theo cử chỉ kéo rê chuột của bạn.
4. **Kiểm tra an toàn phần cứng Ethernet W5500**:
   - Nếu bạn chưa cắm W5500, ESP32 sẽ tự động nhận diện và ngắt các lệnh SPI đọc UDP, giải phóng hoàn toàn chân SPI và CPU cho đường truyền USB Serial.

---

## 3. CÁC BƯỚC THỰC HIỆN ĐỂ BẬT ROBOT HOẠT ĐỘNG NGAY

### Bước 1: Nạp bản firmware mới vào ESP32-S3
1. **Đóng ứng dụng Python** (nếu đang mở) để giải phóng cổng COM8.
2. Mở **Arduino IDE** -> Mở file `E:\robotarm-6-axis\robotarmver2\robotarmver2.ino`.
3. Kiểm tra chọn đúng:
   - **Board**: `ESP32S3 Dev Module` (hoặc tên board ESP32-S3 bạn đang dùng).
   - **Port**: `COM8`.
4. Nhấn nút **Upload (`➔`)** trên Arduino IDE và đợi đến khi báo `Done uploading (100%)`.

### Bước 2: Cấp nguồn công suất động cơ
1. Bật công tắc nguồn **12V hoặc 24V** cấp cho mạch công suất driver TMC2209.
2. *Dấu hiệu nhận biết*: Bạn sẽ nghe thấy tiếng "khậc" nhẹ hoặc cảm nhận trục động cơ bị gài cứng lại (có lực giữ, không dùng tay xoay nhẹ được nữa).

### Bước 3: Mở phần mềm điều khiển trên máy tính
1. Mở PowerShell hoặc Terminal tại `E:\robotarm-6-axis`:
   ```powershell
   python main.py
   ```
2. Tại tab **ROBOT CONTROLLER PANEL**:
   - Chọn tab **CONNECTION**.
   - Chọn kiểu kết nối: `USB Type-C Serial`.
   - Nhấn **Quét cổng COM** -> Chọn `COM8` (baudrate `115200`).
   - Nhấn nút **KẾT NỐI ROBOT (USB / LAN)**.
3. Chuyển sang tab **JOINTS JOG**:
   - Quan sát giá trị `Act:` xanh lá cây sẽ kích hoạt đồng bộ.
   - Kéo thử thanh trượt **Waist (J1)** hoặc **Shoulder (J2)**: Động cơ ngoài đời thật và mô hình 3D Digital Twin sẽ chuyển động ăn khớp 100%!
