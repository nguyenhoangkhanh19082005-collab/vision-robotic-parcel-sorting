# CẨM NANG HƯỚNG DẪN ĐIỀU KHIỂN CÁNH TAY ROBOT 6 BẬC QUA CỔNG USB TYPE-C SERIAL
> **Tài liệu chuyển giao kỹ thuật - Chế độ điều khiển kép Dual-Mode (USB Serial & Ethernet W5500)**  
> *Áp dụng trực tiếp cho phần cứng ESP32-S3 đã hàn chân và Giao diện RoboDK Workcell AURA-6*

---

## MỤC LỤC
1. [Nếu không dùng W5500 mà dùng USB thì sẽ ra sao?](#1-nếu-không-dùng-w5500-mà-dùng-usb-thì-sẽ-ra-sao)
2. [Bảng so sánh chi tiết: USB Type-C Serial vs Ethernet W5500](#2-bảng-so-sánh-chi-tiết-usb-type-c-serial-vs-ethernet-w5500)
3. [Kiến trúc Dual-Mode (Tự động nhận diện kênh)](#3-kiến-trúc-dual-mode-tự-động-nhận-diện-kênh)
4. [Sơ đồ đấu nối & Yêu cầu nguồn điện thực tế](#4-sơ-đồ-đấu-nối--yêu-cầu-nguồn-điện-thực-tế)
5. [Cách xác định cổng COM trên Windows](#5-cách-xác-định-cổng-com-trên-windows)
6. [Các bước kết nối và điều khiển từ Giao diện GUI](#6-các-bước-kết-nối-và-điều-khiển-từ-giao-diện-gui)
7. [Khắc phục sự cố thường gặp (Troubleshooting)](#7-khắc-phục-sự-cố-thường-gặp-troubleshooting)

---

## 1. NẾU KHÔNG DÙNG W5500 MÀ DÙNG USB THÌ SẼ RA SAO?

Khi bạn **chưa muốn dùng module Ethernet W5500** mà dùng trực tiếp **cổng USB Type-C** trên ESP32-S3 để kết nối với máy tính, hệ thống sẽ hoạt động **rất mượt mà, tiện lợi và ổn định**:

### Những điểm vượt trội khi dùng USB:
1. **Cắm là chạy (Plug & Play)**:
   - Không cần cắm dây mạng LAN vào laptop.
   - Không cần cấu hình IP tĩnh mạng LAN của máy tính (`192.168.1.100`), không sợ đụng dải IP với Wi-Fi gia đình hay công ty.
   - Rất thích hợp khi bạn đặt robot trên bàn làm việc (Desk Bench) để test từng động cơ, test hành trình và cân chỉnh góc quay ban đầu.
2. **Không cần bận tâm module W5500**:
   - Nếu bạn chưa mua hoặc chưa cắm module W5500 vào mạch, hệ thống vẫn hoạt động bình thường 100%.
   - **Các chân 10, 11, 12, 13 mà bạn đã hàn chết trên mạch KHÔNG CẦN THÁO BỎ**. Firmware mới đã được thiết kế để không xung đột.
3. **Băng thông và độ trễ**:
   - Gói tin điều khiển nhị phân của chúng ta chỉ nặng **30 bytes** (chuẩn Struct + CRC16).
   - Ở tốc độ Baudrate tiêu chuẩn **115200 baud**, thời gian truyền 30 bytes chỉ mất **khoảng 2.6 ms**.
   - Tần số làm tươi vòng lặp là 30Hz (33 ms) đến 50Hz (20 ms), do đó độ trễ là **gần như tức thì (Zero-Lag)**, mắt người không thể nhận ra sự khác biệt so với Ethernet.

---

## 2. BẢNG SO SÁNH CHI TIẾT: USB TYPE-C SERIAL VS ETHERNET W5500

| Tiêu chí | Cổng USB Type-C Serial | Mạng LAN Ethernet W5500 |
| :--- | :--- | :--- |
| **Độ phức tạp cài đặt** | **Cực kỳ đơn giản** (Cắm cáp USB vào máy tính là nhận) | **Cần cấu hình** (Đặt IP tĩnh máy tính `192.168.1.100`, kết nối qua switch/dây mạng) |
| **Giao thức truyền** | UART Serial Stream (Baud 115200 - 921600) | UDP Socket (Port 8888, IP 192.168.1.177) |
| **Chuẩn gói tin** | **30 bytes Struct + CRC16** (Giống hệt nhau) | **30 bytes Struct + CRC16** (Giống hệt nhau) |
| **Khoảng cách dây dẫn** | 1 mét - 3 mét (Dây cáp USB) | 50 mét - 100 mét (Cáp mạng Cat5e/Cat6) |
| **Chống nhiễu công nghiệp** | Trung bình (Dễ bị spike từ motor nếu dùng cáp USB dỏm) | Rất cao (Có biến áp xung cách ly trên jack RJ45) |
| **Độ trễ truyền gói tin** | ~2.6 ms (@115200 baud) | ~0.5 ms |
| **Ứng dụng tối ưu** | **R&D, Thử nghiệm, Cân chỉnh bàn Lab** | **Triển khai xưởng công nghiệp lâu dài** |

> **Khuyến nghị**: Trong giai đoạn kiểm tra, thử nghiệm và chạy thử cánh tay ngoài đời, **hãy sử dụng cổng USB Type-C**. Khi mọi chuyển động đã chuẩn xác 100% và bạn muốn lắp robot vào dây chuyền cố định, bạn chỉ việc cắm thêm module W5500 và cắm dây mạng LAN là xong!

---

## 3. KIẾN TRÚC DUAL-MODE (TỰ ĐỘNG NHẬN DIỆN KÊNH)

Chúng tôi đã nâng cấp đồng bộ cả Firmware ESP32-S3 (`robotarmver2.ino`) và Phần mềm máy tính (`communication.py`, `main.py`) lên kiến trúc **Dual-Mode**:

```
                     +----------------------------+
                     |    Giao diện RoboDK GUI    |
                     |         (main.py)          |
                     +--------------+-------------+
                                    |
                    +---------------+---------------+
                    | (Tùy chọn kết nối thông minh) |
                    v                               v
         [Chế độ USB Serial]              [Chế độ W5500 LAN]
            (Cổng COMx)                     (IP 192.168.1.177)
                    |                               |
              Cáp USB Type-C                   Cáp mạng RJ45
                    |                               |
                    +---------------+---------------+
                                    v
                     +----------------------------+
                     |      ESP32-S3 Firmware     |
                     |     (robotarmver2.ino)     |
                     |   Lắng nghe đồng thời cả   |
                     |   USB Serial & W5500 UDP   |
                     +----------------------------+
```

### Điểm đặc biệt của kiến trúc này:
1. **Dùng chung 1 bản Firmware duy nhất**: Bạn nạp file `robotarmver2.ino` vào ESP32-S3 **một lần duy nhất**.
2. **Tự động phản hồi đúng kênh**:
   - Nếu máy tính gửi lệnh qua **USB Type-C (COM)**: ESP32 nhận lệnh và gửi dữ liệu phản hồi góc thực tế (Telemetry) ngược lại cổng USB Serial ở tần số 30Hz.
   - Nếu máy tính gửi lệnh qua **Cáp mạng LAN (UDP)**: ESP32 nhận lệnh và gửi phản hồi qua cổng mạng UDP ở tần số 30Hz.
3. **Chống nhiễu & Sai lệch nhịp tuyệt đối**:
   - Dữ liệu Serial được kiểm tra Magic Header `0x55, 0xAA` và mã **CRC16 chuẩn công nghiệp**.
   - Nếu có byte rác do cắm rút cáp hoặc log in ra từ ESP32, bộ lọc của chúng tôi sẽ tự động loại bỏ rác và chỉ nhận đúng frame 30-bytes nguyên vẹn.

---

## 4. SƠ ĐỒ ĐẤU NỐI & YÊU CẦU NGUỒN ĐIỆN THỰC TẾ

> [!CAUTION]
> **LƯU Ý CỰC KỲ QUAN TRỌNG VỀ NGUỒN CẤP ĐỘNG CƠ:**  
> Cáp USB Type-C cắm từ máy tính **CHỈ CẤP NGUỒN 5V CHO VI ĐIỀU KHIỂN ESP32-S3**.  
> Cáp USB **KHÔNG ĐỦ NGUỒN** để nuôi 6 động cơ bước!  
> **Để động cơ bước quay được, bạn BẮT BUỘC phải cấp nguồn công suất riêng (12V - 24V DC, 5A - 10A) vào cọc nguồn Driver của mạch!**

### Checklist chuẩn bị đấu nối:
1. **Dây cáp USB Type-C**:
   - Phải là cáp truyền dữ liệu (Data Cable), có đủ 4 lõi (`VBUS`, `GND`, `D+`, `D-`).
   - *Tránh dùng các loại cáp sạc giá rẻ chỉ có 2 lõi cấp nguồn vì máy tính sẽ không nhận cổng COM!*
2. **Nguồn động lực (Power Supply)**:
   - Nguồn tổ ong 12V DC hoặc 24V DC (khuyến nghị 24V DC 10A để động cơ đạt mô-men xoắn cực đại và không bị trượt bước).
   - Đấu nguồn vào cổng `VCC_MOTOR` / `GND` cấp cho dãy driver TMC2209.
3. **Chân cắm ESP32-S3**:
   - Giữ nguyên 100% tất cả các mối hàn hiện tại trên bo mạch (21 chân: Step/Dir 6 trục, Enable, UART TMC2209, SPI W5500).

---

## 5. CÁCH XÁC ĐỊNH CỔNG COM TRÊN WINDOWS

Sau khi cắm cáp USB từ ESP32-S3 vào máy tính:

1. Nhấn tổ hợp phím **`Windows + X`** trên bàn phím -> Chọn **`Device Manager`** (hoặc gõ `devmgmt.msc` vào hộp thoại Run).
2. Tìm đến mục **`Ports (COM & LPT)`** và nhấp đúp để mở rộng.
3. Bạn sẽ thấy tên một thiết bị Serial mới xuất hiện, ví dụ:
   - `Silicon Labs CP210x USB to UART Bridge (COM3)`
   - `USB-Enhanced-SERIAL CH343 (COM5)`
   - `USB Serial Device (COM4)`
4. Hãy ghi nhớ số hiệu cổng này (ví dụ: **`COM3`**).

*(Nếu cắm vào mà trong Device Manager hiện dấu chấm than vàng ⚠️ ở mục Other Devices, bạn chỉ cần tải driver CP210x hoặc CH340/CH343 về cài đặt trong 1 phút).*

---

## 6. CÁC BƯỚC KẾT NỐI VÀ ĐIỀU KHIỂN TỪ GIAO DIỆN GUI

### Bước 1: Khởi động phần mềm điều khiển
Mở Terminal trong thư mục `E:\robotarm-6-axis` và chạy:
```powershell
.venv\Scripts\python.exe main.py
```

### Bước 2: Thiết lập kết nối USB trên giao diện
1. Nhìn sang bảng điều khiển bên phải (Robot Panel), bấm vào tab **`CONNECTION`**.
2. Tại mục **Giao thức**, giữ mặc định là:
   ```
   🔌 USB Type-C (Serial COM)
   ```
3. Nhấp nút **`🔄 Quét`**:
   - Phần mềm sẽ tự động dò tìm các cổng COM đang cắm vào máy và hiển thị trong danh sách.
   - Nếu bạn biết chắc chắn cổng COM của mình (ví dụ `COM3`), bạn có thể gõ trực tiếp chữ `COM3` vào ô.
4. Mục **Baudrate**: Giữ mặc định `115200`.

### Bước 3: Bấm nút Kết Nối
1. Nhấn nút màu xanh lá: **`KẾT NỐI ROBOT (USB / LAN)`** (hoặc bấm nút `⚡ Connect Robot (USB / LAN)` trên thanh Ribbon phía trên).
2. Quan sát:
   - Nút chuyển thành màu đỏ: **`NGẮT KẾT NỐI (COM3)`**.
   - Nút trên thanh Ribbon hiển thị: **`🟢 Connected (COM3)`**.
   - Dưới khung Log Console báo:  
     `[INFO] Đã mở cổng USB Type-C Serial COM3 baud 115200`  
     `[INFO] Kết nối USB Serial thành công! (COM3 @ 115200 baud)`
   - Tại các ô khớp, chỉ số góc thực tế **`Act: 0.0°`** bắt đầu cập nhật dữ liệu trực tiếp từ ESP32 gửi về với tần số 30Hz!

### Bước 4: Điều khiển cánh tay thực tế
1. **Thử nghiệm Jog từng khớp**:
   - Dùng chuột kéo nhẹ Slider `Joint 1 (Bệ xoay)` từ `0°` lên `15°`.
   - Cánh tay mô phỏng 3D Digital Twin trên màn hình quay sang phải 15°.
   - Động cơ bước trục 1 ngoài đời thực quay êm ái sang phải đúng 15°!
2. **Thử nghiệm Lệnh Home toàn bộ**:
   - Bấm nút **`🔄 MoveJ (Home 0°)`** trên thanh Ribbon.
   - Toàn bộ 6 khớp đồng loạt di chuyển mượt mà về vị trí thẳng đứng 0°.
3. **Thử nghiệm Động học ngược MoveL Cartesian (Pieper IK)**:
   - Chuyển sang tab `CARTESIAN (IK)`.
   - Nhập tọa độ mong muốn, ví dụ `X: 200`, `Y: 0`, `Z: 300`.
   - Bấm `RUN MOVEL (PIEPER IK)`. Cánh tay sẽ tính toán giải nghiệm 6 góc khớp và đưa đầu hút đến đúng tọa độ trong không gian!

---

## 7. KHẮC PHỤC SỰ CỐ THƯỜNG GẶP (TROUBLESHOOTING)

### 1. Bấm Kết nối báo lỗi "Lỗi mở cổng USB Serial COMx: Access is denied (PermissionError)"
- **Nguyên nhân**: Cổng COM đang bị một phần mềm khác chiếm dụng (ví dụ: bạn đang mở Serial Monitor trong Arduino IDE hoặc PuTTY).
- **Cách xử lý**: Đóng cửa sổ Serial Monitor trong Arduino IDE lại rồi bấm nút Kết Nối trên GUI.

### 2. Cắm cáp USB vào máy tính nhưng mục "Ports (COM & LPT)" không xuất hiện
- **Nguyên nhân**: Dây cáp USB chỉ là cáp sạc (không có dây data), hoặc máy tính chưa cài driver chip nạp UART (CP2102/CH340).
- **Cách xử lý**: Đổi một sợi cáp USB Type-C truyền dữ liệu của điện thoại xịn.

### 3. Đã kết nối thành công, thanh Slider di chuyển nhưng động cơ ngoài đời không quay
- **Nguyên nhân 1**: Chưa bật nguồn 12V/24V cấp cho mạch Driver TMC2209 (nguồn USB chỉ đủ nuôi ESP32).
- **Nguyên nhân 2**: Công tắc Enable đang ở trạng thái ngắt. (Trong code firmware, chân `ENABLE_PIN 14` đã được kích hoạt tự động `setAutoEnable(true)`).

### 4. Động cơ kêu rít nhưng không quay (bị kẹt bước)
- **Nguyên nhân**: Dòng cài đặt trên Driver TMC2209 quá nhỏ hoặc tải cơ khí quá nặng.
- **Cách xử lý**: Kiểm tra thông số `MOTOR_CURRENTS` trong file `robotarmver2.ino`. Hiện tại khớp 1, 5, 6 đang để `1000 mA` (1.0A), khớp 2, 3, 4 đang để `350 mA`. Nếu động cơ tải nặng, có thể tăng lên `600 - 800 mA` cho các khớp có hộp số.

---
*Bản quyền tài liệu thuộc về Dự án Cánh Tay Robot 6 Bậc Tự Do AURA-6.*  
*Toàn bộ mã nguồn đã được đồng bộ tại thư mục `E:\robotarm-6-axis\Changes\`.*
