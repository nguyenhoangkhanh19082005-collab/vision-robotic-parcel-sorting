# 🔌 CẨM NANG HƯỚNG DẪN NẠP CODE, ĐẤU NỐI PHẦN CỨNG & ĐIỀU KHIỂN CÁNH TAY ROBOT 6 TRỤC (AURA-6)

> **Tài liệu hướng dẫn thực tế dành cho người dùng đã hàn cố định chân ESP32-S3 và sử dụng module Ethernet W5500.**  
> Lưu trữ tại: `E:\robotarm-6-axis\Changes\HUONG_DAN_NAP_CODE_VA_DAU_NOI_PHAN_CUNG.md`  
> Tuyệt đối không thay đổi hay xóa các chân đã hàn hoạt động tốt!

---

## 1. SO SÁNH CHI TIẾT CHÂN CẮM: BẢN GỐC GITHUB VS BẢN CẬP NHẬT

### 1.1. Tin Vui Cho Phần Cứng Đã Hàn Của Bạn
Toàn bộ **21 chân quan trọng nhất** của hệ thống (gồm 12 chân điều khiển 6 động cơ bước, 1 chân Enable, 4 chân UART giao tiếp driver TMC2209 và 4 chân mạng Ethernet W5500) **HOÀN TOÀN GIỮ NGUYÊN 100% NHƯ BẢN GỐC GITHUB**:
* Bạn **KHÔNG CẦN hút thiếc**, **KHÔNG CẦN cắt mạch** hay hàn lại bất kỳ chân nào của 6 motor và module W5500!

### 1.2. Bảng Đối Chiếu Toàn Bộ Chân ESP32-S3

| Nhóm Thiết Bị | Tên Tín Hiệu | Chân Bản Gốc (GitHub) | Chân Bản Mới (Đã Cập Nhật) | Hiện Trạng Mạch | Hành Động Yêu Cầu |
|---|---|:---:|:---:|:---:|---|
| **Module W5500 (SPI)** | `CS` (Chip Select) | **GPIO 10** | **GPIO 10** | Đã hàn | **GIỮ NGUYÊN 100%** |
| | `MOSI` (Data In) | **GPIO 11** | **GPIO 11** | Đã hàn | **GIỮ NGUYÊN 100%** |
| | `MISO` (Data Out) | **GPIO 12** | **GPIO 12** | Đã hàn | **GIỮ NGUYÊN 100%** |
| | `SCK` (Clock) | **GPIO 13** | **GPIO 13** | Đã hàn | **GIỮ NGUYÊN 100%** |
| **Driver Động Cơ** | `ENABLE` chung | **GPIO 14** | **GPIO 14** | Đã hàn | **GIỮ NGUYÊN 100%** |
| **Khớp 1 (J1 - Đế)** | `STEP` / `DIR` | **GPIO 1 / 2** | **GPIO 1 / 2** | Đã hàn | **GIỮ NGUYÊN 100%** |
| **Khớp 2 (J2 - Vai)** | `STEP` / `DIR` | **GPIO 4 / 5** | **GPIO 4 / 5** | Đã hàn | **GIỮ NGUYÊN 100%** |
| **Khớp 3 (J3 - Khuỷu)**| `STEP` / `DIR` | **GPIO 6 / 7** | **GPIO 6 / 7** | Đã hàn | **GIỮ NGUYÊN 100%** |
| **Khớp 4 (J4 - Cẳng)** | `STEP` / `DIR` | **GPIO 15 / 16**| **GPIO 15 / 16**| Đã hàn | **GIỮ NGUYÊN 100%** |
| **Khớp 5 (J5 - Cổ tay)**| `STEP` / `DIR` | **GPIO 40 / 41**| **GPIO 40 / 41**| Đã hàn | **GIỮ NGUYÊN 100%** |
| **Khớp 6 (J6 - Đầu)** | `STEP` / `DIR` | **GPIO 38 / 39**| **GPIO 38 / 39**| Đã hàn | **GIỮ NGUYÊN 100%** |
| **UART Driver J1-J3** | `RX1` / `TX1` | **GPIO 17 / 18**| **GPIO 17 / 18**| Đã hàn | **GIỮ NGUYÊN 100%** |
| **UART Driver J4-J6** | `RX2` / `TX2` | **GPIO 8 / 9** | **GPIO 8 / 9** | Đã hàn | **GIỮ NGUYÊN 100%** |
| **Relay Bơm Hút (Pump)**| Bơm hút chân không | ⚠️ **GPIO 10 (Trùng CS)** | **Tùy chọn (Xem mục 2)** | *Chưa/đã hàn* | **Cần xử lý linh hoạt** |
| **Relay Van Xả (Valve)**| Van xả khí áp | ⚠️ **GPIO 11 (Trùng MOSI)**| **Tùy chọn (Xem mục 2)** | *Chưa/đã hàn* | **Cần xử lý linh hoạt** |

---

## 2. NÊN SỬ DỤNG CHÂN NÀO CHO ETHERNET W5500 KHI MẠCH ĐÃ KÍN CHÂN?

### 2.1. Đối với Module Mạng W5500: BẮT BUỘC GIỮ NGUYÊN CHÂN 10, 11, 12, 13
* **Lý do kỹ thuật cốt lõi:** Cụm chân `GPIO 10 (CS)`, `GPIO 11 (MOSI)`, `GPIO 12 (MISO)`, `GPIO 13 (SCK)` là cụm chân **phần cứng chuyên dụng FSPI (SPI2)** của ESP32-S3. Nó hỗ trợ DMA tốc độ cao 40MHz - 80MHz giúp việc truyền nhận gói tin mạng Ethernet đạt độ trễ cực thấp (< 5ms) mà không tốn tải CPU.
* **Kết luận:** Bạn **hàn W5500 vào 10, 11, 12, 13 là phương án tối ưu nhất**, tuyệt đối **KHÔNG CẦN ĐỔI CHÂN W5500**!

### 2.2. Vậy 2 Chân Relay Bơm Hút / Van Xả Nên Đấu Vào Đâu?
Vì bản gốc khai báo Bơm=10, Van=11 trùng vào chân SPI của W5500 (nếu kích bơm thì chân 10, 11 bị kéo xuống đất làm W5500 chết mạng), bạn có **3 phương án thông minh** để chọn mà không làm hỏng mạch đã hàn:

#### 🟢 Phương án A (Nếu trên mạch còn 1-2 chân ngoài rìa chưa hàn):
* Sử dụng chân **GPIO 21** (cho Bơm) và **GPIO 47** (cho Van).
* Đây là 2 chân GPIO thông dụng nhất trên ESP32-S3 DevKit, không bị ràng buộc bởi chức năng Boot strapping và không liên quan đến chip nhớ Flash.
* Khai báo trong file `robotarmver2.ino`:
  ```cpp
  #define PUMP_PIN 21
  #define VALVE_PIN 47
  ```

#### 🟡 Phương án B (Nếu mạch ĐÃ HÀN KÍN 100% KHÔNG CÒN CHÂN TRỐNG NÀO):
* **Giải pháp đột phá:** Tận dụng 2 chân UART của Driver TMC2209 (`GPIO 8` và `GPIO 9`)!
* **Tại sao làm được?** Driver bước TMC2209 (loại module cắm phổ biến) mặc định hoạt động hoàn hảo ở chế độ **Standalone Mode** (chỉ cần xung STEP, mức DIR và chân EN). Dòng động cơ đã được chỉnh cố định bằng biến trở Vref trên lưng driver. Việc giao tiếp UART chỉ dùng để đổi vi bước mềm (vốn đã cố định ở 16 vi bước).
* Bạn chỉ việc ngắt 2 dây UART J4-J6 (chân 8 và 9) để cắm vào mạch Relay Bơm và Van.
* Khai báo trong file `robotarmver2.ino`:
  ```cpp
  #define PUMP_PIN 8
  #define VALVE_PIN 9
  ```

#### ⚪ Phương án C (Nếu hiện tại chỉ cần chạy cánh tay 6 trục, chưa lắp đầu hút):
* Nếu hiện tại bạn chỉ cần đồng bộ chuyển động 6 trục của Digital Twin và robot thật ngoài đời (chưa gắn giác hút khí nén):
* Bạn đặt giá trị `-1` để vô hiệu hóa rơ-le:
  ```cpp
  #define PUMP_PIN -1
  #define VALVE_PIN -1
  ```
* Hệ thống sẽ tự động bỏ qua toàn bộ lệnh rơ-le, 6 động cơ và module Ethernet W5500 hoạt động trơn tru 100% mà không cần cắm thêm bất kỳ dây nào!

---

## 3. HƯỚNG DẪN NẠP CODE (FLASH FIRMWARE) TỪNG BƯỚC BẰNG ARDUINO IDE

### Bước 1: Cài Đặt Môi Trường Arduino IDE
1. Tải và cài đặt **Arduino IDE 2.x** (khuyến nghị bản mới nhất từ [arduino.cc](https://www.arduino.cc/en/software)).
2. Mở Arduino IDE $\rightarrow$ Vào **File** $\rightarrow$ **Preferences**.
3. Tại ô *Additional boards manager URLs*, dán đường link:
   ```text
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Vào biểu tượng **Boards Manager** (ở thanh công cụ bên trái) $\rightarrow$ Tìm kiếm `esp32` của *Espressif Systems* $\rightarrow$ Nhấn **Install** (khuyến nghị phiên bản 2.0.14 hoặc 2.0.17).

### Bước 2: Cài Đặt 3 Thư Viện Cần Thiết
Vào biểu tượng **Library Manager** (phím tắt `Ctrl + Shift + I`):
1. Tìm `FastAccelStepper` by *Lennart-K* $\rightarrow$ Nhấn **Install**.
2. Tìm `TMCStepper` by *teemuatlut* $\rightarrow$ Nhấn **Install**.
3. Tìm `Ethernet` by *Various* $\rightarrow$ Nhấn **Install** (Thư viện chuẩn của Arduino hỗ trợ W5500).

### Bước 3: Mở Dự Án Firmware
* Mở file: `E:\robotarm-6-axis\robotarmver2\robotarmver2.ino` trong Arduino IDE.

### Bước 4: Cấu Hình Board Trong Menu "Tools"
Cắm cáp USB Type-C từ máy tính vào cổng USB của board ESP32-S3. Sau đó vào menu **Tools** và chọn chính xác các thông số:
* **Board:** `ESP32S3 Dev Module`
* **Port:** Chọn đúng cổng COM của ESP32-S3 (ví dụ `COM3`, `COM5`...).
* **USB CDC On Boot:** `Enabled` *(CỰC KỲ QUAN TRỌNG: Phải Enabled để ESP32-S3 xuất thông tin log ra cổng Type-C!)*
* **CPU Frequency:** `240MHz (WiFi/BT)`
* **Flash Mode:** `QIO 80MHz`
* **Flash Size:** `8MB (64Mb)` hoặc `16MB` tùy theo phiên bản chip ESP32-S3 của bạn.
* **Partition Scheme:** `Huge APP (3MB No OTA/1MB SPIFFS)` hoặc `Default 4MB with spiffs`.
* **PSRAM:** `OPI PSRAM` (nếu dùng board N8R8 / N16R8) hoặc `Disabled` (nếu dùng board thường không có PSRAM).
* **Upload Speed:** `921600`.

### Bước 5: Nạp Code & Xử Lý Lỗi Bootloader
1. Nhấn nút **Upload** (Mũi tên sang phải `→`).
2. **Nếu Arduino IDE nạp mượt mà:** Chờ hiển thị `Done uploading`.
3. **Nếu báo lỗi `A fatal error occurred: Failed to connect to ESP32-S3: No serial data received`:**
   * Cách vào chế độ Download Mode:
     * Nhấn và **giữ nút BOOT** trên mạch ESP32-S3.
     * Nhấn nhả **nút RESET (EN)** một lần.
     * **Thả nút BOOT** ra.
     * Nhấn lại nút **Upload** trên Arduino IDE. Lần này code sẽ được nạp vào ngay lập tức!
4. Sau khi nạp xong, nhấn nút **RESET (EN)** trên mạch để ESP32-S3 khởi động vào firmware mới.
5. Mở **Serial Monitor** (phím tắt `Ctrl + Shift + M`), chọn baudrate **115200**:
   * Kiểm tra thông báo xuất hiện:
     ```text
     W5500 Industrial Ethernet IP: 192.168.1.177
     System Ready. UDP Binary Server Listening on Port 8888
     ```
   $\rightarrow$ Chúc mừng! Mạch ESP32-S3 và W5500 đã sẵn sàng nhận lệnh điều khiển!

---

## 4. HƯỚNG DẪN ĐIỀU KHIỂN CÁNH TAY ROBOT NGOÀI ĐỜI THẬT

### Bước 1: Cấu Hình IP Tĩnh Cho Card Mạng LAN Trên Máy Tính
ESP32-S3 sử dụng IP cố định `192.168.1.177`. Máy tính điều khiển cần ở cùng dải mạng `192.168.1.xxx`:
1. Trên Windows: Nhấn tổ hợp phím `Win + R` $\rightarrow$ gõ `ncpa.cpl` $\rightarrow$ Enter.
2. Nhấp chuột phải vào card mạng **Ethernet** (hoặc adapter USB-to-LAN đang cắm dây vào W5500) $\rightarrow$ Chọn **Properties**.
3. Chọn dòng **Internet Protocol Version 4 (TCP/IPv4)** $\rightarrow$ Nhấn **Properties**.
4. Chọn **Use the following IP address**:
   * **IP address:** `192.168.1.100` *(hoặc bất kỳ IP nào từ .2 đến .254, khác .177)*
   * **Subnet mask:** `255.255.255.0`
   * **Default gateway:** Để trống.
5. Nhấn **OK** $\rightarrow$ **Close**.

### Bước 2: Kiểm Tra Kết Nối Vật Lý & Thông Mạng
1. Cắm cáp mạng LAN từ cổng RJ45 của module W5500 vào cổng mạng của Laptop.
2. Quan sát đèn LED trên cổng RJ45 của W5500: Đèn vàng sáng đứng và đèn xanh lá chớp nhẹ chứng tỏ đường truyền vật lý đã thông.
3. Mở **PowerShell** trên máy tính và chạy lệnh kiểm tra:
   ```powershell
   ping 192.168.1.177
   ```
   * Nếu màn hình hiển thị: `Reply from 192.168.1.177: bytes=32 time<1ms TTL=128` $\rightarrow$ Kết nối mạng Ethernet cực kỳ hoàn hảo!

### Bước 3: Cấp Nguồn An Toàn Cho Động Cơ
* **Nguồn 1 (Logic):** Cáp USB Type-C cấp 5V cho vi điều khiển ESP32-S3 và 3.3V cho W5500.
* **Nguồn 2 (Động lực):** Cấp nguồn 24VDC (tối thiểu 5A - 10A) vào cọc nguồn cấp cho 6 Driver TMC2209.
* *Lưu ý an toàn:* Đặt cánh tay robot ở vị trí an toàn, không bị vướng vật cản xung quanh trước khi bật nguồn 24V!

### Bước 4: Khởi Chạy Giao Diện Điều Khiển (RoboDK HMI)
Trong thư mục `E:\robotarm-6-axis`, mở PowerShell và chạy:
```powershell
.\.venv\Scripts\python.exe main.py
```
1. Nhấn nút **🌐 Connect W5500 (UDP)** ở thanh công cụ phía trên.
2. Thanh trạng thái bên dưới sẽ chuyển sang màu xanh:
   `[UDP] Đã kết nối thành công tới Robot: 192.168.1.177:8888`.
3. Robot sẽ bắt đầu gửi phản hồi Telemetry 30Hz về giao diện máy tính.

### Bước 5: Thử Nghiệm Từng Bước (Theo Quy Trình Chuẩn Công Nghiệp)
1. **Kiểm tra trạng thái khóa trục (Motor Lock):**
   * Sau khi kết nối, chân `ENABLE_PIN 14` được kéo `LOW`. Bạn thử dùng tay đẩy nhẹ các khớp: các khớp phải cứng và giữ vị trí, không bị quay tự do.
2. **Thử nghiệm nhích từng khớp (Joint Jog):**
   * Tại tab **Joints Control**, thử bấm nút `J1+` hoặc kéo nhẹ thanh trượt $J_1$ thêm $+5^\circ$:
   * Quan sát Digital Twin và khớp 1 ngoài đời thật: Cả hai phải quay cùng một chiều với góc quay tương ứng.
   * Lặp lại lần lượt cho $J_2 \rightarrow J_6$.
3. **Thử nghiệm điều khiển tọa độ không gian (Cartesian MoveL):**
   * Tại ô **MoveL (Linear)**, nhập tọa độ mong muốn (ví dụ nhích Z lên $+10\text{ mm}$):
   * Nhấn **Thực thi (Execute)**: Thuật toán IK Pieper đã hiệu chuẩn CAD sẽ giải 6 góc và ra lệnh cho robot thật di chuyển đầu hút thành một đường thẳng chuẩn xác.
4. **Thử nghiệm tác vụ tự động:**
   * Bấm nút **Chạy Task 1 (Vẽ đường thẳng)** hoặc **Chạy Task 2 (Vẽ hình chữ nhật)**: Digital Twin trên màn hình và cánh tay robot ngoài đời sẽ vẽ đồng bộ cùng một quỹ đạo với độ chính xác cao.

---

## 5. BẢNG TỔNG KẾT DANH MỤC CÁC TỆP TIN ĐÃ CẬP NHẬT TRONG FOLDER `Changes`

Toàn bộ các file cấu hình, mã nguồn mới và tài liệu hướng dẫn đều được lưu giữ tại `E:\robotarm-6-axis\Changes\`:

* [`robotarmver2.ino`](file:///E:/robotarm-6-axis/Changes/robotarmver2.ino): Mã nguồn firmware ESP32-S3 có sẵn 3 phương án lựa chọn chân Relay, an toàn 100% cho chip W5500.
* [`HUONG_DAN_NAP_CODE_VA_DAU_NOI_PHAN_CUNG.md`](file:///E:/robotarm-6-axis/Changes/HUONG_DAN_NAP_CODE_VA_DAU_NOI_PHAN_CUNG.md): Bản hướng dẫn chi tiết này.
* [`BAO_CAO_CHI_TIET_THAY_DOI.md`](file:///E:/robotarm-6-axis/Changes/BAO_CAO_CHI_TIET_THAY_DOI.md): Toàn văn báo cáo khoa học giải quyết lỗi vỡ Digital Twin và hiệu chuẩn động học.
* [`digital_twin_dashboard.html`](file:///E:/robotarm-6-axis/Changes/digital_twin_dashboard.html): Bảng điều khiển kiểm chứng giao diện tương tác (Generative UI).
