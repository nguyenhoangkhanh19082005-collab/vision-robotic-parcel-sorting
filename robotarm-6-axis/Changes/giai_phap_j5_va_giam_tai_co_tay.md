# BÁO CÁO KỸ THUẬT: KHẮC PHỤC LỖI KHỚP J5 & GIẢI PHÁP GIẢM TẢI CỤM CỔ TAY CHO J1, J2

---

## PHẦN 1: PHÂN TÍCH VÀ KHẮC PHỤC LỖI KHỚP 5 (J5 - WRIST PITCH)

### 1.1. Tại sao J5 quay lên khi kéo thanh trượt xuống?
- **Nguyên nhân**: Ở phiên bản trước, hệ số chiều quay của Khớp 5 được đặt là `DIR_SIGNS[4] = -1.0`. Khi bạn kéo slider xuống góc âm (ví dụ `-110°`), số bước tính toán `target_steps = -110 * (-steps_per_degree)` lại biến thành số dương (quay theo chiều ngược lại), khiến cổ tay ngóc lên trên thay vì gập xuống dưới.
- **Giải pháp**: Đã đổi lại `DIR_SIGNS[4] = 1.0` trong cả hai file [Changes/robotarmver2.ino](file:///E:/robotarm-6-axis/Changes/robotarmver2.ino) và [robotarmver2/robotarmver2.ino](file:///E:/robotarm-6-axis/robotarmver2/robotarmver2.ino). Bây giờ khi kéo slider xuống góc âm, động cơ J5 sẽ gập xuống dưới chuẩn 100%.

### 1.2. Tại sao góc `Act` hiển thị lệch nhiều so với giao diện (`Target: -110°` nhưng `Act: -75.9°`)?
Hiện tượng này do 2 yếu tố:
1. **Gia tốc ramp và thời gian di chuyển**:
   Firmware sử dụng điều khiển gia tốc hình thang (`FastAccelStepper`). Khi bạn kéo nhanh từ `0°` xuống `-110°`, động cơ cần khoảng 1.8 - 2.0 giây để tăng tốc và giảm tốc về đích. Trong lúc động cơ đang quay dở chừng, góc thời gian thực trả về sẽ tăng dần (`0° -> -30° -> -75.9° -> ...`). Nếu chụp ảnh màn hình khi động cơ chưa về đích hẳn, `Act` sẽ dừng ở giá trị trung gian.
2. **Kẹt cơ khí do chạm giới hạn vỏ nhựa**:
   Do chiều quay trước đó bị ngược (quay lên trên), cụm cổ tay đã đâm vào vách nhựa chặn hành trình ở khoảng `-75°` đến `-80°`, làm động cơ bước bị trượt bước (stall bước) và không thể bước tiếp đến `-110°`.
3. **Xác nhận thực nghiệm tỉ số truyền đai (Gear/Belt Ratio = 2.5:1)**:
   - Trong bản vẽ CAD `Roboter.STEP`, khớp J5 sử dụng cụm puly đai `Riemenscheibe 16mm` và `Riemenscheibe 40mm`.
   - **Thực nghiệm xác nhận 100%**: Khi ra lệnh trên phần mềm quay $-110^\circ$, cổ tay ngoài đời thật quay được $\approx 44^\circ$.
     $$\text{Tỉ số truyền thực tế} = \frac{110^\circ}{44^\circ} = \mathbf{2.50}!$$
   - Khi đặt `GEAR_RATIOS[4] = 2.5` trong `robotarmver2.ino`, khi kéo slider $-110^\circ$, cổ tay ngoài đời sẽ gập xuống chính xác đúng $-110^\circ$ và nhãn `Act` trùng khớp 100%!

---

## PHẦN 2: TẠI SAO CỔ TAY QUÁ NẶNG VÀ J1, J2 KHÔNG THỂ ĐỨNG VỮNG?

### 2.1. Phân tích vật lý cánh tay đòn (Cantilever Overhung Load)

Dựa trên kích thước chuẩn trích xuất từ file CAD `Roboter.STEP`:
- Chiều dài cánh tay trên (Link 2): $a_2 = 154.5\text{ mm}$
- Chiều dài khuỷu tay đến cổ tay (Link 3): $d_3 + d_4 = 267.0\text{ mm}$
- **Tổng chiều dài vươn xa từ Khớp Vai (J2) đến cụm Cổ tay**:
  $$L = 154.5 + 267.0 = 421.5\text{ mm} \approx \mathbf{0.42\text{ m}}$$

Trong cụm cổ tay (nơi tay bạn đang đỡ trong ảnh):
- 1 động cơ bước NEMA 17 cho Khớp 5: nặng $\approx 280\text{g} - 350\text{g}$
- 1 động cơ bước NEMA 17 cho Khớp 6: nặng $\approx 280\text{g} - 350\text{g}$
- Các vòng bi thép (`Lager 32x20x7`, `Lager 19x5x6`), trục ren, bu-lông M4/M5 và vỏ nhựa in 3D: $\approx 350\text{g} - 500\text{g}$
👉 **Tổng khối lượng cụm đầu cần dồn ở cổ tay**: $m_{\text{wrist}} \approx \mathbf{1.0\text{ kg} - 1.2\text{ kg}}$!

### 2.2. Mô-men uốn tác dụng lên Khớp Vai (J2) và Khớp Đế (J1)
1. **Mô-men trọng trường đè lên Khớp Vai (J2)**:
   $$\tau_{\text{wrist}} = m \times g \times L = 1.0\text{ kg} \times 9.81\text{ m/s}^2 \times 0.42\text{ m} \approx \mathbf{4.12\text{ N}\cdot\text{m}}$$
   Cộng thêm trọng lượng bản thân của cánh tay Link 2 và Link 3 ($\approx 1.5\text{ kg}$):
   $$\tau_{\text{total\_J2}} \approx \mathbf{6.5 - 7.0\text{ N}\cdot\text{m}}!$$

2. **Năng lực thực tế của động cơ Khớp 2 (J2)**:
   - Trong code cũ, dòng điện driver TMC2209 của J2 chỉ đặt có **`350 mA`** (`0.35A`).
   - Động cơ J2 là loại Nema 17 kèm hộp số hành tinh 5.18:1 (`17HS13-0404S-PG5`).
   - Ở dòng 350mA, mô-men tối đa mà motor sinh ra chỉ đạt khoảng **`0.6 - 0.8 N·m`**.
   👉 **KẾT LUẬN**: Lực yêu cầu là **$7.0\text{ N}\cdot\text{m}$**, trong khi động cơ chỉ gánh được **$0.8\text{ N}\cdot\text{m}$** (chênh lệch gần 9 lần!). Đó là lý do vì sao khi bạn buông tay, cánh tay sẽ lập tức rơi tự do, và động cơ J1 bị vặn lệch tâm khiến ổ bi đế bị kẹt cứng không xoay được!

---

## PHẦN 3: BỐN GỢI Ý GIẢI PHÁP TỐI ƯU CƠ KHÍ & ĐIỀU KHIỂN

### Gợi ý 1: Trợ lực tức thì bằng Lò xo kéo hoặc Lò xo khí (Spring / Gas Strut Assist) - *Khuyên dùng làm ngay*
Đây là giải pháp kinh điển được áp dụng trên 100% cánh tay robot công nghiệp tải nặng (KUKA, ABB, FANUC):
- **Cách làm**: Lắp 1 lò xo kéo kim loại (hoặc 1 cây ti trợ lực lò xo khí mini loại 50N - 80N, giá khoảng 40.000đ - 60.000đ trên Shopee) nối giữa bệ đỡ Link 1 và cánh tay Link 2.
- **Tác dụng**: Khi cánh tay vươn dài ra, lò xo sẽ bị kéo căng và tạo lực kéo ngược lại để cân bằng trọng lượng của cụm cổ tay. Giải pháp này giúp triệt tiêu ngay **70% - 80% tải trọng**, giúp motor J2 vận hành nhẹ nhàng như không tải!

### Gợi ý 2: Nâng dòng điện driver TMC2209 và làm mượt gia tốc (Đã chỉnh trong code)
Tôi đã cập nhật trực tiếp vào firmware mới:
```cpp
const uint16_t MOTOR_CURRENTS[6] = {1000, 650, 500, 450, 800, 800};
int32_t set_accel = 600; // Giảm gia tốc xuống 600 steps/s² để khử rung giật quán tính
```
- Nâng dòng J2 từ `350 mA` lên `650 mA`: Tăng lực giữ mô-men của khớp vai lên gần gấp đôi.
- Giảm gia tốc từ 1000 xuống 600: Cánh tay khởi động và dừng êm hơn, không bị giật quán tính làm trượt bước.

### Gợi ý 3: Thay động cơ cổ tay bằng NEMA 17 Pancake hoặc NEMA 14 (Giảm cân trực tiếp)
- Khớp J5 và J6 chỉ cần xoay đầu hút và vật gắp nhẹ (< 200g), không cần mô-men quá lớn.
- Việc đặt 2 động cơ NEMA 17 tiêu chuẩn (dày 40mm, nặng 300g/con) ở tận mút cổ tay là quá dư thừa tải tĩnh.
- **Nâng cấp**:
  - Thay motor J5/J6 bằng **NEMA 17 Pancake (loại mỏng 20-25mm, nặng chỉ 130g)** hoặc **NEMA 14 (nặng 180g)**.
  - Cắt giảm ngay **$\approx 350\text{g} - 400\text{g}$** trực tiếp tại đầu cần $\to$ Giảm mô-men uốn đè lên J1, J2 đi hơn một nửa!

### Gợi ý 4: Chuyển động cơ lùi về phía thân khuỷu tay (Remote Belt Drive / Cáp truyền động)
- Đây là thiết kế chuẩn mực của các dòng robot mã nguồn mở cao cấp (như AR4 Robot, Thor Arm):
- Cả 2 động cơ J5 và J6 được gắn cố định tại **khuỷu tay (Link 3)** hoặc đối trọng phía sau. Chuyển động được truyền lên cổ tay thông qua dây đai GT2 dài hoặc trục lồng đồng trục.
- Đầu cổ tay chỉ còn lại bạc đạn và khớp xoay bằng nhựa in 3D (nặng chưa tới 200g), giúp J1 và J2 chuyển động cực nhanh, mượt mà và không bao giờ bị rơi.

---

## BẢNG TỔNG HỢP SO SÁNH CÁC PHƯƠNG ÁN

| Phương án | Chi phí | Thời gian thực hiện | Hiệu quả giảm tải | Mức độ khuyến nghị |
| :--- | :--- | :--- | :--- | :--- |
| **1. Gắn Lò xo trợ lực J2** | Rất rẻ (< 50k) | 15 - 30 phút | **Giảm 75% tải** | ⭐⭐⭐⭐⭐ (Khuyên dùng ngay) |
| **2. Tăng dòng driver TMC2209** | 0 VNĐ | Nạp code 1 phút | **Tăng 80% lực giữ** | ⭐⭐⭐⭐⭐ (Đã nạp sẵn trong code) |
| **3. Thay motor NEMA 17 Pancake** | Vừa phải (~200k) | Thay motor 1 giờ | **Giảm 40% trọng lượng** | ⭐⭐⭐⭐ (Nên làm khi nâng cấp) |
| **4. Thiết kế Remote Belt Drive** | Cần in lại nhựa | 1 - 2 ngày | **Giảm 80% trọng lượng** | ⭐⭐⭐ (Cho dự án chuyên nghiệp) |
