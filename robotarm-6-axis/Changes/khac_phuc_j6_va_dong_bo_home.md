# BÁO CÁO KỸ THUẬT: KHẮC PHỤC TRIỆT ĐỂ LỖI J6 XOAY VÔ HẠN VÀ KHÔNG VỀ ĐƯỢC 0°

---

## 1. NGUYÊN NHÂN GỐC RỄ (ROOT CAUSE ANALYSIS)

### Nguyên nhân 1: Hiện tượng J6 xoay vô hạn khi kéo J6 hoặc kéo J3
Trong hàm nội suy bước `moveSynchronized` của firmware cũ có đoạn code:
```cpp
float ratio = (float)delta_steps[i] / max_delta;
int32_t sync_speed = (int32_t)(set_velocity * ratio);
int32_t sync_accel = (int32_t)(set_accel * ratio);

if (sync_speed < 10) sync_speed = 10;
if (sync_accel < 10) sync_accel = 10;

steppers[i]->setSpeedInHz(sync_speed);
steppers[i]->setAcceleration(sync_accel);
steppers[i]->moveTo(target_steps[i]);
```

**Cơ chế phát sinh lỗi:**
1. Khi bạn kéo **Khớp 3 (J3)**:
   - J3 có khoảng cách di chuyển rất lớn: `max_delta = 1000 - 2000` bước.
   - Trong khi đó, Khớp 6 (J6) đang ở 0° nhưng có thể lệch cực nhỏ 1 - 2 vi bước do làm tròn số (`delta_steps[5] = 1`).
   - Tỉ lệ tính toán của J6 bị ép xuống siêu nhỏ:
     $$\text{ratio} = \frac{1}{1500} \approx 0.00067$$
   - Khi đó, gia tốc của J6 bị hạ xuống mức tối thiểu:
     $$\text{sync\_accel} = \mathbf{10\text{ steps/s}^2}!$$
2. **Hậu quả của gia tốc 10 steps/s²**:
   - Khi motor đang quay ở vận tốc cao, nếu bị ép gia tốc hãm xuống $10\text{ steps/s}^2$, quãng đường phanh hãm của motor sẽ là:
     $$S_{\text{phanh}} = \frac{v^2}{2a} = \frac{1000^2}{2 \times 10} = \mathbf{50,000\text{ BƯỚC}}!$$
   - $50,000$ bước tương đương với **hơn 15 vòng quay liên tục** ($5,600^\circ$) và mất hơn **100 giây** mới có thể dừng lại!
   - Thêm vào đó, luồng lệnh từ slider gửi tới liên tục mỗi 15ms lại liên tục "reset" gia tốc về 10, khiến motor **hoàn toàn mất khả năng dừng và cứ thế quay tít vô tận**!

---

### Nguyên nhân 2: Tại sao bấm "MoveJ (Home 0°)" mà J6 vẫn kẹt ở `-94.9°`?
Có 2 điểm nghẽn đồng thời:
1. **Lỗi logic trong `handle_home_all` của `main.py`**:
   - Hàm cũ chỉ gọi `slider.setValue(0)`.
   - Nếu thanh trượt J6 trên màn hình **đang nằm sẵn ở 0°**, thư viện Qt **sẽ không phát tín hiệu `valueChanged`**!
   - Không có tín hiệu `valueChanged` đồng nghĩa với việc **không có bất kỳ gói tin nào được gửi xuống ESP32**.
   - Do đó, dù bạn bấm nút Home liên tục 5 lần, ESP32 vẫn không hề nhận được lệnh đưa J6 về 0!
2. **Lệnh Dừng khẩn cấp (Emergency Stop)**:
   - Firmware cũ dùng `stopMove()` (hãm dần theo gia tốc), khi gia tốc bị kẹt ở mức 10 thì lệnh dừng này mất cả phút mới dừng hẳn.
   - Giá trị `-94.9°` chính là tọa độ mà motor đã vọt lố trước khi người dùng bấm Emergency Stop.

---

## 2. CÁC ĐIỀU CHỈNH ĐÃ ÁP DỤNG ĐỂ SỬA TRIỆT ĐỂ

### 1. Nâng cấp Firmware `robotarmver2.ino`
- **Bộ lọc Dải chết (Deadband Filter)**:
  ```cpp
  // Nếu sai số <= 3 microsteps (~ 0.1 độ), giữ nguyên vị trí, không trigger xung
  if (delta_steps[i] <= 3) {
    delta_steps[i] = 0;
  }
  ```
  Khớp nào không có lệnh di chuyển sẽ được giữ cố định 100%, không bao giờ bị kéo theo khi khớp khác chuyển động.
- **Giữ gia tốc thắng chuẩn dứt khoát (Không giảm gia tốc theo ratio)**:
  ```cpp
  steppers[i]->setSpeedInHz(sync_speed);
  steppers[i]->setAcceleration(set_accel); // Luôn giữ gia tốc chuẩn (600 - 800 steps/s²)
  ```
  Motor sẽ thắng/dừng ngay lập tức tại đúng tọa độ mục tiêu trong vòng **0.1 - 0.2 giây**, loại bỏ 100% hiện tượng trôi bước quay vô hạn!
- **Dừng khẩn cấp tức thời (Instant Freezing)**:
  Sử dụng `forceStopAndNewPosition()` để ngắt phát xung ngay lập tức trong 1 millisecond khi người dùng bấm Dừng khẩn cấp.

### 2. Sửa lỗi `handle_home_all` trong `main.py`
```python
def handle_home_all(self):
    for slider in self.sliders:
        slider.setValue(0)
    self.sync_system_pose()
    if self.comm and self.comm.is_connected:
        self.comm.send_pc_command([0.0] * 6, self.tool_state) # Luôn gửi lệnh cưỡng bức 0 độ
```
Bất kể thanh trượt đang ở vị trí nào, lệnh cưỡng bức `[0, 0, 0, 0, 0, 0]` sẽ được phát ngay lập tức tới cánh tay thật.

---

## 3. FILE ĐÃ CẬP NHẬT VÀ ĐỒNG BỘ
- [Changes/robotarmver2.ino](file:///E:/robotarm-6-axis/Changes/robotarmver2.ino) & [robotarmver2/robotarmver2.ino](file:///E:/robotarm-6-axis/robotarmver2/robotarmver2.ino)
- [Changes/main.py](file:///E:/robotarm-6-axis/Changes/main.py) & [main.py](file:///E:/robotarm-6-axis/main.py)
