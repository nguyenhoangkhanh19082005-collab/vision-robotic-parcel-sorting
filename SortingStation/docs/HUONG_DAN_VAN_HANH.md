# Hướng dẫn vận hành trạm phân loại (bản 25/11/2026)

Tài liệu này dành cho người bật trạm lên chạy, gồm cả buổi quay video demo. Phần mềm: `SortingStation\run_station.bat`.

## 1. Chuẩn bị phần cứng

| Thiết bị | Kiểm tra |
|---|---|
| Băng tải + ESP32/W5500 | Đã nạp firmware **v2.1** (`Webcamandconveyor\too\MachineVision\firmware\conveyor_sensor\conveyor_sensor.ino`, hướng dẫn nạp ở `NAP_FIRMWARE.md` cùng thư mục). Cáp mạng nối laptop; card Ethernet của laptop đặt IP `192.168.2.10`, mask `255.255.255.0` |
| Cảm biến quang điểm gắp | Cắm vào ESP32, đặt **trước** điểm gắp; kiện dừng sau 2200 ms (`station_cycle.pick_stop_delay_ms`) |
| Webcam 2K | Gắn cố định trên giá, nhìn thẳng xuống băng. Nếu camera hoặc đèn bị xê dịch, phải chụp lại nền trống (mục 3) |
| Đèn | Bật như lúc chụp nền. Đèn tắt thì chu trình tự dừng và báo "Ánh sáng khác hẳn lúc chụp ảnh nền" |
| Robot 6 trục (nếu dùng) | Đã lưu gốc, có telemetry. Không dùng robot thì người vận hành nhấc kiện ở điểm gắp |
| Nút dừng khẩn phần cứng | Luôn trong tầm tay. Nút **Dừng toàn trạm** trên phần mềm chỉ là lệnh UDP |

## 2. Gửi kết quả lên web (một lần)

Tạo file `Webcamandconveyor\too\MachineVision\.env` với 2 dòng (lấy giá trị trong Supabase, mục Project Settings → API):
```
SUPABASE_URL=https://<project-id>.supabase.co
SUPABASE_KEY=<key máy trạm>
```
Không có file này, trạm vẫn chạy ở chế độ **thử cục bộ**: ô **Web** trên thanh tiêu đề báo "không gửi", kết quả hiện "(thử cục bộ)". Chi tiết: `Webcamandconveyor\too\MachineVision\docs\HUONG_DAN_UPLOAD_TRANG_THAI.md`.

## 3. Bật trạm

1. Chạy `SortingStation\run_station.bat`.
2. Trang **Vận hành**:
   - Nhấn **Kết nối camera**.
   - Khi băng tải trống, nhấn **Chụp nền trống** (chỉ cần khi camera, đèn hoặc băng vừa bị xê dịch).
3. Tab **Băng tải & camera**: nhấn **Kiểm tra kết nối**. Ô **Băng tải** trên thanh tiêu đề chuyển sang "trực tuyến".
4. Có robot: tab **Robot** → kết nối và lưu gốc. Sau đó, trên trang Vận hành, bật "Robot tự gắp sau mỗi lần kiểm tra" trong thẻ Robot 6 trục.

## 4. Chạy tự động

Nhấn **Bắt đầu chạy tự động**. Băng chạy PWM 40 cố định. Dải số 1–5 dưới thanh tiêu đề cho biết kiện đang ở bước nào:

| Bước | Người vận hành làm gì |
|---|---|
| 1. Camera kiểm tra | Đặt kiện ở đầu băng đang chạy, **buông tay ngay**. Nên đặt thẳng theo băng. Kiện kế tiếp đặt khi kiện trước đã ra khỏi vùng camera |
| 2. Tới điểm gắp | Không làm gì. Kiện lỗi chạy thẳng ra cuối băng: đỡ kiện ở sọt lỗi |
| 3. Chờ gắp | Không có robot: nhấc kiện ra rồi nhấn **Đã lấy kiện** (hoặc phím **Space**). Băng đứng yên tới khi bấm |
| 4. Robot thả vào thùng | Robot tự làm; theo dõi trên mô hình trạm |
| 5. Lưu lên web | Tự động. Ô **Web** báo "trực tuyến"; mất mạng thì báo số kết quả đang chờ gửi |

Khi GUI báo "chạy lùi đưa kiện về dưới camera", **không đặt kiện mới** cho tới khi kiện đó được kiểm tra xong.

Dừng: nhấn **Dừng chạy tự động**. Khi có sự cố: nhấn **Dừng toàn trạm**, hoặc nút dừng khẩn phần cứng.

## 5. Khi có thông báo lỗi

| Thông báo | Cách xử lý |
|---|---|
| Ánh sáng khác hẳn lúc chụp ảnh nền | Bật lại đèn. Nếu đổi đèn hẳn: lấy hết kiện khỏi băng, nhấn **Chụp nền trống**, chạy lại |
| Camera thấy cả khung hình khác ảnh nền | Camera bị dịch: lấy hết kiện, nhấn **Chụp nền trống**, chạy lại |
| Kiện tới điểm gắp mà camera không dừng được (Cần xem lại) | Kiện được đẩy xuống sọt lỗi. Xem lại cách đặt kiện (đặt thẳng, không quá sát kiện trước) |
| Không tra cứu được đơn trên Supabase | Mất mạng. Kiện vào sọt lỗi dạng "Cần xem lại"; kết quả sẽ được gửi khi có mạng lại |
| Băng tải dừng ngoài chu trình | Kiểm tra nguồn motor, cáp mạng ESP32 (watchdog 3 s), rồi chạy lại |

Mỗi lần chạy tự động được lưu ở `Webcamandconveyor\too\MachineVision\logs\runs\<giờ chạy>`. Khi báo lỗi, gửi kèm giờ chạy để phát lại đúng lần đó (`scripts\replay_station_runs.py`).

## 6. Mẹo khi quay video demo

- Bật đèn và chụp nền trống ngay trước khi quay.
- Phóng to cửa sổ (1920×1080). Dải hành trình 1–5 và thẻ **Kiện vừa kiểm tra** là hai chỗ nên quay cận.
- Thứ tự gợi ý: 1 kiện đạt → 1 kiện tem rách → 1 kiện QR mờ → 2 kiện đặt liền nhau (thấy băng chạy lùi đưa kiện về) → mở Supabase hoặc trang web để thấy đơn chuyển sang `SORTED`.
