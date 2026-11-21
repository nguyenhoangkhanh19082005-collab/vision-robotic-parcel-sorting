# Kiểm tra J6 (động cơ 17HS3401-S): quay liên tục / kẹt / sai góc so với mô hình 3D

Ngày: 03/11/2026. Động cơ theo datasheet: bước 1,8°, 1,3 A/pha, **2,4 Ω/pha (±10%)**, 2,8 mH, giữ 28 N·cm,
4 dây, trục chữ D Ø5 × 21 mm. Theo CAD `Roboter.STEP`, J6 truyền động **trực tiếp 1:1**. Không có
puly cạnh động cơ J6; cặp puly 16/40 ở cổ tay thuộc J5. Firmware cũng đặt 1:1:
10° trên slider = 10° trên trục = 89 vi bước.

## 1. Đối chiếu triệu chứng
| Quan sát | Nguyên nhân thường gặp | Cách xác nhận |
|---|---|---|
| Rung/lắc tại chỗ, thỉnh thoảng giật một đoạn, quay hướng thất thường, "kẹt" | **Đấu nhầm cặp cuộn** (mỗi cặp chân driver nối vào 2 cuộn khác nhau) hoặc đứt 1 dây | Đo điện trở (mục 2); `[DRV] ... HO_CUON_A/B` |
| Động cơ quay nhưng khớp đứng yên hoặc trượt, góc khớp không theo mô hình | **Vít khóa khớp nối/puly lỏng** hoặc không tì vào mặt phẳng của trục D | Tắt nguồn, giữ trục động cơ, xoay khớp bằng tay (mục 4) |
| Quay tiếp khi firmware đã báo dừng | Nhiễu trên dây STEP (dây lỏng, không chung GND) hoặc lỗi thư viện cũ (FastAccelStepper đã bỏ) | `thu_j6.py`: firmware báo dừng mà dấu vẫn quay |
| Đúng hướng nhưng góc bằng 1/2, 1/4 hoặc gấp đôi | Driver mất cấu hình UART (VM bật sau ESP32 hoặc bị sụt) → vi bước 8/32/64 | `[DRV] J6 driver vua khoi dong lai...` (firmware mới tự cấu hình lại) |
| Ngược chiều mô hình | Đảo dây trong một cặp cuộn (vô hại) | Đổi `DIR_SIGNS[5]` thành `-1.0` |

## 2. Đo cặp cuộn bằng đồng hồ (tắt nguồn tổ ong, rút jack động cơ khỏi driver)
1. Đo lần lượt 6 tổ hợp giữa 4 dây động cơ.
2. Kết quả đúng: **2 cặp đo được ≈ 2,2–2,6 Ω**, các tổ hợp còn lại hở mạch (OL).
   - Có cặp đo hở hoặc chập chờn khi lắc dây: dây/jack đứt, cần thay cáp.
   - Có cặp đo ≈ 0 Ω hoặc giữa hai cặp không hở: chập cuộn hoặc chập dây, cần thay động cơ hoặc cáp.
3. Cắm vào driver TMC2209: **một cặp vào 1A–1B, cặp còn lại vào 2A–2B**. Tùy module, nhãn có thể là
   `A1 A2 B1 B2` hoặc `OA1 OA2 OB1 OB2`. Hai chân cùng một cuộn phải nằm cạnh nhau.
   - Cáp Dupont 1 m bán kèm 17HS3401-S **không có thứ tự màu cố định** giữa các shop, nên phải đo
     chứ đừng tin màu dây.
   - Đảo hai dây trong cùng một cặp chỉ làm đảo chiều quay (sửa bằng `DIR_SIGNS`).

## 3. Thử chéo driver (tách lỗi động cơ/cáp khỏi lỗi driver/dây tín hiệu)
1. Tắt nguồn. Rút động cơ J5 ra, cắm động cơ J6 vào ổ driver J5.
2. Bật nguồn và chạy `thu_j6.py --joint 5`. J5 có tỉ số 2,5, nên khi firmware báo 90° thì trục động cơ
   quay 225°.
3. Đọc kết quả:
   - Chạy êm và đúng: động cơ và cáp tốt. Lỗi nằm ở driver J6, dây STEP GPIO38, DIR GPIO39 hoặc GND.
   - Vẫn rung hoặc kẹt: lỗi ở động cơ hoặc cáp J6.
4. Lưu ý GPIO38 trên ESP32-S3-DevKitC-1 v1.1 cũng nối với LED RGB trên board. LED nháy khi J6 chạy là
   bình thường.

## 4. Cơ khí
- Tắt nguồn. Xoay trục J6 bằng tay: phải trơn đều, có nấc nhẹ (khoảng 1,6 N·cm), không có điểm kẹt.
- Vít khóa của khớp nối hoặc mặt bích phải **tì vào mặt phẳng của trục D** và được siết chặt. Đánh dấu bút
  lên trục và lên khớp nối: sau khi chạy thử, hai vạch phải vẫn thẳng hàng.

## 5. Chạy thử bằng phần mềm (sau khi nạp firmware 03/11)
```
E:\UDMT\.venv\Scripts\python.exe E:\UDMT\robotarm-6-axis\tests\thu_j6.py --port COM8
```
Dán một mẩu băng dính lên đầu J6. Khi chạy chậm +90° rồi về, mẩu băng phải quay đúng 1/4 vòng rồi
trở về chỗ cũ. Khi chạy nhanh +360° rồi về, mẩu băng phải quay đúng 1 vòng và dừng hẳn. Dòng
`[DRV] J6 ...` phải là `OK`. Nếu xuất hiện `HO_CUON_A/B` khi đang quay chậm thì cáp bị đấu nhầm cặp
hoặc bị đứt.
