# Phân tích bộ ảnh Stafor 2K — 2026-09-22

## Dữ liệu

- 15 cm: 20 ảnh PNG, 2560×1440.
- 20 cm: 10 ảnh PNG, 2560×1440.
- 25 cm: 10 ảnh PNG, 2560×1440.
- 30 cm: 10 ảnh PNG, 2560×1440.
- 35 cm: 10 ảnh PNG, 2560×1440.
- 40 cm: 10 ảnh PNG, 2560×1440.
- Camera thực tế thương lượng `YUY2`, 30 FPS, exposure `-6`, autofocus báo `1`.
- Nhãn: Shopee/SPX khổ A6, đặt trên mặt thùng và có dải băng keo đi ngang.

Ảnh raw và báo cáo sinh tự động đã được đưa vào `.gitignore` vì có thể chứa tên,
địa chỉ và mã vận đơn.

## Kết quả QR

| Khoảng cách | Thành công ZXing | Tỷ lệ |
|---|---:|---:|
| 15 cm | 20/20 | 100% |
| 20 cm | 10/10 | 100% |
| 25 cm | 10/10 | 100% |
| 30 cm | 10/10 | 100% |
| 35 cm | 10/10 | 100% |
| 40 cm | 10/10 | 100% |

Tất cả ảnh trả về cùng một QR payload. Giá trị đầy đủ không được ghi lại trong tài liệu
này để hạn chế nhân bản dữ liệu vận đơn.

## Kết quả sau khi chuẩn hóa A6 bằng QR anchor

| Vùng | 15 cm | 20 cm | 25 cm | 30 cm | 35 cm | 40 cm |
|---|---:|---:|---:|---:|---:|---:|
| Toàn nhãn | 9.40 | 11.25 | 17.46 | 19.57 | 25.28 | **26.07** |
| Logo | 12.95 | 14.95 | 18.87 | 18.53 | 19.94 | **24.15** |
| QR | 33.55 | 42.07 | 55.95 | 61.82 | **73.76** | 73.16 |

40 cm là khoảng cách quang học tốt nhất trong bộ ảnh cho toàn nhãn và vùng logo;
QR đạt 100% ở toàn bộ 70 ảnh. Khoảng 35–40 cm là vùng làm việc ưu tiên nếu đo từ mặt
trước ống kính tới bề mặt nhãn.

## Ràng buộc lắp đặt 30–40 cm

Giới hạn cơ khí được xác định là 30–40 cm từ webcam tới **mặt băng tải trống**. Đại
lượng này không đồng nhất với khoảng cách chụp ở trên. Với camera nhìn từ trên xuống:

`khoảng cách ống kính–nhãn = chiều cao camera–băng tải - chiều cao kiện hàng`

Ví dụ, camera lắp ở 40 cm và kiện cao 10 cm chỉ tạo khoảng cách quang học 30 cm.
Do đó:

- Lắp camera ở mức cao nhất cho phép, ưu tiên 40 cm tới băng tải.
- Ghi nhận chiều cao kiện tối thiểu/tối đa trước khi khóa recipe cuối.
- Nếu nhãn luôn ở mặt bên, dùng cữ dẫn hướng để duy trì khoảng cách ngang 35–40 cm;
  khi đó chiều cao kiện không trực tiếp làm thay đổi khoảng cách chụp.
- Nếu nhãn ở mặt trên và kiện cao hơn 5 cm, webcam hiện tại không thể luôn duy trì vùng
  35–40 cm. QR vẫn khả thi ở 25–30 cm; logo được nghiệm thu trong vùng vận hành 30–40 cm.

## Đánh giá kỹ thuật

- **QR:** đạt yêu cầu trên bộ ảnh hiện tại.
- **Mã vận đơn:** lấy trực tiếp từ QR để làm lookup key.
- **Logo:** SIFT + RANSAC nhận được logo SPX ở 30/35/40 cm. Cần bổ sung ảnh âm tính
  không có logo hoặc có logo sai để đo false positive.
- Dải băng keo làm bề mặt nhãn không phẳng và tạo vùng phản xạ/biến dạng ngang nhãn.

Chức năng OCR địa chỉ đã được loại khỏi pipeline vận hành. Điều kiện nhận dạng là QR và
logo; điều kiện vật lý là kiện không bị detector đánh dấu móp/méo/rách mép.

## Thay đổi đã áp dụng

- Dùng QR làm fiducial để chỉnh phối cảnh nhãn về A6 1050×1480.
- Recipe ROI riêng cho QR và vùng tìm logo.
- Camera recipe dùng `YUY2`, 2560×1440, 30 FPS, autofocus bật.
- QR thử lần lượt thang 2× và 3×, có fallback từ ROI sang toàn frame; đạt 70/70 ảnh.
- Quality gate tạm thời được hiệu chỉnh theo bộ ảnh thực tế.
- Mở rộng vùng tìm QR để bao phủ cả vị trí 15 cm và 20 cm.
- Tạo template logo SPX không chứa dữ liệu cá nhân.
- Dùng SIFT + Lowe ratio + RANSAC để chịu được mờ và sai lệch phối cảnh tốt hơn template
  matching trực tiếp.
- Thêm detector đường bao kiện và rổ lỗi `DEFECT`.

## Kiểm tra logo ban đầu

Template matching trực tiếp không ổn định tại một số khoảng cách. Template SIFT được lấy
từ ảnh 35 cm và RANSAC được kiểm tra trên toàn bộ ảnh của vùng vận hành:

| Khoảng cách | Tỷ lệ phát hiện | Điểm min/median/max |
|---|---:|---:|
| 30 cm | 10/10 | 0.186 / 0.231 / 0.308 |
| 35 cm | 10/10 | 0.160 / 0.349 / 0.692 |
| 40 cm | 10/10 | 0.199 / 0.362 / 0.417 |

Ngưỡng tạm thời là 0.15, tối thiểu 20 good matches và RANSAC inlier ratio 0.50. Các ngưỡng
này chỉ được khóa sau khi có tập âm tính.

## Thử nghiệm tiếp theo

1. Đo chiều cao nhỏ nhất/lớn nhất của kiện để suy ra dải khoảng cách ống kính–nhãn thực.
2. Chụp ảnh băng tải trống tại đúng vị trí lắp camera.
3. Thu ground truth gồm kiện tốt, móp, méo, rách mép và rách bề mặt.
4. Thu nhãn không logo/sai logo để đo false positive.
5. Dùng hai đèn tán sáng ở góc 45°, tránh nguồn sáng thẳng vào băng keo.
