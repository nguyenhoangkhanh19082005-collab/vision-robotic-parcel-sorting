# Hướng dẫn chụp ảnh và đo băng tải để dựng mô hình 3D

Mục tiêu: dựng mô hình băng tải **tham số hóa, đúng kích thước** (khung, mặt băng, rulô, chân, động cơ, thanh chắn, giá camera), xuất `.stl` / `.glb` và nạp vào digital twin.
Ảnh cho biết **hình dạng và tỉ lệ**; **số đo bằng thước** quyết định độ chính xác. Thiếu số đo thì mô hình chỉ đúng tỉ lệ, sai kích thước tuyệt đối.

Lưu ảnh vào: `E:\UDMT\SortingStation\assets\conveyor_photos\`

## 1. Chuẩn bị
- Đặt **vật chuẩn kích thước** trong mọi ảnh tổng thể: thước dây kéo dọc băng tải, hoặc tờ A4 (210 × 297 mm) đặt phẳng trên mặt băng.
- Ánh sáng đều, tắt flash (flash gây chói mặt băng), dọn đồ vật thừa quanh băng tải.
- Điện thoại dùng ống kính **1x hoặc 2x**, KHÔNG dùng góc rộng 0.5x (méo hình). Lùi xa để băng tải chiếm khoảng 70–80% khung hình.
- Giữ máy chắc tay, ảnh nét; độ phân giải gốc, không nén khi gửi.

## 2. Danh sách ảnh

| # | Tên file gợi ý | Cách chụp |
|---|---|---|
| 1 | `01_mat_ben_trai.jpg` | Ngang mặt bên, máy đặt ở **độ cao mặt băng**, hướng vuông góc với băng tải, thấy trọn chiều dài |
| 2 | `02_mat_ben_phai.jpg` | Như ảnh 1, phía đối diện (thường thấy động cơ/driver ở một bên) |
| 3 | `03_dau_vao.jpg` | Nhìn thẳng từ đầu vào, ngang mặt băng (thấy bề rộng, rulô, chân) |
| 4 | `04_dau_ra.jpg` | Nhìn thẳng từ đầu ra |
| 5 | `05_tu_tren.jpg` | Từ trên xuống, càng vuông góc càng tốt (đứng trên ghế), thấy cả băng tải + tờ A4/thước |
| 6–9 | `06_goc_45_*.jpg` | 4 góc chéo 45° từ trên xuống (trước-trái, trước-phải, sau-trái, sau-phải) |
| 10 | `10_dong_co.jpg` | Cận cảnh động cơ, hộp số, puly/khớp nối, driver BTS7960 |
| 11 | `11_rulo.jpg` | Cận cảnh đầu rulô và cơ cấu căng băng |
| 12 | `12_chan_khung.jpg` | Cận cảnh chân và mặt cắt thanh nhôm định hình (nếu có) |
| 13 | `13_gia_camera.jpg` | Giá đỡ webcam + vị trí webcam so với băng tải |
| 14 | `14_robot_va_bang_tai.jpg` | Từ trên xuống hoặc chéo, thấy **cả robot lẫn băng tải** (để đặt đúng vị trí tương đối) |

## 3. Số đo cần ghi (mm) — điền vào `kich_thuoc.txt` cùng thư mục

1. Chiều dài tổng khung; chiều dài phần băng chạy (tâm rulô đến tâm rulô).
2. Bề rộng mặt băng; bề rộng ngoài khung.
3. Chiều cao mặt băng so với mặt bàn/sàn đặt robot.
4. Đường kính rulô.
5. Kích thước thanh khung (ví dụ nhôm 20×20, 20×40) và vị trí, số lượng chân.
6. Động cơ: kích thước thân, bên nào, cách đầu ra bao nhiêu.
7. Thanh chắn hai bên (nếu có): chiều cao.
8. Webcam: khoảng cách dọc băng từ đầu vào tới tâm camera; chiều cao ống kính so với mặt băng.
9. **Vị trí so với robot** (quan trọng nhất cho phân loại): từ tâm đế robot (trục J1) đến đường tâm băng tải theo phương ngang; vị trí điểm gắp dọc băng; chiều cao mặt bàn robot so với mặt băng.
10. Vị trí 3 thùng phân loại so với tâm đế robot.

## 4. Sau khi có ảnh và số đo
Mô hình sẽ được dựng bằng code (PyVista/trimesh), xuất `assets\conveyor\conveyor.stl` và `.glb`.
Mục 9–10 cũng dùng để cập nhật `config\station.yaml` (`layout`, `sorting`), để IK nhắm đúng vị trí thật.
