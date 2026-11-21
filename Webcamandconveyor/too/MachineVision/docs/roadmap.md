# Roadmap triển khai

## Phase 1 — Camera và chất lượng ảnh

- [x] Adapter camera OpenCV với 2K/MJPG và property read-back.
- [x] Laplacian, Tenengrad, motion và glare gate.
- [x] Focus peaking phục vụ căn chỉnh.
- [x] Script thu dữ liệu hiệu chỉnh ngưỡng.
- [x] Script độc lập chụp PNG 2K nguyên bản ở 15 cm và 20 cm với khung A6.
- [x] Đo webcam Stafor thật tại 15/20/25/30/35/40 cm; vùng nét tốt nhất là 35–40 cm.
- [x] Chốt khoảng cách quang học ưu tiên 35–40 cm; chiều cao lắp camera là 40 cm tới băng tải.
- [ ] Đo dải chiều cao kiện để xác nhận khoảng cách quang học thực khi nhìn từ trên xuống.
- [x] Chốt ROI A6 và QR anchor từ ảnh nhãn thật.

## Phase 2 — QR và logo

- [x] ZXing-C++ cùng OpenCV fallback.
- [x] Preprocessing variants cho QR.
- [x] Loại OCR địa chỉ khỏi pipeline vận hành.
- [x] Template logo SPX và đối sánh SIFT + RANSAC.
- [x] Perspective correction bằng contour hình chữ nhật, có thể bật theo recipe.
- [x] Đánh giá/tinh chỉnh perspective correction trên 30 ảnh nhãn thực tế.
- [x] Xác minh ZXing 70/70 ảnh tại 15–40 cm.
- [x] Xác minh logo trên bộ ảnh vận hành 30/35/40 cm.
- [ ] Đánh giá logo bằng bộ âm tính không có/sai logo.

## Phase 3 — Tình trạng kiện hàng

- [x] Detector đường bao dựa trên ảnh băng tải trống.
- [x] Metric rectangularity, solidity và số đỉnh cho móp/méo/rách mép.
- [x] Định tuyến kiện lỗi sang rổ `DEFECT`.
- [ ] Chụp ảnh tham chiếu băng tải trống tại vị trí vận hành.
- [ ] Thu bộ ground truth: ≥30 tốt, ≥10 móp, ≥10 méo, ≥10 rách.
- [ ] Huấn luyện detector cho vết rách nằm bên trong bề mặt kiện (không làm biến dạng biên).

## Phase 4 — Data và quyết định

- [x] Repository abstraction và Supabase adapter.
- [x] PASS/REJECT/MANUAL_REVIEW classifier.
- [ ] Ánh xạ schema Supabase thực tế.
- [x] Lưu audit image và inspection result nguyên tử theo inspection ID.

## Phase 5 — HMI

- [x] PyQt6 camera preview, focus metrics, quality gate và lịch sử.
- [x] Render/smoke test không cần camera.
- [x] Hiển thị trạng thái logo và hư hỏng trong kết quả.
- [ ] Hiển thị bounding boxes/mask hư hỏng trực tiếp trên preview.
- [ ] Operator login/recipe selection nếu hệ thống thực tế yêu cầu.

## Phase 6 — ESP32/W5500/HW039

- [x] Dự thảo protocol và state machine.
- [ ] Chốt wiring, sensor type, motor voltage/current và safety chain.
- [ ] Firmware ESP32 TCP server/client.
- [ ] PC station controller với sequence/ACK/timeout/retry.
- [ ] Hardware-in-the-loop test.

## Phase 7 — Robot và bàn giao

- [ ] Chốt giao thức robot và mapping bin.
- [ ] End-to-end cycle test.
- [ ] Soak test 500 chu kỳ và báo cáo accuracy/latency.
- [ ] Khởi tạo Git repository, loại bỏ secrets, tạo README vận hành.
- [ ] Đẩy repository lên GitHub sau khi người dùng chọn tên và phạm vi visibility.
