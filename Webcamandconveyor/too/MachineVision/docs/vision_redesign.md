# Thiết kế lại pipeline vision (2026-11-04)

## 1. Vấn đề quan sát được (GUI 03/11, 3 lần kiểm tra)

| Hiện tượng | Nguyên nhân gốc |
|---|---|
| "THỦNG LỖ" trên cảm biến quang và mép băng tải; nhãn nguyên vẹn cũng bị REJECT | Lỗ thủng được tìm bằng cách quét **mọi vùng tối** trong khung kiện |
| Khung kiện phủ gần cả ảnh | Khung kiện được ước lượng từ QR theo tỷ lệ của **nhãn A6 cố định**, rồi dò cạnh dọc theo hướng ngang; cạnh dò được lại là cảm biến hoặc dây điện |
| Kiện đúng là bị rách nhưng lý do đưa ra sai | Không có phép đo nào dành riêng cho nhãn; khi kết luận đúng là do trùng hợp |
| Kết quả khác nhau sau mỗi lần dịch camera | Ảnh nền đã cũ mà **không có bước nào kiểm tra** việc này; ngoài ra băng tải chạy làm vết chói và bụi đổi chỗ |
| Không đọc được QR thì phải chờ 11–14 s | zxing dò **mọi loại mã** trên ảnh phóng 3 lần (7680×4320), còn OpenCV chạy lại sau **từng biến thể** |

## 2. Nghiên cứu

| Hướng | Kết luận áp dụng |
|---|---|
| **Phát hiện bất thường chỉ học từ mẫu tốt** (PatchCore, anomalib) [1][2][3] | Phù hợp nhất cho rách, móp, bẩn, vì không cần thu ảnh kiện lỗi. Huấn luyện chỉ từ ảnh kiện tốt; backbone MobileNetV3 cho tỉ lệ tốc độ/độ chính xác tốt nhất. Cần PyTorch, có thể train trên RTX 2050 của máy. **Giai đoạn 3.** |
| **YOLO detect/segment kiện hỏng** [4][5][6] | Có dataset công khai: khoảng 2.787 ảnh (YOLOv11, phân loại damaged/normal, precision 0,927), cùng các bộ nhỏ 48–105 ảnh. Các bộ này chụp cảnh khác trạm, nên phải fine-tune trên ảnh của trạm. Dùng **YOLO11n-seg để phân vùng kiện** thay cho trừ nền, và lấy kích thước để phân 3 cỡ. **Giai đoạn 2.** |
| **Trừ nền trên băng tải** [7][8][9] | Các nhóm làm hệ thống đo kích thước kiện ghi nhận trừ nền **dễ vỡ khi camera rung hoặc ánh sáng đổi**, và nền băng tải chuyển động là một khó khăn riêng. Đúng với lỗi đã gặp. Nếu dùng thì phải có cơ chế phát hiện ảnh nền đã cũ. |
| **Đổi pixel sang kích thước thật** [7] | Quy về mm nhờ một vật có kích thước biết trước. Ở trạm này **cạnh QR** đóng vai trò đó, và nằm đúng trên mặt kiện. |
| **WeChat QR (CNN)** [10][11] | Mô hình siêu phân giải giúp đọc mã nhỏ. Benchmark của Dynamsoft (nhà cung cấp) cho thấy WeChat đọc ảnh mờ tốt hơn ZXing Java nhưng chậm hơn khoảng 4 lần; chưa có so sánh trực tiếp với zxing-cpp. OpenCV trong venv đã có module này. Chỉ nên thêm làm phương án dự phòng **sau khi tự đo** trên ảnh của trạm. |

## 3. Kiến trúc mới

```
frame ─► QR (zxing, chỉ tìm QR) ─► góc QR = mốc tỉ lệ + vị trí
            │
            ├─► Nhãn: chỉnh thẳng (similarity) ─► logo SIFT
            ├─► Viền giấy nhãn (KHÔNG cần ảnh nền) ─► label_tear
            └─► Kiện = vùng khác nền CHỨA QR ─┬─► nền không khớp → bỏ qua, ghi lý do
                                             ├─► khía viền / độ chữ nhật → dent / deformation
                                             └─► lỗ CHỈ trong lòng kiện, trừ vùng nhãn → hole
```

### Giai đoạn 1: đã làm (cổ điển, không cần train)
1. **`services/label_integrity.py`**:
   - Phân vùng giấy nhãn theo **độ sáng tương đối so với vùng trống quanh QR** cùng độ bão hòa màu thấp, nên tự thích nghi khi đèn hay exposure thay đổi.
   - Đo **độ sâu khía lõm** của viền nhãn theo đơn vị cạnh QR.
   - Nhãn chạm mép ảnh hoặc phân vùng hỏng thì trả `None` (không kết luận), không đoán.
2. **`services/damage.py`**:
   - Kiện là thành phần liên thông **chứa QR**, không phải vùng lớn nhất. Ảnh trừ nền dùng cả độ sáng lẫn màu sắc (Lab).
   - **Kiểm tra ảnh nền cũ**: tính tỉ lệ vùng nền thay đổi bên ngoài kiện, và diện tích kiện. Nếu bất thường thì bỏ qua phần viền kiện và ghi rõ lý do.
   - Mở hình thái học theo kích thước QR để cắt các "đuôi" do vết chói băng tải hoặc dây điện.
   - Lỗ thủng chỉ tìm trong lòng kiện (đã co vào 0,12 cạnh QR) và trừ vùng nhãn.
   - Đổi phép giãn 250 px bằng distance transform: thời gian từ 2 s xuống khoảng 50 ms.
3. **`readers/qr_reader.py`**: zxing chỉ tìm QR và thử hai bộ nhị phân hóa; OpenCV dự phòng chỉ chạy một lần.
4. **Nút CHỤP NỀN TRỐNG**: lấy trung vị 10 frame và **từ chối nếu còn thấy QR**.
5. **GUI**: thêm loại lỗi "RÁCH NHÃN"; khi chưa kết luận được thì hiện lý do.

### Giai đoạn 2: YOLO11n-seg cho kiện và cỡ kiện (cần dữ liệu)
- **Thu dữ liệu**: 3 cỡ kiện × khoảng 100 ảnh mỗi cỡ, đặt ở nhiều vị trí và góc xoay, có cả kiện lỗi. Ảnh lưu kèm metadata (dùng lại `capture_2k_samples.py`).
- **Gán nhãn** polygon kiện (có thể dùng SAM để tạo nhãn nháp rồi sửa tay), train `yolo11n-seg` 640 px trên GPU. Mục tiêu suy luận trên CPU khoảng 100 ms.
- **Cỡ kiện**: lấy cạnh `minAreaRect` của mask, quy ra mm bằng cạnh QR. Ngưỡng to/vừa/bé lấy từ kích thước thật của 3 loại kiện.
- Thay hoàn toàn bước trừ nền cho viền kiện. Ảnh nền chỉ còn dùng để đối chứng.

### Giai đoạn 3: PatchCore cho bề mặt kiện và nhãn (cần khoảng 50–200 ảnh kiện tốt)
- Đầu vào: mặt kiện và nhãn đã chỉnh thẳng (canvas QR-anchored sẵn có).
- Kết quả: bản đồ bất thường, phát hiện được rách, móp, bẩn, ướt, băng keo bung, kể cả loại lỗi chưa từng gặp.
- Dùng anomalib (PatchCore, backbone MobileNetV3), xuất ONNX hoặc OpenVINO để chạy trên CPU.

## 4. Kết quả giai đoạn 1 trên ảnh thật

| Bộ ảnh | Kết quả |
|---|---|
| 6 lần chụp GUI (1 nguyên vẹn, 5 rách) cùng 11 burst nguyên vẹn | **17/17 đúng** (thuật toán cũ: 3/6 lần chụp GUI sai lý do hoặc sai kết luận) |
| Độ sâu khía viền nhãn | nguyên vẹn ≤ 0,164 cạnh QR; rách 0,39–0,41; ngưỡng 0,27 |
| 56 frame nhãn nguyên vẹn | 0 báo nhầm |
| 60 ảnh dataset chụp ngang (nhãn sát tường trắng) | 60 `None` (không kết luận), 0 báo nhầm |
| Đọc QR, 130 ảnh | vẫn 130/130; trung bình 885 → **217 ms**; chậm nhất 11,4 → **1,4 s**; ảnh không có QR 14 → **2,1 s** |
| Một lần kiểm tra end-to-end | khoảng 0,8–1,0 s |

**Giới hạn còn lại:**
- Viền nhãn cần **nền tối hơn nhãn**: đúng với băng tải xanh và bìa xám, không đúng nếu nền trắng.
- Viền kiện vẫn phụ thuộc ảnh nền; băng tải chạy sẽ làm tỉ lệ thay đổi nền tăng dần, nên **chụp lại nền mỗi phiên**.
- Mới có **5 mẫu rách cùng một kiểu** (khía ở mép phải). Cần thử thêm: rách góc, rách giữa, nhãn nhăn, nhãn dán lệch.
- Phát hiện lỗ trong lòng kiện mới kiểm chứng trên ảnh tổng hợp.

## Nguồn
1. Roth et al., *Towards Total Recall in Industrial Anomaly Detection* (PatchCore), CVPR 2022 — https://mlanthology.org/cvpr/2022/roth2022cvpr-total
2. anomalib PatchCore — https://anomalib.readthedocs.io/en/v0.3.7/reference_guide/algorithms/patchcore.html
3. *Sequential PatchCore* (2025) — https://arxiv.org/pdf/2501.09579
4. PackageDamage (YOLOv11) — https://huggingface.co/cvtechniques/PackageDamage/blob/main/README.md
5. Roboflow Damaged boxes — https://universe.roboflow.com/labels-kpsk0/damaged-boxes/dataset/2
6. Roboflow BoxCondition — https://universe.roboflow.com/robot-arm-xlohb/boxcondition
7. Ninja Van, *Prototyping A Dimensioning System* — https://medium.com/ninjavan-tech/prototyping-a-dimensioning-system-eb33cd44b4f9 ; USPTO 8132728
8. USPTO 11401113 (conveyor image analysis) — https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11401113
9. Annals CSIS 2017, single-camera moving-object dimensioning — https://annals-csis.org/proceedings/2017/drp/26.html
10. OpenCV WeChat QR — https://docs.opencv.org/4.7.0/dd/d63/group__wechat__qrcode.html
11. Dynamsoft QR benchmark — https://www.dynamsoft.com/codepool/qr-code-reading-benchmark-and-comparison.html
