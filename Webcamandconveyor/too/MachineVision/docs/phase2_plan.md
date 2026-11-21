# Kế hoạch giai đoạn 2: phân vùng kiện bằng YOLO11-seg và phân loại 3 cỡ kiện

Liên quan: [vision_redesign.md](vision_redesign.md), mục 3.

## 1. Mục tiêu và tiêu chí nghiệm thu

| # | Mục tiêu | Tiêu chí đạt (trên tập test giữ riêng, chụp tại trạm) |
|---|---|---|
| G1 | Tách đúng **viền kiện** mà không phụ thuộc ảnh nền | Mask IoU ≥ 0,90; tìm được kiện ≥ 99% ảnh; không có ảnh nào nhận nhầm cảm biến, dây điện hay tay là kiện |
| G2 | Phân loại **to / vừa / bé** | Đúng ≥ 98%; sai số cạnh đo được ≤ 5 mm hoặc ≤ 3% |
| G3 | Phát hiện **móp, méo, rách viền kiện** dựa trên mask mới | Recall ≥ 90% trên kiện lỗi; báo nhầm ≤ 2% trên kiện tốt |
| G4 | Chạy được trên laptop của trạm | Suy luận trên CPU ≤ 150 ms/ảnh (GPU ≤ 30 ms); toàn bộ một lần kiểm tra ≤ 1,2 s |
| G5 | Không làm hỏng phần đang chạy | 66 test cũ vẫn đạt; vẫn quay về được cách trừ nền bằng một khóa config |

## 2. Thông tin cần bạn cung cấp trước khi bắt đầu
1. **Kích thước thật** (dài × rộng × cao, mm) của 3 loại kiện to, vừa, bé, kèm ảnh chụp từng loại.
2. **Cạnh mã QR trên nhãn** (mm, đo bằng thước): đây là "thước đo" để quy pixel ra mm. Nếu nhãn của các hãng khác nhau có QR khác cỡ thì ghi riêng từng loại.
3. Độ cao **cố định** của camera so với mặt băng tải ở bản cuối, và kiện cao nhất. Hai số này dùng để kiểm tra kiện to có nằm trọn trong khung không.
4. Đồng ý cài môi trường train riêng (mục 4). Cần khoảng **4 GB** trên ổ E, hiện còn 8,2 GB trống.

## 3. Thu dữ liệu (ước tính 2–3 buổi)

### 3.1 Công cụ
Mở rộng `scripts/capture_2k_samples.py` thành **`scripts/capture_parcels.py`**:
- Phím `1/2/3` chọn cỡ kiện (`small/medium/large`), phím `G/D` chọn tình trạng (`good/damaged`), phím `E` chụp băng tải trống.
- Ghi ảnh PNG không nén vào `dataset/parcels/<size>/<condition>/`.
- Lưu file JSON đi kèm: thông số camera, exposure, mã QR đọc được, góc QR và cạnh QR (px), giờ chụp, độ cao camera.
- Dùng **đúng cấu hình camera của trạm** (exposure −7, MSMF 2K) và cùng đèn, để ảnh train giống ảnh lúc chạy thật.

### 3.2 Kịch bản chụp (khoảng 450 ảnh)
| Nhóm | Số ảnh | Cách chụp |
|---|---|---|
| Mỗi cỡ, kiện tốt | 3 × 100 | 5 vị trí (giữa, trái, phải, trước, sau) × 4 góc xoay (0°, ±15°, 30°) × 5 lần đặt lại. Nên dùng ≥ 2 kiện khác nhau cho mỗi cỡ |
| Kiện lỗi (mọi cỡ) | 90 | Móp góc, bẹp cạnh, rách thành hộp, thủng lỗ, băng keo bung, rách nhãn, mỗi kiểu khoảng 15 ảnh |
| Ảnh khó | 40 | Tay người trong khung, 2 kiện cạnh nhau, kiện chạm mép khung, băng tải đang chạy |
| Băng tải trống | 20 | Đối chứng âm, và để kiểm tra mô hình không nhận nhầm cảm biến hay dây là kiện |

Tách **theo kiện vật lý** (không theo ảnh): 70% train / 15% val / 15% test. Như vậy ảnh của cùng một kiện không lọt sang tập test.

### 3.3 Quyền riêng tư
Nhãn vận đơn có **họ tên, số điện thoại, địa chỉ**. Vì vậy:
- **Không** tải ảnh lên Roboflow hay bất kỳ dịch vụ gắn nhãn trên cloud nào.
- Chỉ gắn nhãn bằng công cụ chạy trên máy (mục 3.4).
- Thư mục `dataset/parcels` không đưa vào gói bàn giao. Nếu cần chia sẻ, làm mờ vùng địa chỉ (có script tự làm mờ dựa vào vị trí nhãn sau khi chỉnh thẳng).

### 3.4 Gắn nhãn (ước tính 1 buổi)
- **Tự động tạo mask nháp:** dùng chính pipeline hiện tại. Có QR → vùng khác nền chứa QR (đã có trong `damage._segment_parcel`) → polygon. Chụp nền trống ngay trước mỗi đợt chụp thì mask nháp sẽ khá sát.
- **Sửa tay** bằng **X-AnyLabeling** (chạy cục bộ, có SAM hỗ trợ click-to-segment), xuất polygon định dạng YOLO.
- **Lớp:** chỉ có **1 lớp `parcel`**. Cỡ kiện **tính từ hình học** (mục 5), không để mô hình tự học. Nhờ vậy không cần train lại khi thêm một cỡ mới.
- Thuộc tính đi kèm mỗi ảnh (lưu trong JSON, không phải lớp YOLO): `size`, `condition`, `defect_type`, dùng để đánh giá G2 và G3.

## 4. Môi trường train (tách khỏi runtime)
- Tạo **`.venv-train`** riêng, trong đó cài `torch` (bản CUDA cho RTX 2050, 4 GB VRAM) và `ultralytics`. **Không** cài vào `.venv` của ứng dụng, để ứng dụng vẫn nhẹ và ổn định.
- **Runtime** chỉ thêm `onnxruntime` (khoảng 15 MB) để chạy file `parcel_seg.onnx`.
- Trước khi cài, dọn tình trạng venv đang có hai gói OpenCV (`opencv-python` 5.0 và `opencv-contrib-python` 4.10): chỉ giữ `opencv-contrib-python`.

## 5. Train và tính cỡ kiện (ước tính 1–2 buổi)

### 5.1 Train
- Mô hình: `yolo11n-seg.pt` (pretrained COCO), ảnh 640 px, batch 8 (vừa 4 GB VRAM), 100–150 epoch, early stopping theo mask mAP trên tập val.
- Augmentation: xoay ±20°, độ sáng ±25%, mờ nhẹ, cắt kiện chạm mép. Không lật ngang nếu nhãn cần đọc chữ. Mosaic chỉ dùng ở các epoch đầu.
- Script mới: `scripts/train_parcel_seg.py`. Script này chạy train, xuất ONNX (`opset 17`, `imgsz 640`), ghi metrics vào `reports/phase2/`.

### 5.2 Từ mask ra cỡ kiện
1. Phóng mask về kích thước 2K, lấy `minAreaRect` để có cạnh dài L và cạnh ngắn W (px).
2. Tỉ lệ `mm/px = QR_mm / cạnh_QR_px`. QR nằm **trên mặt kiện**, nên tỉ lệ này đã tự tính tới chiều cao kiện, không cần camera 3D.
3. Phân cỡ bằng **khoảng cách gần nhất** tới (L, W) của 3 loại kiện trong config. Nếu kết quả nằm giữa hai loại, hoặc lệch quá 15% so với mọi loại → `UNKNOWN_SIZE` → MANUAL_REVIEW.
4. Trường hợp **không có QR**: dùng tỉ lệ mm/px tính từ độ cao camera và bảng hiệu chuẩn (bàn cờ hoặc thước đặt trên mặt băng tải), có độ cao kiện bù theo từng cỡ.

### 5.3 Lỗi viền kiện dựa trên mask mới
- Dùng lại các phép đo đã có: độ chữ nhật, độ sâu khía lõm (theo cạnh QR), lỗ bên trong mask. Lúc này mask đến từ YOLO, không còn từ ảnh nền.
- Hiệu chỉnh lại ngưỡng trên tập val bằng ROC, chọn điểm thỏa G3. Ghi ngưỡng vào config kèm ngày và số liệu.

## 6. Tích hợp vào ứng dụng (ước tính 1 buổi)
| Thành phần | Thay đổi |
|---|---|
| `services/parcel_segmenter.py` (mới) | Lớp `YoloParcelSegmenter` (onnxruntime) và `BackgroundParcelSegmenter` (logic `_segment_parcel` hiện tại), dùng chung một interface |
| `services/damage.py` | Nhận mask từ bộ phân vùng được chọn; phần kiểm tra rách nhãn giữ nguyên |
| `services/parcel_size.py` (mới) | Tính L, W (mm) và cỡ kiện |
| `models.py` | Thêm `ParcelData.size_class`, `size_mm`; ghi vào `result.json` |
| `services/classifier.py` | Chọn thùng đích theo cỡ kiện, ví dụ `BIN_SMALL/MEDIUM/LARGE`; kiện lỗi vẫn vào DEFECT |
| `config/default.yaml` | `parcel_segmentation.method: yolo \| background`, đường dẫn model, ngưỡng confidence; `parcel_sizes` với kích thước 3 loại |
| GUI | Hiện "CỠ KIỆN: VỪA (212 × 148 mm)" và viền mask lên ảnh kiểm tra |
| `ui/camera_worker.py` | Khung kiện ở chế độ live lấy từ YOLO, chạy mỗi N frame, thay cho khung ước lượng theo A6 |

**Phương án dự phòng:** nếu không có file model hoặc onnxruntime lỗi, hệ thống tự quay về `background` và ghi cảnh báo, không làm sập ứng dụng.

## 7. Đánh giá
- Script `scripts/evaluate_phase2.py` chạy trên tập test, xuất `reports/phase2/report.md`, gồm: mask IoU, ma trận nhầm lẫn của cỡ kiện, sai số mm, ROC của lỗi viền kiện, thời gian suy luận trên CPU và GPU, và các ảnh sai kèm overlay.
- Chạy lại toàn bộ test hiện có cùng các test mới: segmenter (dùng model giả), parcel_size (hình học tổng hợp), fallback khi thiếu model.
- **Thử tại trạm**: 30 kiện chạy thật qua GUI, mỗi cỡ 10 kiện, trong đó 3 kiện lỗi. So kết quả với ghi chép tay.

## 8. Thứ tự và thời lượng ước tính
| Bước | Việc | Ai | Thời lượng |
|---|---|---|---|
| 1 | Cung cấp kích thước kiện, cạnh QR, độ cao camera; dọn OpenCV; tạo `.venv-train` | Bạn + tôi | 0,5 buổi |
| 2 | Viết `capture_parcels.py` và script tạo mask nháp | Tôi | 0,5 buổi |
| 3 | Chụp khoảng 450 ảnh theo mục 3.2 | Bạn | 2–3 buổi |
| 4 | Tạo mask nháp, sửa trong X-AnyLabeling | Tôi tạo nháp, bạn sửa | 1 buổi |
| 5 | Train, xuất ONNX, chọn ngưỡng | Tôi | 1–2 buổi |
| 6 | Tích hợp vào ứng dụng, test, GUI | Tôi | 1 buổi |
| 7 | Đánh giá trên tập test và thử tại trạm | Bạn + tôi | 0,5–1 buổi |

**Mốc kiểm tra giữa chừng:** sau bước 4, train thử với khoảng 100 ảnh. Nếu mask IoU < 0,85 thì xem lại cách chụp hoặc gắn nhãn trước khi chụp tiếp.

## 9. Rủi ro và cách xử lý
| Rủi ro | Xử lý |
|---|---|
| Kiện to vượt khung hình ở độ cao hiện tại | Kiểm tra ở bước 1 bằng kích thước thật; nâng camera hoặc chấp nhận "kiện chạm mép → MANUAL_REVIEW" |
| Ít ảnh kiện lỗi | Lỗi viền kiện vẫn dùng phép đo hình học trên mask; YOLO chỉ cần học "đâu là kiện". Lỗi bề mặt để sang giai đoạn 3 (PatchCore) |
| Ổ E còn ít chỗ | Ảnh PNG 2K khoảng 3–4 MB/ảnh, 450 ảnh khoảng 1,6 GB; cộng `.venv-train` khoảng 4 GB. Dọn `build/` và các file trong `tmp/_review` trước |
| Mô hình học lệch theo ánh sáng hay nền của trạm | Chụp ở 2–3 thời điểm trong ngày; thêm augmentation độ sáng; ghi đèn và exposure vào JSON |
| CPU chậm hơn dự kiến | Giảm `imgsz` xuống 480; chạy YOLO trên ảnh đã thu nhỏ khi xem live; dùng GPU nếu có CUDA runtime |
