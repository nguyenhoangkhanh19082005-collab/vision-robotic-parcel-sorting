# Đánh giá model có sẵn trên Hugging Face cho kiểm tra kiện (2026-11-07)

Mục tiêu: phát hiện hư hỏng **mà không phải đục thủng hay làm hỏng nhiều thùng** để lấy dữ liệu.

## 1. Đã cài đặt
| Thành phần | Chi tiết |
|---|---|
| Môi trường | `.venv-ml` (Python 3.12, torch 2.15 **CPU**, transformers 5.22, 1,1 GB). Tách riêng khỏi `.venv` của ứng dụng |
| Model (`models/hf`, 892 MB) | `facebook/dinov2-small` (22M), `facebook/sam2.1-hiera-tiny` (39M), `IDEA-Research/grounding-dino-tiny` (172M); đều Apache-2.0 |
| Công cụ | `scripts/ml/parcel_anomaly.py`: lệnh `build` dựng bộ nhớ từ ảnh kiện tốt, lệnh `score` chấm điểm và xuất heatmap kèm `scores.json` |
| Bộ nhớ mẫu | `models/parcel_anomaly_bank.npz`: 10 frame nhãn BN-99-02 ở trạm (chỉ dùng cho demo) |
| Sửa lỗi kèm theo | `convexityDefects` trả mảng khác chiều giữa OpenCV 4.x và 5.x (`label_integrity.py`, `damage.py`); 66/66 test vẫn đạt |

Cache của pip và Hugging Face đặt trong thư mục dự án trên ổ E; ổ E còn khoảng 4,8 GB.

## 2. Cách ghép các model
```
QR (zxing) ─► căn ảnh theo QR (xoay, tỉ lệ, dịch) ─► chuẩn hóa độ sáng theo giấy nhãn
   mask kiện: SAM2.1 (chấm 1 điểm vào QR → ra NHÃN)  hoặc
              Grounding DINO "a cardboard box" → khung → SAM2.1 (→ ra CẢ THÙNG)
   DINOv2-small: đặc trưng 14×14 px → khoảng cách tới ô gần nhất trong bộ nhớ kiện TỐT
   điểm = trung bình 1% ô cao nhất, trong bao lồi của mask (trừ vùng nhãn nếu dùng --exclude-label)
```
Đây là phương pháp AnomalyDINO: không train, chỉ cần ảnh kiện tốt. Bộ nhớ được bổ sung các biến thể làm mờ (σ = 1 và 2) và độ sáng ±12–15%, để chịu được việc độ nét và phơi sáng thay đổi theo vị trí kiện trên băng tải.

## 3. Kết quả

### 3.1 Tách kiện
| Cách | Kết quả | Thời gian (CPU, ảnh 1280×720) |
|---|---|---|
| SAM2.1, chấm 1 điểm vào tâm QR | 8/8 ảnh trạm: viền nhãn chính xác, kể cả chỗ rách; không dính băng tải, cảm biến hay dây. **Nhưng với thùng thật thì chỉ ra vùng nhãn** | khoảng 1,65 s |
| Grounding DINO + SAM2.1 | 5/5 (thùng chụp ngang 15/30/40 cm, kiện trên trạm): **mask cả thùng**, không dính tường, sàn hay thước | khoảng 10 s (Grounding DINO chiếm 8 s) |

### 3.2 Phát hiện bất thường, chấm cả nhãn (nhãn BN-99-02 đã có trong bộ nhớ)
| Tập ảnh | Điểm | Kết quả ở ngưỡng 0,33 |
|---|---|---|
| Kiện tốt (16, trong đó 1 ảnh ở phiên khác) | tối đa 0,21 | 0 báo nhầm |
| Nhãn rách thật (5) | 0,48–0,61, heatmap đỏ đúng chỗ khuyết | 5/5 |
| Lỗi tổng hợp: lỗ, vết bẩn, rách, băng keo (64) | tối thiểu 0,47 | 64/64 (AUROC 1,0) |
| **Nhãn khác, còn nguyên (BN-99-03, 3 ảnh)** | **0,42–0,50, cả nhãn đỏ** | **3/3 báo nhầm** |

→ **Phát hiện quan trọng:** bộ nhớ ghi nhớ **đúng mặt chữ** của nhãn đã thấy. Nhãn thật thì mỗi kiện một nội dung, nên **không được chấm vùng nhãn bằng phương pháp này**. Chỗ rách nhãn để cho phép đo hình học trong `label_integrity.py`, vốn không phụ thuộc nội dung chữ.

### 3.3 Phát hiện bất thường, chỉ chấm bề mặt carton (`--exclude-label --box-detector`)
Thùng thật trong dataset. Bộ nhớ dựng từ 8 ảnh ở 30 cm; thử trên ảnh ở 25, 35 và 40 cm, có vẽ thêm lỗi lên phần carton.

| Tập ảnh | Điểm | Kết quả ở ngưỡng 0,33 |
|---|---|---|
| Kiện tốt (12, ở 3 khoảng cách khác với lúc dựng bộ nhớ) | tối đa 0,125 | **0/12 báo nhầm** |
| Lỗ tổng hợp (12) | tối thiểu 0,335 | 12/12, AUROC 1,0 |
| Vết bẩn (12) | tối thiểu 0,434 | 12/12, AUROC 1,0 |
| Vết rách (12) | tối thiểu 0,412 | 12/12, AUROC 1,0 |

Heatmap chỉ đúng vị trí lỗi; băng keo vàng và phần carton bình thường không bị đánh dấu. **Lỗ có sẵn trên chính thùng này không bị báo**, vì ảnh thùng đó đã nằm trong bộ nhớ "kiện tốt". Vậy **chỉ dùng thùng thật sự lành** để dựng bộ nhớ.

### 3.4 Thời gian (CPU i5-11400H)
| Bước | Thời gian |
|---|---|
| DINOv2-small chấm một ảnh | 0,18–0,28 s |
| SAM2.1-tiny | khoảng 1,65 s |
| Grounding DINO tiny | khoảng 8 s |

Như vậy chỉ DINOv2 là chạy được trực tiếp trong lúc vận hành. Với GPU RTX 2050 (cần cài thêm torch bản CUDA, khoảng 2,5 GB), SAM2 và Grounding DINO dự kiến nhanh hơn khoảng 10 lần. Kết quả này chưa đo.

## 4. Giới hạn của lần đánh giá
- Lỗi trên carton là **tổng hợp** (vẽ lên ảnh). Lỗi thật như móp hay rách lớp giấy carton có thể khó phát hiện hơn.
- Mỗi tập chỉ có **một thùng** và **một nhãn**. Ngưỡng 0,33 mới là ước lượng ban đầu, phải hiệu chỉnh lại trên ảnh kiện tốt **giữ riêng**, gồm nhiều kiện ở nhiều vị trí.
- Ảnh thùng chụp **ngang**, không phải nhìn từ trên xuống như ở trạm.
- Các mẫu thử ở trạm hiện tại là nhãn dán gần kín một tấm bìa, nên chế độ carton chưa có gì để chấm (`NOT_SCORED`).

## 5. Đề xuất thay đổi kế hoạch giai đoạn 2 (`phase2_plan.md`)
1. **Không cần làm hỏng nhiều thùng.** Chụp **20–50 ảnh mỗi cỡ, toàn thùng lành**, ở nhiều vị trí và góc, rồi dựng bộ nhớ. Kiểm chứng bằng **3–5 thùng lỗi**: dùng lại một thùng bằng cách xoay hoặc lật, giả lập không phá thùng (giấy đen dán tròn giả lỗ, băng keo bong, vết bẩn), cộng thêm lỗi tổng hợp.
2. **Tự động gắn nhãn cho YOLO11n-seg:** Grounding DINO + SAM2 tạo mask cả thùng cho mọi ảnh đã chụp, không cần vẽ polygon bằng tay. Chạy offline, khoảng 10 s/ảnh, 450 ảnh mất khoảng 75 phút. YOLO11n-seg train xong sẽ chạy nhanh (khoảng 0,1 s), thay Grounding DINO + SAM2 khi vận hành.
3. **Khi vận hành:** YOLO11n-seg tách thùng (sau khi train) → DINOv2 chấm bề mặt carton (`--exclude-label`) → `label_integrity` kiểm tra rách nhãn → cỡ kiện tính từ mask, dùng cạnh QR làm thước. Dự kiến cộng thêm khoảng 0,3–0,5 s mỗi kiện trên CPU.
4. **Chưa nối vào GUI** cho tới khi có bộ nhớ dựng từ thùng thật của trạm. Khi nối thì dùng `onnxruntime`, hoặc gọi `.venv-ml` như một tiến trình riêng, để `.venv` chính không phải cài torch.

## 6. Thử trên kiện thật ở trạm (2026-11-10)
Kiện `SPXVN54506627407`, nền xám mới. Bộ nhớ dựng từ 6 ảnh lần chụp đầu, thử trên 10 ảnh lần chụp thứ hai (`tmp/_review/ml/station_test.py`).

| Thử nghiệm | Kết quả |
|---|---|
| SAM2.1, chấm điểm vào QR | Viền bám sát mặt trên kiện, 1,70 s |
| Grounding DINO + SAM2.1 | Viền gần như trùng (29% khung so với 29%), 8,9 s |
| DINOv2, kiện tốt (10 ảnh) | điểm tối đa 0,143, không báo nhầm ở ngưỡng 0,33 |
| DINOv2, lỗi vẽ thêm: lỗ, vết bẩn, rách (10 ảnh mỗi loại) | điểm tối thiểu 0,61; phát hiện 30/30; heatmap đúng vị trí |
| Chế độ chỉ chấm carton (`--exclude-label`) | **Không chấm được**: nhãn phủ kín mặt trên của kiện, không còn phần carton nào lộ ra |

Giới hạn: bộ nhớ và ảnh thử cùng một kiện, cùng một nhãn, kiện không dịch chuyển giữa hai lần chụp; lỗi là vẽ lên ảnh. Kết quả này chưa nói được gì về kiện có nhãn khác (mục 3.2).

## Nguồn
- AnomalyDINO (WACV 2025): https://openaccess.thecvf.com/content/WACV2025/html/Damm_AnomalyDINO_Boosting_Patch-Based_Few-Shot_Anomaly_Detection_with_DINOv2_WACV_2025_paper.html
- https://hf.co/facebook/dinov2-small
- https://hf.co/facebook/sam2.1-hiera-tiny
- https://hf.co/IDEA-Research/grounding-dino-tiny
- https://hf.co/SMU-MaLA-Lab/AnomalyCLIP
- https://hf.co/cvtechniques/PackageDamage
