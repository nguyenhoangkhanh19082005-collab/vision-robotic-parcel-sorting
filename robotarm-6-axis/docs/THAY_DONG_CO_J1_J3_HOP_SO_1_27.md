# Thay động cơ J1, J2, J3 bằng 42STH38-1684A + hộp số hành tinh 36JXF27G (1:27)

Ngày: 03/11/2026. Dữ liệu gốc: `E:\robot công nghiệp\Bài_thi_2026\Roboter.STEP` (chỉ đọc, không sửa).
Toàn bộ phân tích dùng mô hình CAD, chưa đo trên robot thật. Các số liệu cần kiểm tra lại được đánh dấu **[ĐO]**.

## 1. Động cơ mới so với động cơ đang dùng

| | Mới: 42STH38-1684A + 36JXF27G | J2, J3 hiện tại: 17HS13-0404S-PG5 | J1 hiện tại: Nema17 trơn |
|---|---|---|---|
| Tỉ số hộp số | 1:27 (2 cấp) | 1:5,18 (1 cấp) | không có |
| Mặt lắp | Ø35,9, **4×M2.5 trên vòng Ø28** | Ø36, 4×M3 trên vòng Ø28 | mặt NEMA 31×31, M3 |
| Trục ra | **vuông 6 mm**, dài 14,5 | tròn (CAD Ø6, thực tế Ø8 D), dài 18 | Ø5, dài 16,6 |
| Chiều dài từ mặt lắp ra đuôi | 34,1 + 39,5 = **73,6 mm** | 61,3 mm | khoảng 36 mm |
| Dòng định mức | 1,68 A, 1,6 Ω | 0,4 A | — |

## 2. Kết quả phân tích CAD (đặt động cơ mới vào vị trí cũ và kiểm tra va chạm)

- **J2:** lắp trực tiếp vào tấm gá `AX-1_6` có sẵn. Hốc Ø37 ôm hộp số Ø35,9, còn 4 lỗ Ø3,5 trên vòng Ø28
  dùng cho vít M2.5 + long đền. Không va chạm. Dải góc không va chạm khi quay J2 giữ **y hệt** như với PG5
  (đã quét −180° đến +180°, bước 10°).
- **J3:** cũng dùng `AX-1_6` như J2. Đuôi động cơ mới dài hơn 12,3 mm nên va vào thành sau của vỏ `AX-2_2`
  (khoảng 0,4 cm³). **Đã sửa:** khoét hốc trong `AX-2_2` với khe hở 1,5 mm. Thành ngoài còn khoảng 10 mm.
  Dải góc J3 không đổi so với PG5.
- **J1:** mặt hộp số áp vào mặt dưới tấm gá `AX-1_11`, đúng vị trí mặt bích Nema17 cũ, nên puly giữ nguyên
  cao độ.
  - **Đã sửa:** thêm 4 lỗ Ø3,0 cho vít M2.5 trên vòng Ø28 (các lỗ NEMA cũ giữ nguyên).
  - Động cơ mới dài hơn nên **thò 37,1 mm dưới đáy đế `AX-1_10`**. Đã tạo **đế nâng 43 mm**
    (`De_nang_J1`): ôm đúng biên dạng đáy, thành dày 6 mm, 4 tai bắt vít M4 xuống bàn.
- Chỉ những chi tiết trên là phải in lại. Tất cả các khớp khác, ổ bi và puly lớn giữ nguyên.

### Phát hiện cần kiểm tra ngay (ảnh hưởng cả robot hiện tại)
CAD cho thấy J1, J2, J3 đều truyền động **qua đai**. Puly nhỏ có rãnh Ø12, nhiều khả năng là GT2 20 răng.
Từ đường kính rãnh của puly lớn, ước lượng số răng và tỉ số như sau:

| Khớp | Puly lớn (rãnh) | Tỉ số đai ước lượng | Tổng (CAD) | Firmware đang đặt |
|---|---|---|---|---|
| J1 | Ø37,6 → khoảng 60 răng | 3,0 | 3,0 | **1,0** |
| J2 | Ø50,5 → khoảng 80 răng | 4,0 | 5,18 × 4 = 20,7 | **5,18** |
| J3 | Ø37,6 → khoảng 60 răng | 3,0 | 5,18 × 3 = 15,5 | **5,18** |
| J5 | Ø37,6 / 20 răng | 3,0 (đo thực tế trước đây ra khoảng 2,5) | | 2,5 |

**[ĐO]** Đếm số răng của puly lớn và puly nhỏ, hoặc lệnh một khớp quay +30° rồi đo góc thật. Nếu CAD đúng
thì robot hiện tại đang quay J1, J2, J3 ít hơn lệnh từ 3 đến 4 lần.

## 3. Khuyến nghị trước khi mua
1. **Trục ra vuông 6 mm là vấn đề lớn nhất.** Puly GT2 20 răng bán sẵn chỉ có lỗ tròn 5, 6,35 hoặc 8 mm.
   - Đo đường chéo trục: **[ĐO]** nếu khoảng 8,0 mm (góc bo tròn) thì dùng puly GT2 20T lỗ 8 mm, hai vít
     trí siết lên hai mặt phẳng.
   - Nếu là vuông sắc (đường chéo khoảng 8,5 mm) thì nên **hỏi shop bản hộp số trục Ø8 có mặt D**. Dòng 36 mm
     này thường có bản đó.
2. **Tỉ số:** vì đã có đai 3–4 lần, hộp số 1:27 cho tổng tỉ số 81–108.
   - Ở 12 V, động cơ chạy êm khoảng 3 vòng/giây, nên J2 đạt khoảng 10°/s và J1, J3 khoảng 13°/s. Tốc độ này
     tương đương robot đang chạy hiện nay.
   - Muốn nhanh gấp đôi mà mô-men vẫn dư thì chọn bản **1:14** cùng dòng: cùng mặt lắp, cùng chiều dài 2 cấp,
     nên các chi tiết đã sửa vẫn dùng được.
   - J1 chỉ xoay đế nên bản 1:5,18 (1 cấp, ngắn hơn) cũng đủ. Khi đó đế nâng thấp hơn: chạy lại script với
     `GEAR_L` mới.
3. **Giới hạn thật nằm ở đai, không ở động cơ.** Đai GT2 rộng 6 mm (rãnh puly CAD 7,5 mm) với puly 80 răng
   chỉ truyền được khoảng 2–2,5 N·m ở J2 trước khi bị nhảy răng. Muốn tận dụng mô-men của hộp số thì cần
   đai rộng 9–10 mm, căng đai đúng, hoặc thêm lò xo trợ lực cho J2.
4. **Khối lượng:** mỗi động cơ mới nặng khoảng 0,5 kg. Động cơ J3 nằm trên khâu 2, nên mô-men tải lên J2
   tăng thêm một chút.

## 4. File đã tạo (`assets/CAD_dong_co_1_27/`, cùng hệ tọa độ với Roboter.STEP, đơn vị mm)
| File | Dùng để |
|---|---|
| `Roboter_dong_co_1_27.step` | Lắp ráp hoàn chỉnh: chi tiết gốc + 3 chi tiết đã sửa + 3 động cơ mới, có tên từng component |
| `AX-2_2_sua_J3.step/.stl` | In lại vỏ khâu 2 |
| `AX-1_11_sua_J1_M2.5.step/.stl` | In lại tấm gá động cơ J1 |
| `De_nang_J1.step/.stl` | In mới đế nâng 43 mm |
| `Dong_co_42STH38_hop_so_1_27.step/.stl` | Mô hình động cơ mới, đơn giản hóa, dùng để lắp thử |
| `ket_qua_kiem_tra.txt` | Kết quả kiểm tra va chạm |

Ảnh kiểm tra nằm trong `reports/cad_dong_co_1_27/`. Script tạo lại các file trên:
`src/cad/tao_chi_tiet_dong_co_1_27.py`. Khi có số đo thật thì sửa các hằng số `GEAR_L`, `MOTOR_L`,
`SHAFT_L` và `CLEAR` ở đầu file rồi chạy lại:
```
E:\UDMT\.venv\Scripts\python.exe E:\UDMT\robotarm-6-axis\src\cad\tao_chi_tiet_dong_co_1_27.py
```

## 5. Làm tiếp trên Fusion 360
1. **Mở lắp ráp:** File → Open → Open from my computer → chọn `Roboter_dong_co_1_27.step` (73 MB, Fusion tải
   lên và chuyển đổi mất vài phút). Các component mang tên `..._sua` và `Dong_co_moi_J1/J2/J3`.
2. **Kiểm tra:**
   - Inspect → **Interference**, chọn động cơ mới và các chi tiết quanh nó. Kết quả phải không có va chạm,
     trừ trục với puly 16 mm, vì lỗ puly vẫn là lỗ cũ.
   - Inspect → **Section Analysis** để xem hốc khoét trong `AX-2_2`.
   - Quay J2, J3 bằng cách vào Assemble → Joint/Drive Joints. Nếu muốn quay thử thì cần tạo Revolute joint
     quanh trục ổ bi.
3. **Chỉnh sửa thêm:** các chi tiết nhập từ STEP không có lịch sử dựng hình. Chọn body rồi dùng
   **Press Pull** để chỉnh mặt hoặc khe hở, và **Hole** để đổi cỡ lỗ, ví dụ Ø2,9 hoặc Ø3,1 tùy máy in.
   Muốn sửa theo tham số thì sửa script ở mục 4 rồi Insert lại file STEP.
4. **Xuất file in:** chuột phải vào body → **Save As Mesh** → STL, đơn vị mm, độ mịn High. Có thể in thẳng
   các file `.stl` đã có. Chúng nằm theo tư thế lắp ráp nên cần đặt lại hướng trong slicer:
   - `De_nang_J1`: đặt mặt có tai xuống bàn in.
   - `AX-1_11`: đặt mặt tấm gá xuống bàn in.
   - `AX-2_2`: in theo hướng của bản cũ.
5. **Thông số in đề xuất:** PETG hoặc PLA+, 4 lớp thành, infill 40% gyroid. Riêng `AX-1_11` (chịu lực đai)
   và `De_nang_J1` nên dùng infill 50% trở lên.

## 6. Khi lắp động cơ mới (firmware, giao diện)
- Firmware `GEAR_RATIOS`: J1 = 27 × (tỉ số đai J1), J2 = 27 × (đai J2), J3 = 27 × (đai J3). Theo CAD là 81,
  108 và 81. **[ĐO]** tỉ số đai trước khi đặt.
- `MOTOR_CURRENTS` cho J1–J3: 1100–1200 mA RMS. TMC2209 phải có tản nhiệt và quạt. Nguồn 12 V nên từ 5 A
  trở lên.
- Nên đổi vi bước J1–J3 xuống 8. Hộp số đã chia rất mịn (khoảng 0,0075° mỗi vi bước), và tần số xung giảm
  một nửa.
- Station: đế robot cao thêm 43 mm so với mặt bàn, nên `layout.conveyor.base_z` trong `station.yaml` giảm
  0,043 m nếu băng tải đặt cùng mặt bàn.
- Vít: M2.5 × 10 có long đền cho J2, J3 (tấm 5 mm). M2.5 × 12 cho J1.
  **[ĐO]** chiều sâu lỗ ren trên hộp số để vít không chạm đáy.
