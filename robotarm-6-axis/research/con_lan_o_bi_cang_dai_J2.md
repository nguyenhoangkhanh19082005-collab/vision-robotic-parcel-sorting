# Con lăn ổ bi căng đai J2 (tì lưng đai) — nghiên cứu + kiểm tra hình học

Ngày tra cứu: 2026-11-20. Vai trò: researcher (+ kiểm tra hình học trên vỏ J2 gốc AX-1_3, bản lắp 1:27).

## Nguồn
- York Industries — finding the required belt width (quy tắc con lăn): https://www.york-ind.com/engineering/belts/finding-the-required-belt-width.html
  → đường kính con lăn KHÔNG nhỏ hơn puly nhỏ nhất của bộ truyền; giữ góc ôm trên con lăn nhỏ; ưu tiên con lăn cố định.
- SDP/SI — Timing Belt Design and Installation Suggestions: https://sdp-si.com/resources/beltdesignsuggestions.php
  → chỉ dùng con lăn khi cần; nếu dùng thì đặt ở nhánh chùng.
- Tyma FAQ — inner or outer idler: https://tyma.eu/technical-information/faq/is-it-better-to-use-an-inner-or-outer-idler-pulley/
  và bằng sáng chế US 7913479: https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/7913479
  → con lăn tì LƯNG đai (bên ngoài vòng đai) làm TĂNG góc ôm trên puly; đặt gần puly nhỏ càng tăng góc ôm puly nhỏ.
- Texas Belting — timing belt specifications: https://texasbelting.com/pages/timing-belt-specifications
  → tì lưng đai gây uốn ngược (thêm ứng suất) → không dùng con lăn quá nhỏ; góc ôm puly nhỏ ≥ 60°.
- Kích thước ổ bi có vành (NSK/SKF và nhà bán): F623ZZ 3×10×4, vành Ø11.5×1; F624ZZ 4×13×5, vành Ø15×1;
  F625ZZ 5×16×5, vành Ø18×1. https://www.oss.nsk.com/f624zzmc5-apn.html , https://www.oss.nsk.com/f625zzmc3-apn.html ,
  https://www.kugellager-express.de/miniature-flanged-ball-bearing-f-623-zz-3x10x4-mm

## Áp dụng cho J2
- Puly nhỏ J2 ≈ 20T (Ø bước ≈ 12.7 mm) → con lăn ≥ Ø13: **F623 (Ø10) loại**, **F624 (Ø13) tối thiểu đạt**, F625 (Ø16) tốt hơn nhưng chật hơn.
- J2 đảo chiều liên tục → nhánh chùng đổi theo chiều quay → dùng 2 con lăn đối xứng (giống ảnh tham khảo).
- Kiểm tra trong vỏ J2 (mặt phẳng đai thật w ≈ −1.5, ép 4 → 12 mm):
  - Đặt sát puly lớn như ảnh tham khảo (f ≤ 0.40) KHÔNG được: vành puly lớn Ø55 và vành ổ bi cùng mặt phẳng → chạm khi ép ≥ 8–12 mm, ép ít thì sát thành vỏ.
  - 2×F624ZZ: đạt từ f = 0.44; tại f = 0.50: cách vành puly lớn ≥ 4.7 mm, puly nhỏ ≥ 18 mm, thành/trụ vít ≥ 3 mm, đáy vỏ thấp hơn mặt đai ≈ 16–18 mm, phía trên tới nắp AX-1_4 ≈ 16 mm.
  - 2×F625ZZ: chỉ đạt từ f = 0.52, cách thành chỉ 2 mm khi ép ít.
  - Ép 12 mm tại giữa nhánh → đai dài thêm ≈ 4.0 mm/nhánh (mẩu đũa hiện tại ≈ 3 mm).

## Kết luận / đề xuất
2 con lăn, mỗi con = 2×F624ZZ ghép lưng (vành quay ra ngoài), trục vít M4, đặt ở f ≈ 0.50 hai nhánh, trụ đỡ in liền đáy vỏ,
rãnh trục dài 8 mm theo phương ép (nới vít → đẩy con lăn → siết). Bỏ 2 trụ căng cũ sát puly.
Hình: assets/Ranh_truot_cang_dai/con_lan_o_bi_J2_de_xuat.png. Script: E:\UDMT\.tmp\dai\j2_con_lan.py, j2_con_lan_tren.py.

## Cách ghép con lăn (làm rõ theo câu hỏi người dùng)
F624ZZ chỉ có 1 vành → mỗi con lăn dùng 2 ổ ghép lưng tựa lưng, vành quay RA NGOÀI (ổ trên vành ở trên, ổ dưới vành ở dưới)
→ 1 con lăn có 2 vành Ø15, khe giữa 2 vành = 2×5 − 2×1 = 8 mm (đai 6 mm, xê dịch ±1 mm). Tổng 4 ổ cho 2 con lăn.
Kẹp chỉ vào VÒNG TRONG: gờ Ø6.5 cao 0.5 mm trên đỉnh trụ + long đen mỏng (DIN 988 4×8×0.5 hoặc long đen M4 thường) dưới đầu vít.
Kiểm tra 3D ở dưới đã dùng đúng khối này (cao 10 mm, vành Ø15 ở 2 đầu).

## Kiểm tra 3D bổ sung (2026-11-20, sau ảnh chụp thực tế của người dùng)
Con lăn quét hết hành trình (ép 4→12 mm) + trụ đỡ hình chữ nhật bo 13 mm, so với mọi chi tiết bản lắp 1:27 (puly nhỏ +3 mm):
| f | trụ ↔ ổ bi chính 65x50x7 | con lăn ↔ thành/trụ vít | con lăn ↔ nắp AX-1_4 | con lăn ↔ puly 55 | trụ ↔ con thoi |
|---|---|---|---|---|---|
| 0.46 | CHẠM (chồng 8.5 mm³) | 2.3 | 2.8 | 3.5 | — |
| 0.48 | 0.7 | 3.0 | 3.4 | 4.7 | — |
| 0.50 | 1.9 | 3.8 | 4.1 | 5.9 | ≥ 7 |
| 0.52 | 3.1 | 4.6 | 4.9 | ≥ 6.9 | 5.9 |
Đáy vỏ dưới trụ dày 2.6–5 mm (mặt trên w ≈ −17.5…−19.3); khâu 2 (AX-2_5, quay) nằm dưới đáy, cách trụ ≥ 1.9 mm qua lớp đáy.
→ Vùng đặt được rất hẹp: f = 0.50–0.52 (dịch dọc đai chỉ ±1–2 mm). Chọn **f = 0.52** (cách điểm đai rời puly lớn ≈ 36.7 mm).
Ảnh: assets/Ranh_truot_cang_dai/con_lan_J2_khong_gian_3D.png. Script: E:\UDMT\.tmp\dai\j2_con_lan_3d.py, j2_con_lan_3d_b.py, j2_con_lan_render.py.
