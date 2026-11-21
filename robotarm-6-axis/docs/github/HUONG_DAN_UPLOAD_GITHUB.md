# Hướng dẫn tự đưa toàn bộ hệ thống lên GitHub (21/11/2026)

Hướng dẫn này đưa 3 phần (MachineVision, robotarm-6-axis, SortingStation) vào **một repository chung**, giữ nguyên cấu trúc thư mục để SortingStation chạy được ngay sau khi clone.

Dữ liệu được **chép sang một thư mục riêng** rồi mới tạo git. Nhờ vậy thư mục làm việc `E:\UDMT` không bị đụng tới, kể cả repo cũ `robotarm-6-axis` (remote `github.com/Phucreat/robotarm-6-axis`).

## 0. Chốt với nhóm trưởng trước khi làm

1. **URL repository.** Nếu nhóm đã có repo thì dùng repo đó; không tự tạo repo mới. Nếu chưa có, bạn tạo trên github.com và mời nhóm trưởng vào.
2. **Public hay private.** Khuyên để **private**, vì `MachineVision/docs/HUONG_DAN_UPLOAD_TRANG_THAI.md` có URL project Supabase.
3. **Nhánh đẩy lên.** Repo trống thì dùng `main`. Repo đã có lịch sử thì tạo nhánh riêng, ví dụ `toan-he-thong`, rồi mở Pull Request. **Không force-push.**
4. Repo cũ `Phucreat/robotarm-6-axis` có cần cập nhật luôn không? Xem mục 7.

## 1. Đã chuẩn bị sẵn (không cần làm lại)

- `robotarm-6-axis/.gitignore`: thêm luật bỏ `.tmp/`, `.claude/`, `research/cad_cache*`, `*.bak` và 7 file lắp ráp STEP 70–76 MB. GitHub cảnh báo khi file > 50 MB và chặn khi > 100 MB.
- Lắp ráp cuối được nén thành `robotarm-6-axis/assets/CAD_lap_rap_v3/Roboter_v3_J4_cu_bau_ngan_giac_hut_dai.step.zip` (15 MB).
- `SortingStation/.gitignore`: mới.
- `Webcamandconveyor/too/MachineVision/.gitignore`: phiên Vision đã chuẩn bị. File này bỏ `.env`, ảnh nhãn, `dataset/`, `models/hf`, `tmp/`, `exports/`.
- Bản nháp README cho repo tổng: `robotarm-6-axis/docs/github/README_repo_tong.md`.

Dung lượng ước tính: robotarm khoảng 470 MB (nhiều nhất là STL/STEP chi tiết in), SortingStation 7 MB, MachineVision khoảng 1–2 MB. Không file nào vượt 50 MB, nên không cần Git LFS.

## 2. Khai báo tên Git (chỉ làm lần đầu)

Máy hiện chưa đặt tên/email cho Git. Nên dùng email *noreply* của GitHub (Settings → Emails) để không lộ email cá nhân.

```powershell
git config --global user.name "Ten Cua Ban"
```
```powershell
git config --global user.email "ID+username@users.noreply.github.com"
```

## 3. Chép 3 phần sang thư mục upload

Sao chép bỏ qua môi trường ảo, cache, dữ liệu cá nhân và file STEP lớn. Chạy cả khối sau trong **PowerShell**:

```powershell
$DST = "E:\UDMT\_github\he-thong-phan-loai"
$XD  = ".git",".venv",".venv-ml",".tmp","tmp",".pytest_cache",".ruff_cache",".runtime","__pycache__","cad_cache","cad_cache_moi",".claude","models","dataset","captures","exports","build","work"
$XF  = ".env","Roboter_*.step","*.bak"
robocopy "E:\UDMT\robotarm-6-axis" "$DST\robotarm-6-axis" /E /XD $XD /XF $XF /NFL /NDL /NP
robocopy "E:\UDMT\SortingStation" "$DST\SortingStation" /E /XD $XD /XF $XF /NFL /NDL /NP
robocopy "E:\UDMT\Webcamandconveyor\too\MachineVision" "$DST\Webcamandconveyor\too\MachineVision" /E /XD $XD /XF $XF /NFL /NDL /NP
Copy-Item "E:\UDMT\robotarm-6-axis\docs\github\README_repo_tong.md" "$DST\README.md"
Set-Content -Encoding utf8 "$DST\.gitignore" ".venv/`n__pycache__/`n*.py[cod]`n"
```

robocopy trả mã 0–7 là thành công. Mã từ 8 trở lên là lỗi.

## 4. Tạo git và rà soát TRƯỚC khi commit

```powershell
cd E:\UDMT\_github\he-thong-phan-loai
```
```powershell
git init -b main
```
```powershell
git add .
```

Rà soát. Mỗi lệnh phải cho kết quả đúng như ghi chú:

```powershell
git status --short | Select-String "\.env$|dataset/|captures/|/tmp/|\.venv"
```
→ phải **không in ra dòng nào**.

```powershell
git grep --cached -nE "eyJ[A-Za-z0-9_-]{20,}|sb_secret|service_role"
```
→ phải **không in ra dòng nào** (không lọt khóa Supabase).

```powershell
git ls-files | Select-String "MachineVision/.*\.(png|jpg|jpeg)$"
```
→ chỉ được có `assets/logo/*.png` và `hardware/*.png`. Ảnh có nhãn thật (họ tên, địa chỉ) tuyệt đối không được lọt.

```powershell
git ls-files | ForEach-Object { Get-Item $_ } | Where-Object Length -gt 50MB
```
→ phải **không in ra file nào**.

Nếu lọt file không muốn đưa lên, bỏ nó khỏi danh sách bằng `git rm --cached <đường_dẫn>`, thêm luật vào `.gitignore` tương ứng rồi `git add .` lại.

## 5. Commit theo từng phần (lịch sử dễ đọc cho báo cáo)

```powershell
git reset
```
```powershell
git add Webcamandconveyor
git commit -m "Vision: QR, logo, kiem tra hu hong, GUI, firmware bang tai ESP32"
```
```powershell
git add robotarm-6-axis
git commit -m "Robot 6 truc: firmware ESP32-S3/TMC2209, GUI + digital twin, dong hoc tu CAD, CAD in 3D"
```
```powershell
git add SortingStation README.md .gitignore
git commit -m "Tram hop nhat: digital twin toan tram, tuan tu phan loai 3 thung, README tong"
```
```powershell
git count-objects -vH
```
→ `size-pack` khoảng 400–500 MB là hợp lý.

## 6. Đẩy lên GitHub

**Repo trống:**

```powershell
git remote add origin https://github.com/<to-chuc-hoac-ten>/<ten-repo>.git
```
```powershell
git config http.postBuffer 524288000
```
```powershell
git push -u origin main
```

Lần push đầu, Git Credential Manager mở trình duyệt để đăng nhập GitHub; bạn tự đăng nhập. Push khoảng 500 MB có thể mất 5–20 phút tùy mạng. `http.postBuffer` giúp tránh lỗi `RPC failed` khi đẩy dữ liệu lớn qua HTTPS.

**Repo đã có lịch sử:** đẩy lên nhánh riêng rồi mở Pull Request trên web.

```powershell
git remote add origin https://github.com/<to-chuc-hoac-ten>/<ten-repo>.git
```
```powershell
git push -u origin main:toan-he-thong
```

## 7. (Tùy chọn) Cập nhật repo cũ `Phucreat/robotarm-6-axis`

Repo cũ mới có commit đầu tiên. Phần CAD, động học và firmware từ tháng 11 chưa có trên đó. Nếu nhóm muốn giữ repo riêng cho robot, bạn chỉ cần có quyền ghi và làm trong `E:\UDMT\robotarm-6-axis`:

```powershell
cd E:\UDMT\robotarm-6-axis
git switch -c cap-nhat-thang-11
git add .
git status
git commit -m "Cap nhat thang 11: dong co 1:27, CAD in 3D, dong hoc tu STEP, ga giac hut dai"
git push -u origin cap-nhat-thang-11
```

Sau đó mở Pull Request vào `main` trên GitHub. Nếu không có quyền ghi, nhờ chủ repo thêm bạn làm collaborator.

## 8. Kiểm tra sau khi đẩy

- Trên GitHub: không có `.env`, không có ảnh nhãn; README hiển thị ở trang đầu.
- Clone thử về thư mục khác, làm theo phần Cài đặt trong README, rồi chạy `pytest` trong `SortingStation` (22 test) và `MachineVision` (75 test).
- Bật **RLS** cho các bảng Supabase, dù repo public hay private.
