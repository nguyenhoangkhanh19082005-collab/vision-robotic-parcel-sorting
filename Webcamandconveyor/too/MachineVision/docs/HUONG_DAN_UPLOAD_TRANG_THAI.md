# Hướng dẫn upload trạng thái kiện hàng lên trạm dữ liệu (Supabase)

Cập nhật: 2026-11-10. Áp dụng cho project Supabase `<project-id>` và phần mềm
MachineVision trong thư mục này.

## 1. Luồng hoạt động

```
Kiện đi qua webcam
   └─ Phần mềm kiểm tra 3 thứ: QR, logo trên tem, tình trạng bề mặt (tốt / móp méo / thủng)
        └─ Đọc bảng orders theo mã vận đơn trong QR  → biết sọt đích
             └─ Ghi 1 dòng vào bảng sorting_logs      → kết quả của lần kiểm tra
                  └─ Database tự cập nhật đơn hàng (trigger sorting_logs_apply)
                       └─ Đội quản lý xem trên Supabase (v_order_tracking, ...)
```

Máy trạm **chỉ ghi vào `sorting_logs`**. Máy trạm không sửa bảng `orders`; việc đổi trạng
thái đơn do database tự làm.

## 2. Quy tắc phân loại và dữ liệu được ghi

| Kết quả kiểm tra | `decision` | Sọt (`chute_id`) | `status` của log | Đơn hàng (`orders`) |
|---|---|---|---|---|
| QR đọc được, có logo, kiện tốt, mã có trong hệ thống | `PASS` | Sọt đích của đơn (1–5) | `SORTED` | `status` → `SORTED`, điền `sorted_at` |
| Kiện móp méo / thủng | `REJECT` | Sọt lỗi (0) | `REJECTED` | Giữ nguyên |
| Tem rách, thiếu logo, không đọc được QR | `REJECT` | Sọt lỗi (0) | `REJECTED` | Giữ nguyên |
| Mã vận đơn không có trong hệ thống | `REJECT` | Sọt lỗi (0) | `REJECTED` | Không có đơn để cập nhật |
| Bộ kiểm tra chưa hiệu chuẩn, không kết luận được | `MANUAL_REVIEW` | Sọt lỗi (0) | `MANUAL_REVIEW` | Giữ nguyên |

Các cột của một dòng `sorting_logs`:

| Cột | Ý nghĩa | Ví dụ |
|---|---|---|
| `tracking_code` | Mã vận đơn đọc từ QR; `UNKNOWN` nếu không đọc được | `SPXVN40228347634` |
| `order_id` | Đơn tương ứng; database tự điền nếu để trống | `8` |
| `chute_id` | Sọt thực tế kiện được đưa vào | `1` |
| `status` | Trạng thái của lần quét | `SORTED` |
| `decision` | `PASS`, `REJECT` hoặc `MANUAL_REVIEW` | `PASS` |
| `reason` | Lý do của quyết định | `Phát hiện lỗi: hole_or_puncture` |
| `qr_ok` | Đọc được QR | `true` |
| `logo_ok` | Có logo trên tem; để trống nếu chưa kiểm tra được | `true` |
| `damage_detected` | Kiện bị hư hỏng; để trống nếu chưa kiểm tra được | `false` |
| `damage_types` | Loại hư hỏng: `deformation` (méo), `dent_or_edge_tear` (móp/rách mép), `hole_or_puncture` (thủng), `irregular_outline`, `label_tear` (rách tem) | `{hole_or_puncture}` |
| `station_id` | Tên trạm ghi dòng này | `VISION-01` |
| `scanned_from` | Nguồn quét | `MACHINE_VISION` |
| `evidence_path` | Thư mục ảnh bằng chứng trong `captures/` trên máy trạm | `2026-11-10/20261110T...` |
| `inspection_uuid` | Mã duy nhất của lần kiểm tra; chống ghi trùng khi gửi lại | (tự sinh) |
| `created_at` | Thời điểm ghi, giờ UTC; database tự điền | (tự sinh) |

## 3. Cài đặt một lần trên máy trạm

**Bước 1. Cài thư viện Supabase** (chạy trong thư mục MachineVision):

```bash
.venv\Scripts\python.exe -m pip install -e ".[database]"
```

**Bước 2. Lấy URL và key.** Trong Supabase Dashboard mở **Project Settings → API**:

- `Project URL`: `https://<project-id>.supabase.co`
- Key: hiện tại database chưa bật RLS nên key nào cũng chạy được. Sau khi nhóm bật RLS
  (Phần B của `docs/supabase_bo_sung_2026-11-10.sql`) thì **bắt buộc dùng secret /
  service_role key**. Nên dùng luôn key này từ đầu để không phải đổi lại.

**Bước 3. Tạo file `.env`** ở thư mục gốc MachineVision (cạnh `pyproject.toml`; với bản
EXE thì đặt cạnh file `.exe`). Sao chép từ `.env.example` rồi điền:

```
SUPABASE_URL=https://<project-id>.supabase.co
SUPABASE_KEY=<key của máy trạm>
```

Không gửi file `.env` qua nhóm chat, không commit lên Git, không dùng key này trong mã
trang web.

**Bước 4. Bật kết nối** trong `config/default.yaml`:

```yaml
classification:
  local_test_mode: false     # trước đây: true

supabase:
  enabled: true              # trước đây: false
```

Các mục còn lại của phần `supabase` đã đặt sẵn đúng với database (bảng `orders`, cột
`tracking_code`, sọt lỗi `0`). Đổi `station_id` nếu có nhiều hơn một trạm.

## 4. Vận hành hằng ngày

1. Mở phần mềm. Ô trạng thái **SUPABASE** ở đầu cửa sổ hiện `READY`.
2. Đặt kiện dưới webcam và bấm kiểm tra như bình thường.
3. Sau mỗi lần kiểm tra, ô **SUPABASE** chuyển sang:
   - `ONLINE`: đã ghi kết quả lên trạm dữ liệu.
   - `OFFLINE`: chưa gửi được (rê chuột vào ô để xem lỗi). Kết quả đã được lưu chờ.
4. Không cần thao tác gì thêm. Mỗi lần kiểm tra tạo đúng một dòng trong `sorting_logs`.

**Khi mất mạng:** kết quả được lưu vào `captures/supabase_outbox.jsonl`. Lần kiểm tra kế
tiếp gửi thành công thì các kết quả đang chờ được gửi lại theo đúng thứ tự, sau đó file
này tự xóa. Không xóa file này bằng tay khi nó còn dữ liệu.

**Lưu ý:** nếu `local_test_mode: true` thì phần mềm không tra cứu và không ghi gì lên
Supabase, ô trạng thái hiện `BYPASSED`.

## 5. Xem trạng thái trên trạm dữ liệu

Trong Supabase Dashboard mở **Table Editor** và chọn:

| Tên | Dùng để |
|---|---|
| `v_order_tracking` | Mỗi đơn một dòng: trạng thái, sọt dự kiến, sọt thực tế, kết quả kiểm tra gần nhất |
| `v_chute_summary` | Mỗi sọt: tổng số đơn, đã phân loại, đang chờ |
| `v_sorting_exceptions` | Các kiện bị loại hoặc cần kiểm tra lại, mới nhất ở trên |
| `sorting_logs` | Toàn bộ lịch sử từng lần quét |

Ví dụ truy vấn trong **SQL Editor**:

```sql
-- Kiện bị loại hôm nay và lý do
select created_at, tracking_code, reason, damage_types
from v_sorting_exceptions
where created_at >= current_date;

-- Kiện vào sai sọt (sọt thực tế khác sọt dự kiến)
select tracking_code, planned_chute_id, actual_chute_id, last_reason
from v_order_tracking
where last_decision = 'PASS' and actual_chute_id <> planned_chute_id;
```

## 6. Upload thủ công hoặc từ thiết bị khác

Dùng khi cần ghi bù một kiện, hoặc khi một thiết bị khác (ví dụ ESP32, script riêng) cần
báo trạng thái. Chỉ cần thêm một dòng vào `sorting_logs`; ba cột bắt buộc là
`tracking_code`, `chute_id`, `status`.

**Cách 1: SQL Editor**

```sql
insert into sorting_logs
    (tracking_code, chute_id, status, decision, reason,
     qr_ok, logo_ok, damage_detected, station_id, scanned_from)
values
    ('SPXVN40228347634', 1, 'SORTED', 'PASS', 'Ghi bù thủ công',
     true, true, false, 'MANUAL', 'MANUAL');
```

**Cách 2: REST API** (thay `<KEY>` bằng key của máy trạm):

```bash
curl -X POST "https://<project-id>.supabase.co/rest/v1/sorting_logs" -H "apikey: <KEY>" -H "Authorization: Bearer <KEY>" -H "Content-Type: application/json" -d "{\"tracking_code\":\"SPXVN40228347634\",\"chute_id\":0,\"status\":\"REJECTED\",\"decision\":\"REJECT\",\"reason\":\"Kien thung\",\"qr_ok\":true,\"logo_ok\":true,\"damage_detected\":true,\"damage_types\":[\"hole_or_puncture\"],\"station_id\":\"MANUAL\"}"
```

**Cách 3: Python**

```python
from supabase import create_client

client = create_client(SUPABASE_URL, SUPABASE_KEY)
client.table("sorting_logs").insert({
    "tracking_code": "SPXVN40228347634",
    "chute_id": 1,
    "status": "SORTED",
    "decision": "PASS",
    "qr_ok": True, "logo_ok": True, "damage_detected": False,
    "station_id": "MANUAL",
}).execute()
```

Trong cả ba cách, ghi `decision = 'PASS'` thì đơn hàng tự chuyển sang `SORTED`.

## 7. Lỗi thường gặp

| Hiện tượng | Nguyên nhân | Cách xử lý |
|---|---|---|
| Phần mềm báo `SUPABASE_URL and SUPABASE_KEY are required` khi mở | Thiếu file `.env` hoặc đặt sai chỗ | Làm lại Bước 3 |
| Báo `supabase is not installed` | Chưa cài thư viện | Làm lại Bước 1 |
| Ô SUPABASE hiện `BYPASSED` | Đang ở chế độ thử cục bộ | Đặt `local_test_mode: false` |
| Mọi kiện đều bị loại với lý do `Waybill not found in station database` | Mã trong QR không khớp `orders.tracking_code`, hoặc key không đọc được bảng `orders` | Kiểm tra mã trên tem; sau khi bật RLS phải dùng secret key |
| Ô SUPABASE hiện `OFFLINE`, lỗi có `401` hoặc `permission denied` | Sai key, hoặc đã bật RLS nhưng vẫn dùng anon key | Đổi sang secret key trong `.env` |
| Ô SUPABASE hiện `OFFLINE`, lỗi có `violates check constraint` | `decision` không thuộc `PASS` / `REJECT` / `MANUAL_REVIEW` | Sửa giá trị gửi lên |
| Đơn không chuyển sang `SORTED` dù đã quét | Kết quả không phải `PASS`, hoặc mã vận đơn không có trong `orders` | Xem `v_sorting_exceptions` |

## 8. Việc chưa làm

- **RLS chưa bật.** Database vẫn mở cho bất kỳ ai có anon key. Người có quyền trong nhóm
  cần chạy Phần B của `docs/supabase_bo_sung_2026-11-10.sql`.
- **Giá trị `sorting_logs.status`.** Trang quản lý của backend có đếm `sorting_logs` theo
  `status`, nhưng chưa rõ nó đếm giá trị nào. Nếu số liệu trên trang đó không tăng, đổi
  `supabase.status_map` trong `config/default.yaml` cho khớp với backend.
- **Ảnh bằng chứng** vẫn nằm trên máy trạm (`captures/`); `evidence_path` chỉ ghi tên thư mục.
- **Khổ tem A7.** Cột `orders.label_size` đã có (mặc định `A6`), nhưng mẫu in ở backend
  chưa đổi và phần mềm chưa được thử với tem A7 thật.
- **Chưa chạy thử với webcam và key thật.** Phần ghi dữ liệu mới được kiểm tra bằng test
  tự động và bằng một lần ghi thử trên database (đã hủy sau khi thử).

## 8. Cập nhật 2026-11-25: chạy tự động với băng tải

Đã làm sẵn trên máy trạm, bạn không cần làm lại:
- Cài `supabase` 2.34 và `python-dotenv` vào `E:\UDMT\.venv` (môi trường của `run_station.bat`).
- `config/default.yaml`: `supabase.enabled: true`, `classification.local_test_mode: false`.
- `run_station.bat` (SortingStation) có gửi kết quả lên web. Trước bản này trạm tích hợp không gửi gì.

**Việc còn lại duy nhất: tạo file `.env`.** Đặt file ở `E:\UDMT\Webcamandconveyor\too\MachineVision\.env`, có 2 dòng:
```
SUPABASE_URL=https://<project-id>.supabase.co
SUPABASE_KEY=<key máy trạm>
```
Khi chưa có file này, phần mềm vẫn mở và chạy ở chế độ kiểm tra cục bộ. Ô SUPABASE báo "chưa cấu hình", rê chuột vào ô để xem lý do.

Cách chạy khi đã có `.env`:
| Việc | Cách làm |
|---|---|
| Tra cứu đơn | Danh sách đơn (chỉ cột định tuyến, không có tên hay số điện thoại) tải về khi mở phần mềm, làm mới mỗi 60 s (`cache_refresh_s`). Mất mạng ngắn vẫn tra được. |
| Mã không có trong bản sao | Hỏi Supabase một lần, chờ tối đa 4 s. Không hỏi được thì kiện thành MANUAL_REVIEW ("Không tra cứu được đơn…"), không bị REJECT oan là "mã lạ". |
| Gửi kết quả | Ghi vào `captures/supabase_outbox.jsonl` trước, rồi luồng nền gửi theo thứ tự. Băng tải không bao giờ chờ mạng. |
| Mất mạng | Kết quả nằm chờ trong outbox, tự thử lại mỗi 10 s (`retry_s`). Tắt máy giữa chừng thì lần mở sau gửi tiếp. |
| Ô SUPABASE trên GUI | `ONLINE` (đã gửi), `OFFLINE · n chờ gửi` (mất mạng), `READY` (chưa gửi kết quả nào). Rê chuột để xem chi tiết. |
| Đơn trên web | Kiện PASS: trigger `sorting_logs_apply` chuyển đơn sang `SORTED` và ghi `sorted_at`. Đã kiểm trên database thật 25/11 bằng giao dịch có rollback. Kiện REJECT / MANUAL_REVIEW: đơn giữ nguyên trạng thái, kết quả xem ở `v_sorting_exceptions`. |

