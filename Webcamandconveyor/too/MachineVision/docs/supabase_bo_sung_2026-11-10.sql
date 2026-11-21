-- =====================================================================
-- BO SUNG CO SO DU LIEU SUPABASE CHO TRAM PHAN LOAI
-- Project: <project-id>        Cap nhat: 2026-11-10
--
-- PHAN A  DA CHAY ngay 2026-11-10 (migration "sorting_station_inspection_fields",
--         xem trong Supabase > Database > Migrations). Chep lai o day de doi chieu.
-- PHAN B  CHUA CHAY: bat RLS. Nguoi co quyen trong nhom chay trong SQL Editor.
-- =====================================================================


-- =====================================================================
-- PHAN A: DA CHAY (khong can chay lai)
-- =====================================================================

-- A.1 Kho tem cua don hang. Mac dinh A6 nen don cu va backend cu khong doi.
alter table public.orders
    add column if not exists label_size varchar not null default 'A6'
        constraint orders_label_size_check check (label_size in ('A6', 'A7', 'A8')),
    add column if not exists label_printed_at timestamptz;

-- A.2 Ket qua kiem tra cua webcam (QR, logo, tinh trang be mat kien).
alter table public.sorting_logs
    add column if not exists inspection_uuid uuid,      -- ma duy nhat cua lan kiem tra
    add column if not exists decision varchar
        constraint sorting_logs_decision_check
        check (decision in ('PASS', 'REJECT', 'MANUAL_REVIEW')),
    add column if not exists reason text,               -- ly do phan loai / ly do loai
    add column if not exists qr_ok boolean,             -- doc duoc QR
    add column if not exists logo_ok boolean,           -- tem con nguyen, co logo
    add column if not exists damage_detected boolean,   -- kien mop, meo, thung
    add column if not exists damage_types text[],       -- loai hu hong
    add column if not exists station_id varchar,        -- tram nao ghi
    add column if not exists evidence_path text;        -- thu muc anh bang chung tren may tram

alter table public.sorting_logs
    add constraint sorting_logs_inspection_uuid_key unique (inspection_uuid);

alter table public.sorting_logs
    alter column created_at set default (now() at time zone 'utc'),
    alter column scan_attempts set default 1;

-- A.3 Index cho khoa ngoai.
create index if not exists ix_order_items_order_id  on public.order_items (order_id);
create index if not exists ix_orders_user_id        on public.orders (user_id);
create index if not exists ix_products_category_id  on public.products (category_id);
create index if not exists ix_sorting_logs_order_id on public.sorting_logs (order_id);
create index if not exists ix_sorting_logs_created  on public.sorting_logs (created_at desc);

-- A.4 Trigger: tram chi INSERT vao sorting_logs; database tu noi voi don hang.
create or replace function public.sorting_logs_apply()
returns trigger
language plpgsql
security definer
set search_path = ''
as $$
begin
    if new.order_id is null then
        select o.id into new.order_id
        from public.orders o
        where o.tracking_code = new.tracking_code;
    end if;

    -- Chi kien dat (PASS) moi chuyen don sang SORTED. Kien bi loai hoac can
    -- kiem tra lai giu nguyen trang thai don; ket qua xem o v_order_tracking.
    if new.order_id is not null and new.decision = 'PASS' then
        update public.orders o
        set status = 'SORTED',
            sorted_at = (now() at time zone 'utc')
        where o.id = new.order_id;
    end if;

    return new;
end;
$$;

revoke all on function public.sorting_logs_apply() from public, anon, authenticated;

drop trigger if exists trg_sorting_logs_apply on public.sorting_logs;
create trigger trg_sorting_logs_apply
before insert on public.sorting_logs
for each row execute function public.sorting_logs_apply();

-- A.5 View theo doi cho doi quan ly (khong chua ten/SDT/dia chi chi tiet).
create or replace view public.v_order_tracking
with (security_invoker = true) as
select o.id, o.tracking_code, o.order_code, o.recipient_province, o.routing_code,
       o.destination_chute_id as planned_chute_id, o.destination_hub, o.status,
       o.label_size, o.created_at, o.confirmed_at, o.hub_received_at, o.sorted_at,
       l.decision        as last_decision,
       l.reason          as last_reason,
       l.qr_ok           as last_qr_ok,
       l.logo_ok         as last_logo_ok,
       l.damage_detected as last_damage_detected,
       l.damage_types    as last_damage_types,
       l.chute_id        as actual_chute_id,
       l.station_id      as last_station_id,
       l.created_at      as last_scanned_at
from public.orders o
left join lateral (
    select s.* from public.sorting_logs s
    where s.tracking_code = o.tracking_code
    order by s.created_at desc, s.id desc
    limit 1
) l on true;

create or replace view public.v_chute_summary
with (security_invoker = true) as
select o.destination_chute_id as chute_id,
       max(o.destination_hub) as destination_hub,
       count(*)                                         as total_orders,
       count(*) filter (where o.sorted_at is not null)  as sorted_orders,
       count(*) filter (where o.sorted_at is null)      as waiting_orders
from public.orders o
group by o.destination_chute_id
order by o.destination_chute_id;

create or replace view public.v_sorting_exceptions
with (security_invoker = true) as
select s.id, s.created_at, s.tracking_code, s.order_id, s.decision, s.reason,
       s.qr_ok, s.logo_ok, s.damage_detected, s.damage_types,
       s.chute_id, s.station_id, s.evidence_path
from public.sorting_logs s
where s.decision in ('REJECT', 'MANUAL_REVIEW')
order by s.created_at desc;


-- =====================================================================
-- PHAN B: CHUA CHAY - BAT RLS
-- Hien tai ca 6 bang tat RLS va role anon co du quyen doc/sua/xoa, ke ca
-- users.password_hash va thong tin nguoi nhan.
--
-- Da kiem tra ngay 2026-11-10: moi truy van cua trang web deu di qua role
-- postgres (backend ket noi thang Postgres), khong co truy van nao qua anon
-- key. Vi vay backend KHONG bi anh huong boi phan nay.
--
-- Sau khi chay:
--   - Ai co anon key: chi DOC duoc categories va products.
--   - May tram webcam: phai dung secret/service_role key trong file .env.
--   - Doi quan ly: xem qua Supabase Dashboard nhu cu.
--
-- Quay lai neu can:
--   alter table public.<ten_bang> disable row level security;
--   grant all on public.<ten_bang> to anon, authenticated;
-- =====================================================================
begin;

alter table public.users        enable row level security;
alter table public.categories   enable row level security;
alter table public.products     enable row level security;
alter table public.orders       enable row level security;
alter table public.order_items  enable row level security;
alter table public.sorting_logs enable row level security;

create policy "catalog_read_categories" on public.categories
    for select to anon, authenticated using (true);
create policy "catalog_read_products" on public.products
    for select to anon, authenticated using (true);

revoke all on public.users, public.orders, public.order_items, public.sorting_logs
    from anon, authenticated;
revoke insert, update, delete, truncate, references, trigger
    on public.categories, public.products
    from anon, authenticated;
revoke all on public.v_order_tracking, public.v_chute_summary, public.v_sorting_exceptions
    from anon, authenticated;

commit;
