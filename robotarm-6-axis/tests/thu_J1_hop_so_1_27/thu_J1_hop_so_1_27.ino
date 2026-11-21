// =====================================================================================
// FIRMWARE THỬ RIÊNG J1 — động cơ 42STH38-1684A + hộp số hành tinh 1:27 (+ đai 20T→60T theo CAD)
// Chân, driver, UART giống robotarmver2.ino: STEP GPIO1, DIR GPIO2, EN GPIO14 (LOW = chạy),
// TMC2209 J1 trên UART1 (RX17/TX18, địa chỉ 0). Không dùng thư viện phát xung: timer 40 kHz + hình thang.
//
// Cách 1 — Serial Monitor 115200, kết thúc dòng "Newline", gõ lệnh:
//   g 30    : đi tới góc khớp +30° (tuyệt đối)       r -10 : quay thêm -10° (tương đối)
//   v 10    : tốc độ tối đa 10°/s                     a 20  : gia tốc 20°/s²
//   k 81    : tỉ số tổng (27 × đai 3 = 81; không có đai thì k 27) — đổi tỉ số sẽ đặt lại vị trí = 0
//   i 1100  : dòng RMS (mA, 200–1400)                 d     : đảo chiều quay
//   z       : lấy vị trí hiện tại làm 0°              s     : DỪNG NGAY
//   x / e   : tắt / bật driver (tắt = thả lỏng trục)  ?     : trạng thái (góc, bước, TMC2209)
// Cách 2 — GUI main.py / Station qua USB: nhận gói nhị phân 30 byte như firmware chính, chỉ dùng
//   góc J1 (slider Waist); telemetry trả góc J1, các khớp khác = 0.
// =====================================================================================
#include <Arduino.h>
#include <TMCStepper.h>
#include "soc/gpio_reg.h"

#define STEP_PIN 1
#define DIR_PIN 2
#define ENABLE_PIN 14
#define RX1_PIN 17
#define TX1_PIN 18
#define STEP_TICK_HZ 40000

// Gói nhị phân 30 byte của GUI (khai báo sớm: Arduino tự sinh prototype hàm ở đầu file)
struct __attribute__((packed)) Packet { uint16_t header; uint8_t cmd, tool; float ang[6]; uint16_t crc; };

// ---- thông số mặc định (sửa bằng lệnh khi chạy) ----
const float STEPS_PER_REV = 200.0f;
const uint16_t MICROSTEPS = 8;      // hộp số 1:27 đã chia mịn → 8 vi bước đủ (≈0.0028°/vi bước ở tỉ số 81)
float gear_ratio = 81.0f;           // 27 (hộp số) × 3 (đai 20T→60T, ước lượng từ CAD)
float dir_sign = 1.0f;
float max_speed_deg = 12.0f;        // °/s khớp (= 4320 bước/s = 2.7 vòng/s trục động cơ)
float accel_deg = 20.0f;            // °/s²
uint16_t current_ma = 1100;         // 1.68 A định mức → 1100 mA RMS (đỉnh 1.56 A)

TMC2209Stepper driver(&Serial1, 0.11f, 0);

// ---- bộ phát xung (như robotarmver2) ----
volatile int32_t pos = 0, target = 0;
volatile uint32_t rate_q16 = 0, phase = 0;
volatile int8_t dir = 1;
volatile bool step_high = false;
volatile float v = 0;  // bước/s có dấu
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t *timer = nullptr;
bool driver_enabled = true;

float steps_per_deg() { return STEPS_PER_REV * MICROSTEPS * gear_ratio * dir_sign / 360.0f; }

void IRAM_ATTR onTimer() {
  if (step_high) {
    step_high = false;
    REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << STEP_PIN);
    return;
  }
  uint32_t r = rate_q16;
  if (!r) return;
  uint32_t ph = phase + r;
  if (ph >= 65536UL) {
    ph -= 65536UL;
    pos += dir;
    step_high = true;
    REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << STEP_PIN);
  }
  phase = ph;
}

// Hình thang vận tốc 1 kHz: v_des = sign(d)·min(vmax, √(2·a·|d|)); đổi DIR chỉ khi đã dừng phát ≥ 1 ms
void profileTask(void *) {
  TickType_t last = xTaskGetTickCount();
  while (true) {
    float spd = fabsf(steps_per_deg());
    float a = accel_deg * spd, vmax = fminf(max_speed_deg * spd, 0.45f * STEP_TICK_HZ), dv = a * 0.001f;
    portENTER_CRITICAL(&mux);
    int32_t d = target - pos;
    float vv = v, vdes = 0;
    if (d) vdes = (d > 0 ? 1.0f : -1.0f) * fminf(vmax, sqrtf(2.0f * a * fabsf((float)d)));
    if (vdes > vv + dv) vv += dv; else if (vdes < vv - dv) vv -= dv; else vv = vdes;
    if (d == 0 && fabsf(vv) <= 2 * dv) vv = 0;
    int8_t want = vv > 0 ? 1 : (vv < 0 ? -1 : dir);
    if (want != dir) {
      if (rate_q16 == 0) { dir = want; gpio_set_level((gpio_num_t)DIR_PIN, want > 0); }
      vv = 0;
      rate_q16 = 0;
    } else {
      rate_q16 = (uint32_t)(fabsf(vv) * (65536.0f / STEP_TICK_HZ));
    }
    v = vv;
    portEXIT_CRITICAL(&mux);
    vTaskDelayUntil(&last, 1);
  }
}

void forceStop(bool zero) {
  portENTER_CRITICAL(&mux); rate_q16 = 0; v = 0; target = pos; portEXIT_CRITICAL(&mux);
  delayMicroseconds(80);
  portENTER_CRITICAL(&mux);
  if (zero) pos = 0;
  target = pos; phase = 0; v = 0; rate_q16 = 0;
  portEXIT_CRITICAL(&mux);
}

float angleNow() { return (float)pos / steps_per_deg(); }
void goTo(float deg) { target = (int32_t)lroundf(deg * steps_per_deg()); }

void configureDriver() {
  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(current_ma);
  driver.microsteps(MICROSTEPS);
  driver.mstep_reg_select(true);
  driver.internal_Rsense(false);
  driver.pdn_disable(true);
  driver.en_spreadCycle(true);
  driver.TPWMTHRS(0);
  driver.GSTAT(0b111);
}

void report() {
  uint8_t c = driver.test_connection();
  uint32_t ds = driver.DRV_STATUS();
  Serial.printf("[J1] goc=%.2f do  dich=%.2f do  buoc=%ld  dang_chay=%s  ti_so=%.2f  chieu=%+.0f  v=%.1f do/s  a=%.1f do/s2\n",
                angleNow(), (float)target / steps_per_deg(), (long)pos, (v != 0 || pos != target) ? "co" : "khong",
                gear_ratio, dir_sign, max_speed_deg, accel_deg);
  Serial.printf("[J1] TMC2209: %s, vi buoc=%u, dong=%umA, DRV_STATUS=0x%08lX%s%s%s%s%s  EN=%s\n",
                c == 0 ? "OK" : "KHONG TRA LOI (bat nguon dong luc?)", driver.microsteps(), driver.rms_current(),
                (unsigned long)ds, (ds & 0x40) ? " HO_CUON_A" : "", (ds & 0x80) ? " HO_CUON_B" : "",
                (ds & 0x3C) ? " CHAM_MASS/NGUON" : "", (ds & 0x02) ? " QUA_NHIET" : "", (ds & 0x01) ? " CANH_BAO_NHIET" : "",
                driver_enabled ? "bat" : "TAT");
}

void handleText(char *line) {
  char cmd = line[0];
  float val = atof(line + 1);
  switch (cmd) {
    case 'g': goTo(val); Serial.printf("[J1] di toi %.2f do\n", val); break;
    case 'r': goTo((float)target / steps_per_deg() + val); Serial.printf("[J1] quay them %.2f do\n", val); break;
    case 'v': if (val > 0) max_speed_deg = val; Serial.printf("[J1] v = %.1f do/s\n", max_speed_deg); break;
    case 'a': if (val > 0) accel_deg = val; Serial.printf("[J1] a = %.1f do/s2\n", accel_deg); break;
    case 'k': if (val >= 1) { forceStop(true); gear_ratio = val; } Serial.printf("[J1] ti so = %.3f (vi tri dat lai = 0)\n", gear_ratio); break;
    case 'i': if (val >= 200 && val <= 1400) { current_ma = (uint16_t)val; driver.rms_current(current_ma); }
              Serial.printf("[J1] dong = %u mA\n", current_ma); break;
    case 'd': forceStop(true); dir_sign = -dir_sign; Serial.printf("[J1] dao chieu: %+.0f (vi tri dat lai = 0)\n", dir_sign); break;
    case 'z': forceStop(true); Serial.println("[J1] vi tri hien tai = 0 do"); break;
    case 's': forceStop(false); Serial.printf("[J1] DUNG tai %.2f do\n", angleNow()); break;
    case 'x': forceStop(false); digitalWrite(ENABLE_PIN, HIGH); driver_enabled = false; Serial.println("[J1] driver TAT"); break;
    case 'e': digitalWrite(ENABLE_PIN, LOW); driver_enabled = true; Serial.println("[J1] driver BAT"); break;
    case '?': report(); break;
    default: if (cmd) Serial.println("[J1] lenh: g r v a k i d z s x e ?");
  }
}

// ---- giao thức nhị phân của GUI (30 byte, CRC16 Modbus) ----
uint16_t crc16(const uint8_t *d, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) { c ^= d[i]; for (int j = 0; j < 8; j++) c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1; }
  return c;
}
uint8_t bin[sizeof(Packet)];
size_t bin_idx = 0;
char text[48];
size_t text_idx = 0;
bool gui_mode = false;
uint32_t last_tx = 0, last_byte = 0;

void handlePacket(const Packet &p) {
  gui_mode = true;
  if (p.cmd == 0) forceStop(false);            // STOP
  else if (p.cmd == 1) goTo(p.ang[0]);         // MOVE_PC: chỉ dùng góc J1
  else if (p.cmd == 5) forceStop(true);        // SET_ZERO
}

void setup() {
  Serial.begin(115200);
  pinMode(STEP_PIN, OUTPUT); digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT); digitalWrite(DIR_PIN, HIGH);
  gpio_set_drive_capability((gpio_num_t)STEP_PIN, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)DIR_PIN, GPIO_DRIVE_CAP_3);
  pinMode(ENABLE_PIN, OUTPUT); digitalWrite(ENABLE_PIN, LOW);
  Serial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);
  gpio_set_drive_capability((gpio_num_t)TX1_PIN, GPIO_DRIVE_CAP_0);  // bus UART1 cần TX yếu (chẩn đoán 10/2026)
  delay(200);
  configureDriver();
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1000000 / STEP_TICK_HZ, true, 0);
  xTaskCreatePinnedToCore(profileTask, "profile", 4096, NULL, 5, NULL, 0);
  Serial.println("\n[J1] FIRMWARE THU RIENG J1 (hop so 1:27) san sang. Go ? de xem trang thai, g 30 de quay toi 30 do.");
  report();
}

void loop() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    uint32_t now = millis();
    if (bin_idx && now - last_byte > 100) bin_idx = 0;
    last_byte = now;
    if (bin_idx == 0 && b == 0x55) { bin[bin_idx++] = b; continue; }
    if (bin_idx == 1) { if (b == 0xAA) { bin[bin_idx++] = b; } else { bin_idx = 0; } continue; }
    if (bin_idx >= 2) {
      bin[bin_idx++] = b;
      if (bin_idx == sizeof(Packet)) {
        Packet *p = (Packet *)bin;
        if (p->crc == crc16(bin, sizeof(Packet) - 2)) handlePacket(*p);
        bin_idx = 0;
      }
      continue;
    }
    if (b == '\n' || b == '\r') {
      if (text_idx) { text[text_idx] = 0; handleText(text); text_idx = 0; }
    } else if (text_idx < sizeof(text) - 1 && b >= 32 && b < 127) {
      text[text_idx++] = (char)b;
    }
  }
  // telemetry 30 Hz chỉ khi đang nói chuyện với GUI (tránh rác nhị phân trên Serial Monitor)
  if (gui_mode && millis() - last_tx > 33) {
    last_tx = millis();
    Packet t = {0x55AA, 0, 0, {angleNow(), 0, 0, 0, 0, 0}, 0};
    t.crc = crc16((uint8_t *)&t, sizeof(Packet) - 2);
    Serial.write((uint8_t *)&t, sizeof(Packet));
  }
  delay(1);
}
