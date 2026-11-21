// =====================================================================================
// FIRMWARE THỬ 1 ĐỘNG CƠ — chỉ dùng ổ cắm J1 (STEP GPIO1, DIR GPIO2, EN GPIO14, UART1 RX17/TX18).
// Cắm động cơ cần thử (J1 hoặc động cơ cũ khác) vào ổ J1, rút ra cắm động cơ khác khi cần (TẮT NGUỒN trước khi rút/cắm!).
// Tự dò địa chỉ TMC2209 trên UART1 (0..3), không cần biết ổ J1 đặt địa chỉ nào.
// Góc tính theo TRỤC ĐỘNG CƠ (tỉ số mặc định 1). Có đai/hộp số thì đặt "k <tỉ số>".
//
// Serial Monitor 115200, kết thúc dòng "Newline". Lệnh:
//   g 90    : đi tới góc +90° (tuyệt đối)        r -45 : quay thêm -45°
//   t       : thử đúng 1 vòng trục động cơ        p 200 : phát thẳng 200 xung STEP (không hình thang, kiểm tra cứng)
//   v 90    : tốc độ °/s                          a 180 : gia tốc °/s²
//   i 800   : dòng RMS mA (200–1400)              m 16  : vi bước
//   k 2.5   : tỉ số truyền                        d     : đảo chiều
//   z       : lấy vị trí hiện tại làm 0           s     : DỪNG
//   x / e   : tắt / bật driver (EN)               ?     : trạng thái + chẩn đoán
//   u       : dò lại địa chỉ driver (sau khi đổi động cơ/driver, bật lại nguồn)
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

float steps_rev = 200.0f, gear_ratio = 1.0f, dir_sign = 1.0f;
uint16_t microsteps = 16, current_ma = 800;
float max_speed_deg = 90.0f, accel_deg = 180.0f;

TMC2209Stepper *drv = nullptr;
int8_t drv_addr = -1;  // -1 = chưa thấy driver

volatile int32_t pos = 0, target = 0;
volatile uint32_t rate_q16 = 0, phase = 0;
volatile int8_t dir = 1;
volatile bool step_high = false;
volatile float vel = 0;
volatile uint32_t pulse_left = 0;  // lệnh p: số xung thô còn lại
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t *timer = nullptr;
bool enabled = true;

float spd() { return steps_rev * microsteps * gear_ratio * dir_sign / 360.0f; }

void IRAM_ATTR onTimer() {
  if (step_high) { step_high = false; REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << STEP_PIN); return; }
  uint32_t r = rate_q16;
  if (!r) return;
  uint32_t ph = phase + r;
  if (ph >= 65536UL) { ph -= 65536UL; pos += dir; step_high = true; REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << STEP_PIN); }
  phase = ph;
}

void profileTask(void *) {
  TickType_t last = xTaskGetTickCount();
  while (true) {
    float s = fabsf(spd()), a = accel_deg * s, vmax = fminf(max_speed_deg * s, 0.45f * STEP_TICK_HZ), dv = a * 0.001f;
    portENTER_CRITICAL(&mux);
    int32_t d = target - pos;
    float vv = vel, vdes = 0;
    if (d) vdes = (d > 0 ? 1.0f : -1.0f) * fminf(vmax, sqrtf(2.0f * a * fabsf((float)d)));
    if (vdes > vv + dv) vv += dv; else if (vdes < vv - dv) vv -= dv; else vv = vdes;
    if (d == 0 && fabsf(vv) <= 2 * dv) vv = 0;
    int8_t want = vv > 0 ? 1 : (vv < 0 ? -1 : dir);
    if (want != dir) {
      if (rate_q16 == 0) { dir = want; gpio_set_level((gpio_num_t)DIR_PIN, want > 0); }
      vv = 0; rate_q16 = 0;
    } else rate_q16 = (uint32_t)(fabsf(vv) * (65536.0f / STEP_TICK_HZ));
    vel = vv;
    portEXIT_CRITICAL(&mux);
    vTaskDelayUntil(&last, 1);
  }
}

void forceStop(bool zero) {
  portENTER_CRITICAL(&mux); rate_q16 = 0; vel = 0; target = pos; portEXIT_CRITICAL(&mux);
  delayMicroseconds(80);
  portENTER_CRITICAL(&mux);
  if (zero) pos = 0;
  target = pos; phase = 0; vel = 0; rate_q16 = 0;
  portEXIT_CRITICAL(&mux);
}

float angleNow() { return (float)pos / spd(); }
void goTo(float deg) { target = (int32_t)lroundf(deg * spd()); }

// Dò địa chỉ 0..3: driver nào trả lời thì dùng
bool findDriver() {
  delete drv; drv = nullptr; drv_addr = -1;
  for (uint8_t a = 0; a < 4; a++) {
    TMC2209Stepper probe(&Serial1, 0.11f, a);
    probe.begin();
    if (probe.test_connection() == 0) { drv_addr = a; break; }
  }
  if (drv_addr < 0) return false;
  drv = new TMC2209Stepper(&Serial1, 0.11f, (uint8_t)drv_addr);
  drv->begin();
  drv->toff(4); drv->blank_time(24);
  drv->rms_current(current_ma); drv->microsteps(microsteps);
  drv->mstep_reg_select(true); drv->internal_Rsense(false);
  drv->pdn_disable(true); drv->en_spreadCycle(true); drv->TPWMTHRS(0);
  drv->GSTAT(0b111);
  return true;
}

void report() {
  Serial.printf("goc=%.2f do  buoc=%ld  dich=%.2f  v=%.0f do/s a=%.0f  ti_so=%.2f chieu=%+.0f vi_buoc=%u dong=%umA EN=%s\n",
                angleNow(), (long)pos, (float)target / spd(), max_speed_deg, accel_deg, gear_ratio, dir_sign, microsteps,
                current_ma, enabled ? "bat" : "TAT");
  if (!drv) {
    Serial.println("!! KHONG tim thay TMC2209 nao tren UART1 (dia chi 0..3). Kiem tra: (1) da BAT NGUON dong luc 12V?"
                   " (2) dung cong COM CH343, (3) day UART RX17/TX18 toi driver, (4) driver co bi chay nong/hong. Go u de do lai.");
    return;
  }
  uint8_t c = drv->test_connection();
  uint32_t ds = drv->DRV_STATUS();
  Serial.printf("TMC2209 dia chi %d: %s  IFCNT=%u  DRV_STATUS=0x%08lX%s%s%s%s%s\n", drv_addr, c == 0 ? "OK" : "MAT KET NOI",
                drv->IFCNT(), (unsigned long)ds, (ds & 0x40) ? " HO_CUON_A" : "", (ds & 0x80) ? " HO_CUON_B" : "",
                (ds & 0x3C) ? " CHAM_MASS" : "", (ds & 0x02) ? " QUA_NHIET" : "", (ds & 0x01) ? " CANH_BAO_NHIET" : "");
  if (ds & 0xC0) Serial.println("!! Cuon day ho: kiem tra day dong co (dung cap A/B), dau noi, day dut.");
}

void handleText(char *line) {
  char cmd = line[0];
  float val = atof(line + 1);
  switch (cmd) {
    case 'g': goTo(val); Serial.printf("di toi %.2f do (%ld buoc)\n", val, (long)target); break;
    case 'r': goTo((float)target / spd() + val); Serial.printf("quay them %.2f do\n", val); break;
    case 't': forceStop(true); target = (int32_t)(steps_rev * microsteps * dir_sign);
              Serial.printf("thu 1 vong truc dong co = %ld xung\n", (long)target); break;
    case 'p': forceStop(false); { int32_t n = (int32_t)val; if (n < 1) n = 200; target = pos + (int32_t)(n * (dir_sign > 0 ? 1 : -1)); }
              Serial.printf("phat %ld xung\n", (long)(target - pos)); break;
    case 'v': if (val > 0) max_speed_deg = val; Serial.printf("v = %.0f do/s\n", max_speed_deg); break;
    case 'a': if (val > 0) accel_deg = val; Serial.printf("a = %.0f do/s2\n", accel_deg); break;
    case 'k': if (val >= 0.1f) { forceStop(true); gear_ratio = val; } Serial.printf("ti so = %.3f (vi tri=0)\n", gear_ratio); break;
    case 'i': if (val >= 200 && val <= 1400) { current_ma = (uint16_t)val; if (drv) drv->rms_current(current_ma); }
              Serial.printf("dong = %u mA\n", current_ma); break;
    case 'm': if (val >= 1 && val <= 256) { forceStop(true); microsteps = (uint16_t)val; if (drv) drv->microsteps(microsteps); }
              Serial.printf("vi buoc = %u (vi tri=0)\n", microsteps); break;
    case 'd': forceStop(true); dir_sign = -dir_sign; Serial.printf("chieu %+.0f (vi tri=0)\n", dir_sign); break;
    case 'z': forceStop(true); Serial.println("vi tri = 0"); break;
    case 's': forceStop(false); Serial.printf("DUNG tai %.2f do\n", angleNow()); break;
    case 'x': forceStop(false); digitalWrite(ENABLE_PIN, HIGH); enabled = false; Serial.println("driver TAT"); break;
    case 'e': digitalWrite(ENABLE_PIN, LOW); enabled = true; Serial.println("driver BAT"); break;
    case 'u': forceStop(false); Serial.println(findDriver() ? "da do lai driver" : "van khong thay driver"); report(); break;
    case '?': report(); break;
    default: Serial.println("lenh: g r t p v a i m k d z s x e u ?");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(STEP_PIN, OUTPUT); digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT); digitalWrite(DIR_PIN, HIGH);
  gpio_set_drive_capability((gpio_num_t)STEP_PIN, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)DIR_PIN, GPIO_DRIVE_CAP_3);
  pinMode(ENABLE_PIN, OUTPUT); digitalWrite(ENABLE_PIN, LOW);
  Serial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);
  gpio_set_drive_capability((gpio_num_t)TX1_PIN, GPIO_DRIVE_CAP_0);
  delay(300);
  findDriver();
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1000000 / STEP_TICK_HZ, true, 0);
  xTaskCreatePinnedToCore(profileTask, "profile", 4096, NULL, 5, NULL, 0);
  Serial.println("\n=== THU 1 DONG CO O O J1 === go ? de xem trang thai, r 90 de quay 90 do, s de dung");
  report();
}

char text[48];
size_t text_idx = 0;

void loop() {
  while (Serial.available()) {
    char b = Serial.read();
    if (b == '\n' || b == '\r') {
      if (text_idx) { text[text_idx] = 0; handleText(text); text_idx = 0; }
    } else if (text_idx < sizeof(text) - 1 && b >= 32 && b < 127) text[text_idx++] = b;
  }
  delay(1);
}
