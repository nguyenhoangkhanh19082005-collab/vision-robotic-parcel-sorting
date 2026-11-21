// =====================================================================================
// FIRMWARE THỬ RIÊNG J5 và J6 — động cơ CŨ (J5: Nema17 + đai ×2.5, J6: 17HS3401-S truyền thẳng)
// Chân giống robotarmver2.ino: J5 STEP 40 / DIR 41, J6 STEP 38 / DIR 39, EN GPIO14 (LOW = chạy),
// TMC2209 trên UART2 (RX8/TX9): J5 địa chỉ 1, J6 địa chỉ 2 (J4 địa chỉ 0 nếu có cắm cũng được đọc ở lệnh ?).
// Phát xung: timer 40 kHz + hình thang vận tốc (giống firmware chính).
//
// Mở Serial Monitor 115200, kết thúc dòng "Newline". Mỗi lệnh bắt đầu bằng số khớp 5 hoặc 6:
//   5g 30   : J5 đi tới +30° (tuyệt đối)      6r -45 : J6 quay thêm -45° (tương đối)
//   5v 20   : tốc độ tối đa 20°/s             5a 40  : gia tốc 40°/s²
//   5k 2.5  : tỉ số truyền (đặt lại vị trí=0) 5i 800 : dòng RMS mA (200–1400)
//   5d      : đảo chiều                       5z     : lấy vị trí hiện tại làm 0°
//   5s      : dừng khớp 5                     5m 16  : vi bước (1,2,4,8,16,32...; đặt lại vị trí=0)
//   5t      : thử xung "cua": 1 vòng trục động cơ chạy chậm (kiểm tra động cơ quay đủ 360° động cơ)
//   s       : DỪNG CẢ HAI                     x / e  : tắt / bật driver (EN)
//   ?       : trạng thái J4, J5, J6 (TMC2209: kết nối, cuộn dây hở, quá nhiệt...)
// Mẹo kiểm tra J6 quay vô hạn: gõ "?" khi KHÔNG ra lệnh gì. Nếu J6 vẫn quay thì xung STEP (GPIO38)
// bị nhiễu/hở dây — gõ "x" (tắt driver): hết quay nghĩa là có xung rác; vẫn quay là lỗi khác.
// =====================================================================================
#include <Arduino.h>
#include <TMCStepper.h>
#include "soc/gpio_reg.h"

#define ENABLE_PIN 14
#define RX2_PIN 8
#define TX2_PIN 9
#define STEP_TICK_HZ 40000
#define NJ 2  // chỉ số 0 = J5, 1 = J6

const uint8_t STEP_PIN[NJ] = {40, 38};
const uint8_t DIR_PIN[NJ] = {41, 39};
const uint8_t TMC_ADDR[NJ] = {1, 2};
const char *NAME[NJ] = {"J5", "J6"};

// ---- thông số mặc định (giống robotarmver2.ino, sửa bằng lệnh khi chạy) ----
const float STEPS_PER_REV = 200.0f;
uint16_t microsteps[NJ] = {16, 16};
float gear_ratio[NJ] = {2.5f, 1.0f};
float dir_sign[NJ] = {1.0f, 1.0f};
float max_speed_deg[NJ] = {20.0f, 30.0f};   // °/s khớp (thấp để dễ quan sát)
float accel_deg[NJ] = {40.0f, 60.0f};       // °/s²
uint16_t current_ma[NJ] = {800, 900};

TMC2209Stepper drv5(&Serial2, 0.11f, 1);
TMC2209Stepper drv6(&Serial2, 0.11f, 2);
TMC2209Stepper drv4(&Serial2, 0.11f, 0);  // chỉ để đọc trạng thái J4 (không phát xung)
TMC2209Stepper *drv[NJ] = {&drv5, &drv6};

volatile int32_t pos[NJ] = {0, 0}, target[NJ] = {0, 0};
volatile uint32_t rate_q16[NJ] = {0, 0}, phase[NJ] = {0, 0};
volatile int8_t dir[NJ] = {1, 1};
volatile bool step_high[NJ] = {false, false};
volatile float vel[NJ] = {0, 0};  // bước/s có dấu
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t *timer = nullptr;
bool driver_enabled = true;

float steps_per_deg(int j) { return STEPS_PER_REV * microsteps[j] * gear_ratio[j] * dir_sign[j] / 360.0f; }

void IRAM_ATTR onTimer() {
  for (int j = 0; j < NJ; j++) {
    if (step_high[j]) {
      step_high[j] = false;
      REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << STEP_PIN[j]);
      continue;
    }
    uint32_t r = rate_q16[j];
    if (!r) continue;
    uint32_t ph = phase[j] + r;
    if (ph >= 65536UL) {
      ph -= 65536UL;
      pos[j] += dir[j];
      step_high[j] = true;
      REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << STEP_PIN[j]);
    }
    phase[j] = ph;
  }
}

// Hình thang vận tốc 1 kHz cho từng khớp; đổi DIR chỉ khi đã ngừng phát xung
void profileTask(void *) {
  TickType_t last = xTaskGetTickCount();
  while (true) {
    for (int j = 0; j < NJ; j++) {
      float spd = fabsf(steps_per_deg(j));
      float a = accel_deg[j] * spd, vmax = fminf(max_speed_deg[j] * spd, 0.45f * STEP_TICK_HZ), dv = a * 0.001f;
      portENTER_CRITICAL(&mux);
      int32_t d = target[j] - pos[j];
      float vv = vel[j], vdes = 0;
      if (d) vdes = (d > 0 ? 1.0f : -1.0f) * fminf(vmax, sqrtf(2.0f * a * fabsf((float)d)));
      if (vdes > vv + dv) vv += dv; else if (vdes < vv - dv) vv -= dv; else vv = vdes;
      if (d == 0 && fabsf(vv) <= 2 * dv) vv = 0;
      int8_t want = vv > 0 ? 1 : (vv < 0 ? -1 : dir[j]);
      if (want != dir[j]) {
        if (rate_q16[j] == 0) { dir[j] = want; gpio_set_level((gpio_num_t)DIR_PIN[j], want > 0); }
        vv = 0;
        rate_q16[j] = 0;
      } else {
        rate_q16[j] = (uint32_t)(fabsf(vv) * (65536.0f / STEP_TICK_HZ));
      }
      vel[j] = vv;
      portEXIT_CRITICAL(&mux);
    }
    vTaskDelayUntil(&last, 1);
  }
}

void forceStop(int j, bool zero) {
  portENTER_CRITICAL(&mux); rate_q16[j] = 0; vel[j] = 0; target[j] = pos[j]; portEXIT_CRITICAL(&mux);
  delayMicroseconds(80);
  portENTER_CRITICAL(&mux);
  if (zero) pos[j] = 0;
  target[j] = pos[j]; phase[j] = 0; vel[j] = 0; rate_q16[j] = 0;
  portEXIT_CRITICAL(&mux);
}

float angleNow(int j) { return (float)pos[j] / steps_per_deg(j); }
void goTo(int j, float deg) { target[j] = (int32_t)lroundf(deg * steps_per_deg(j)); }

void configureDriver(int j) {
  drv[j]->begin();
  drv[j]->toff(4);
  drv[j]->blank_time(24);
  drv[j]->rms_current(current_ma[j]);
  drv[j]->microsteps(microsteps[j]);
  drv[j]->mstep_reg_select(true);
  drv[j]->internal_Rsense(false);
  drv[j]->pdn_disable(true);
  drv[j]->en_spreadCycle(true);
  drv[j]->TPWMTHRS(0);
  drv[j]->GSTAT(0b111);
}

void printDriver(const char *name, TMC2209Stepper &d) {
  uint8_t c = d.test_connection();
  uint32_t ds = d.DRV_STATUS();
  Serial.printf("  %s TMC2209: %s  DRV_STATUS=0x%08lX%s%s%s%s%s\n", name,
                c == 0 ? "OK" : "KHONG TRA LOI (nguon dong luc/day UART?)", (unsigned long)ds,
                (ds & 0x40) ? " HO_CUON_A" : "", (ds & 0x80) ? " HO_CUON_B" : "",
                (ds & 0x3C) ? " CHAM_MASS/NGUON" : "", (ds & 0x02) ? " QUA_NHIET" : "", (ds & 0x01) ? " CANH_BAO_NHIET" : "");
}

void report() {
  for (int j = 0; j < NJ; j++)
    Serial.printf("[%s] goc=%.2f do dich=%.2f do buoc=%ld chay=%s ti_so=%.3f chieu=%+.0f vi_buoc=%u dong=%umA v=%.1f do/s a=%.1f do/s2\n",
                  NAME[j], angleNow(j), (float)target[j] / steps_per_deg(j), (long)pos[j],
                  (vel[j] != 0 || pos[j] != target[j]) ? "co" : "khong", gear_ratio[j], dir_sign[j], microsteps[j],
                  current_ma[j], max_speed_deg[j], accel_deg[j]);
  printDriver("J4", drv4);
  printDriver("J5", drv5);
  printDriver("J6", drv6);
  Serial.printf("  EN=%s\n", driver_enabled ? "bat" : "TAT");
}

void handleText(char *line) {
  char c0 = line[0];
  if (c0 == '?') { report(); return; }
  if (c0 == 's') { for (int j = 0; j < NJ; j++) forceStop(j, false); Serial.println("DUNG CA HAI"); return; }
  if (c0 == 'x') { for (int j = 0; j < NJ; j++) forceStop(j, false); digitalWrite(ENABLE_PIN, HIGH); driver_enabled = false; Serial.println("driver TAT"); return; }
  if (c0 == 'e') { digitalWrite(ENABLE_PIN, LOW); driver_enabled = true; Serial.println("driver BAT"); return; }
  if (c0 != '5' && c0 != '6') { Serial.println("lenh: <5|6><g r v a k i d z s m t> [so]   hoac  s x e ?"); return; }
  int j = c0 - '5';
  char cmd = line[1];
  float val = atof(line + 2);
  const char *n = NAME[j];
  switch (cmd) {
    case 'g': goTo(j, val); Serial.printf("[%s] di toi %.2f do\n", n, val); break;
    case 'r': goTo(j, (float)target[j] / steps_per_deg(j) + val); Serial.printf("[%s] quay them %.2f do\n", n, val); break;
    case 'v': if (val > 0) max_speed_deg[j] = val; Serial.printf("[%s] v = %.1f do/s\n", n, max_speed_deg[j]); break;
    case 'a': if (val > 0) accel_deg[j] = val; Serial.printf("[%s] a = %.1f do/s2\n", n, accel_deg[j]); break;
    case 'k': if (val >= 0.1f) { forceStop(j, true); gear_ratio[j] = val; } Serial.printf("[%s] ti so = %.3f (vi tri = 0)\n", n, gear_ratio[j]); break;
    case 'i': if (val >= 200 && val <= 1400) { current_ma[j] = (uint16_t)val; drv[j]->rms_current(current_ma[j]); }
              Serial.printf("[%s] dong = %u mA\n", n, current_ma[j]); break;
    case 'm': if (val >= 1 && val <= 256) { forceStop(j, true); microsteps[j] = (uint16_t)val; drv[j]->microsteps(microsteps[j]); }
              Serial.printf("[%s] vi buoc = %u (vi tri = 0)\n", n, microsteps[j]); break;
    case 'd': forceStop(j, true); dir_sign[j] = -dir_sign[j]; Serial.printf("[%s] dao chieu: %+.0f (vi tri = 0)\n", n, dir_sign[j]); break;
    case 'z': forceStop(j, true); Serial.printf("[%s] vi tri hien tai = 0 do\n", n); break;
    case 's': forceStop(j, false); Serial.printf("[%s] DUNG tai %.2f do\n", n, angleNow(j)); break;
    case 't': {  // đúng 1 vòng trục động cơ (200 × vi bước), chạy chậm: quan sát xem trục có quay đủ 1 vòng không
      forceStop(j, true);
      target[j] = (int32_t)(STEPS_PER_REV * microsteps[j] * dir_sign[j]);  // dùng tốc độ v hiện tại (đặt bằng lệnh v)
      Serial.printf("[%s] thu 1 vong truc dong co (%ld xung). Xong go ? de doc so buoc.\n", n, (long)target[j]);
      break;
    }
    default: Serial.println("lenh: g r v a k i m d z s t");
  }
}

void setup() {
  Serial.begin(115200);
  for (int j = 0; j < NJ; j++) {
    pinMode(STEP_PIN[j], OUTPUT); digitalWrite(STEP_PIN[j], LOW);
    pinMode(DIR_PIN[j], OUTPUT); digitalWrite(DIR_PIN[j], HIGH);
    gpio_set_drive_capability((gpio_num_t)STEP_PIN[j], GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)DIR_PIN[j], GPIO_DRIVE_CAP_3);
  }
  pinMode(ENABLE_PIN, OUTPUT); digitalWrite(ENABLE_PIN, LOW);
  Serial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);
  gpio_set_drive_capability((gpio_num_t)TX2_PIN, GPIO_DRIVE_CAP_0);
  delay(200);
  for (int j = 0; j < NJ; j++) configureDriver(j);
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1000000 / STEP_TICK_HZ, true, 0);
  xTaskCreatePinnedToCore(profileTask, "profile", 4096, NULL, 5, NULL, 0);
  Serial.println("\nFIRMWARE THU RIENG J5 + J6 (dong co cu) san sang. Vi du: 5g 20  6g 90  s  ?");
  report();
}

char text[48];
size_t text_idx = 0;

void loop() {
  while (Serial.available()) {
    char b = Serial.read();
    if (b == '\n' || b == '\r') {
      if (text_idx) { text[text_idx] = 0; handleText(text); text_idx = 0; }
    } else if (text_idx < sizeof(text) - 1 && b >= 32 && b < 127) {
      text[text_idx++] = b;
    }
  }
  delay(1);
}
