// =====================================================================================
// FIRMWARE THỬ CHÂN STEP/DIR — KHÔNG dùng FastAccelStepper (phát xung bằng digitalWrite)
// Mục đích: tách lỗi phần cứng (dây, driver, động cơ) khỏi lỗi thư viện phát xung.
// Chân, driver TMC2209 và cấu hình UART giống hệt robotarmver2.ino.
//
// Lệnh qua Serial 115200 (gõ rồi Enter):
//   1..6 : quay TRỤC ĐỘNG CƠ của khớp đó 2 vòng thuận rồi 2 vòng nghịch, chậm (~3 s mỗi chiều)
//   m J N H : tùy chỉnh — khớp J, N vi bước mỗi chiều, nửa chu kỳ xung H µs (vd: m 1 3200 400)
//   s    : báo trạng thái chân, không chuyển động
// Ngoài lúc thử, MỌI chân STEP giữ LOW → nếu động cơ nào vẫn tự quay thì xung không đến từ ESP32.
// =====================================================================================
#include <Arduino.h>
#include <TMCStepper.h>

#define ENABLE_PIN 14
#define RX1_PIN 17
#define TX1_PIN 18
#define RX2_PIN 8
#define TX2_PIN 9

const uint8_t STEP_PINS[6] = {1, 4, 6, 15, 40, 38};
const uint8_t DIR_PINS[6]  = {2, 5, 7, 16, 41, 39};
const uint16_t MOTOR_CURRENTS[6] = {1000, 850, 700, 500, 800, 800};
const uint32_t TEST_STEPS = 6400;         // 2 vòng trục động cơ ở 16 vi bước (3200 vi bước/vòng)
const uint16_t HALF_PERIOD_US = 250;      // 2 kHz ≈ 0.6 vòng/s: chậm, đủ lâu để phân biệt quay thật với rung

TMC2209Stepper driver0(&Serial1, 0.11f, 0);
TMC2209Stepper driver1(&Serial1, 0.11f, 1);
TMC2209Stepper driver2(&Serial1, 0.11f, 2);
TMC2209Stepper driver3(&Serial2, 0.11f, 0);
TMC2209Stepper driver4(&Serial2, 0.11f, 1);
TMC2209Stepper driver5(&Serial2, 0.11f, 2);
TMC2209Stepper* drivers[6] = {&driver0, &driver1, &driver2, &driver3, &driver4, &driver5};

void pulse(uint8_t joint, uint32_t steps, bool forward, uint16_t half_us) {
  digitalWrite(DIR_PINS[joint], forward ? HIGH : LOW);
  delayMicroseconds(100);  // thời gian chốt chiều cho TMC2209
  for (uint32_t k = 0; k < steps; k++) {
    digitalWrite(STEP_PINS[joint], HIGH);
    delayMicroseconds(half_us);
    digitalWrite(STEP_PINS[joint], LOW);
    delayMicroseconds(half_us);
    if ((k & 1023) == 0) yield();  // tránh watchdog khi chạy lâu
  }
}

void runTest(uint8_t j, uint32_t steps, uint16_t half_us) {
  Serial.printf("[TEST] J%d: THUAN %lu vi buoc, nua chu ky %u us (GPIO%d)...\n", j + 1,
                (unsigned long)steps, half_us, STEP_PINS[j]);
  pulse(j, steps, true, half_us);
  delay(800);
  Serial.printf("[TEST] J%d: NGHICH %lu vi buoc...\n", j + 1, (unsigned long)steps);
  pulse(j, steps, false, half_us);
  Serial.printf("[TEST] J%d: XONG (STEP giu LOW)\n", j + 1);
}

void report() {
  for (int i = 0; i < 6; i++) {
    uint8_t status = drivers[i]->test_connection();
    Serial.printf("[TEST] J%d STEP=GPIO%-2d (muc %d) DIR=GPIO%-2d  TMC: %s, microsteps=%u\n", i + 1,
                  STEP_PINS[i], digitalRead(STEP_PINS[i]), DIR_PINS[i],
                  status == 0 ? "OK" : "KHONG TRA LOI (kiem tra nguon dong luc)", drivers[i]->microsteps());
  }
  Serial.printf("[TEST] EN=GPIO%d muc %d (0 = driver duoc phep chay)\n", ENABLE_PIN, digitalRead(ENABLE_PIN));
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);
  Serial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);
  // Giả thuyết: UART1 hàn đảo quy ước (TX nối thẳng PDN_UART). Dòng ra TX yếu (~5 mA) để driver
  // vẫn kéo được đường dây khi trả lời — giống điều kiện lúc bộ dò trong firmware chính báo OK.
  gpio_set_drive_capability((gpio_num_t)TX1_PIN, GPIO_DRIVE_CAP_0);
  gpio_set_drive_capability((gpio_num_t)TX2_PIN, GPIO_DRIVE_CAP_0);
  for (int i = 0; i < 6; i++) {
    pinMode(STEP_PINS[i], OUTPUT);
    digitalWrite(STEP_PINS[i], LOW);
    pinMode(DIR_PINS[i], OUTPUT);
    digitalWrite(DIR_PINS[i], LOW);
  }
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  delay(200);
  for (int i = 0; i < 6; i++) {
    drivers[i]->begin();
    drivers[i]->toff(4);
    drivers[i]->blank_time(24);
    drivers[i]->rms_current(MOTOR_CURRENTS[i]);
    drivers[i]->microsteps(16);
    drivers[i]->mstep_reg_select(true);
    drivers[i]->pdn_disable(true);
    drivers[i]->en_spreadCycle(true);
  }
  Serial.println("[TEST] FIRMWARE THU CHAN STEP san sang. Lenh: 1..6 = quay truc dong co 2 vong thuan/nghich, m J N H = tuy chinh, s = trang thai");
  report();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c >= '1' && c <= '6') {
    runTest(c - '1', TEST_STEPS, HALF_PERIOD_US);
  } else if (c == 'm') {
    long j = Serial.parseInt(), n = Serial.parseInt(), h = Serial.parseInt();
    if (j >= 1 && j <= 6 && n > 0 && n <= 32000 && h >= 50 && h <= 5000) {
      runTest(j - 1, n, h);
    } else {
      Serial.println("[TEST] Lenh m sai: m J N H (J 1-6, N 1-32000, H 50-5000 us)");
    }
  } else if (c == 's') {
    report();
  }
}
