// ================================================================
// BANG TAI + CAM BIEN QUANG — ESP32 DEVKIT V1 + W5500 + BTS7960
// Firmware v2.1 (2026-11-24), theo so do "BANG TAI VA CAM BIEN".
// v2.1: cam bien dat TRUOC diem gap -> kien chay them ~2.2 s sau khi cam bien thay roi moi dung
//   (AUTO:1 khong kem <ms> dung DEFAULT_AUTO_DELAY_MS); kien thu hai cham cam bien khong lui
//   thoi diem dung cua kien dau; STATUS them ASTOP=<0|1> (lan dung gan nhat do AUTO), vi luc
//   dung kien co the da qua khoi cam bien.
//
// Tuong thich nguoc voi GUI Machine Vision (controllers/conveyor_udp.py):
//   PING      -> PONG
//   FWD:<pwm> -> ACK:FWD:<pwm> (State: FWD Speed: <pwm>)
//   REV:<pwm> -> ACK:REV:<pwm> (State: REV Speed: <pwm>)
//   STOP:0    -> ACK:STOP:0 (State: STOP Speed: 0)
// Lenh moi (GUI hien tai khong gui, dung cho cong cu kiem tra / tich hop sau):
//   STATUS            -> STATUS:<dir>:<pwm>:SENSOR=<0|1>:COUNT=<n>:AUTO=<0|1>:DELAY=<ms>:ISR=<mV>:ISL=<mV>:UP=<s>:ASTOP=<0|1>
//   AUTO:<0|1>[:<ms>] -> ACK:AUTO:<0|1>:<ms>   (tu dung bang tai <ms> sau khi cam bien thay kien;
//                        bo trong <ms> = DEFAULT_AUTO_DELAY_MS)
//   RESETCOUNT        -> ACK:RESETCOUNT
//   Lenh sai          -> ERR:<ly do>           (ban cu tra ACK ca voi lenh sai)
// Su kien day len laptop (UDP, cong EVENT_PORT, toi IP cua host da ra lenh gan nhat):
//   EVT:SENSOR:<0|1>:<count>:<ms>   EVT:AUTOSTOP:<count>   EVT:WATCHDOG_STOP   EVT:LINK_DOWN_STOP
//   EVT:OVERCURRENT_STOP:<mV>
//
// An toan (firmware KHONG thay the nut dung khan vat ly):
//   - Watchdog: motor dang chay ma WATCHDOG_MS khong nhan goi hop le tu host dieu khien -> STOP.
//     GUI ping moi 1 s khi online va ngung ping khi mat phan hoi, nen watchdog 3 s du an toan.
//   - Rut cap LAN (LinkOFF) khi motor dang chay -> STOP.
//   - Doi chieu truc tiep FWD <-> REV: dung DIRECTION_DEADTIME_MS truoc khi dao.
//   - Qua dong (tuy chon): OVERCURRENT_MV > 0 thi dung khi R_IS/L_IS vuot nguong.
// ================================================================
#include <SPI.h>
#include <Ethernet.h>

// ---------------- 1. CHAN (theo so do) ----------------
// W5500 (SPI VSPI)
#define W5500_MOSI 23
#define W5500_MISO 19
#define W5500_SCLK 18
#define W5500_CS    5
#define W5500_RST   4      // -1 neu khong noi chan RST

// BTS7960 (module IBT-2)
#define RPWM_PIN   25
#define LPWM_PIN   26
#define R_EN_PIN   27
#define L_EN_PIN   14
#define R_IS_PIN   34      // chan input-only, ADC1: dien ap ty le dong nhanh phai
#define L_IS_PIN   35      // ADC1: dien ap ty le dong nhanh trai

// Cam bien quang (net "Proximity" tren so do), nguon 5V
#define SENSOR_PIN 13
// Cam bien kieu E18-D80NK (NPN, thuong mo): ngo ra keo xuong GND khi CO vat.
// Dat false neu cam bien cua ban xuat muc CAO khi co vat (PNP).
const bool SENSOR_ACTIVE_LOW = true;
const uint32_t SENSOR_DEBOUNCE_MS = 15;   // loc nhieu/rung

// ---------------- 2. MANG ----------------
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 2, 100);
IPAddress gateway(192, 168, 2, 10);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 2, 10);
const uint16_t LOCAL_PORT = 8120;   // nhan lenh (giu nhu ban cu)
const uint16_t EVENT_PORT = 8121;   // day su kien cam bien len laptop

// ---------------- 3. THONG SO ----------------
const int PWM_FREQ = 5000;          // Hz, LEDC
const int PWM_RES = 8;              // 0..255
const uint32_t WATCHDOG_MS = 3000;
const uint32_t DIRECTION_DEADTIME_MS = 150;
const uint32_t MONITOR_PERIOD_MS = 3000;
// BTS7960: I_IS ~ I_load / 8500, module IBT-2 dung dien tro 1 kOhm tren IS
// => ~0.12 V moi 1 A. Do thuc te roi moi dat nguong; 0 = tat bao ve qua dong.
const int OVERCURRENT_MV = 0;
const uint32_t OVERCURRENT_HOLD_MS = 200;  // phai vuot nguong lien tuc
// Cam bien nam truoc diem gap: tu luc cam bien thay mep truoc cua kien toi luc dung de robot gap
// ~1-1.5 s (uoc luong cua nguoi van hanh 24/11/2026). PC gui gia tri rieng qua AUTO:1:<ms>
// (config station_cycle.pick_stop_delay_ms) nen chinh duoc ma khong can nap lai.
const uint32_t DEFAULT_AUTO_DELAY_MS = 2200;   // 24/11: 1200 -> 2200 (chay them 1 s theo nguoi van hanh)

EthernetUDP Udp;

// ---------------- 4. TRANG THAI ----------------
enum Direction { DIR_STOP, DIR_FWD, DIR_REV };
Direction direction = DIR_STOP;
int speedPwm = 0;

bool sensorActive = false;          // sau khi loc nhieu
bool sensorRawLast = false;
uint32_t sensorChangedAt = 0;
uint32_t objectCount = 0;

bool autoStopEnabled = false;
uint32_t autoStopDelayMs = DEFAULT_AUTO_DELAY_MS;
bool autoStopPending = false;
uint32_t autoStopAt = 0;
bool autoStopped = false;           // lan dung gan nhat do AUTO (xoa khi chay lai)

IPAddress controllerIp(0, 0, 0, 0); // host da gui lenh hop le gan nhat
bool haveController = false;
uint32_t lastControllerPacket = 0;

EthernetLinkStatus lastLink = Unknown;
uint32_t lastMonitor = 0;
uint32_t overcurrentSince = 0;

// ---------------- 5. DONG CO ----------------
const char* dirName(Direction d) {
  return d == DIR_FWD ? "FWD" : (d == DIR_REV ? "REV" : "STOP");
}

void stopMotor() {
  ledcWrite(RPWM_PIN, 0);
  ledcWrite(LPWM_PIN, 0);
  digitalWrite(R_EN_PIN, LOW);
  digitalWrite(L_EN_PIN, LOW);
  direction = DIR_STOP;
  speedPwm = 0;
  autoStopPending = false;
}

void runMotor(Direction d, int pwm) {
  if (direction != DIR_STOP && direction != d) {
    // Dao chieu khi dang chay: ngat ca hai nhanh mot luc de tranh xung dong.
    stopMotor();
    delay(DIRECTION_DEADTIME_MS);
  }
  // Lenh chay moi: ket qua dung AUTO truoc khong con dung; dung AUTO chi ap dung khi chay toi.
  autoStopped = false;
  if (d != DIR_FWD) autoStopPending = false;
  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);
  if (d == DIR_FWD) {
    ledcWrite(LPWM_PIN, 0);
    ledcWrite(RPWM_PIN, pwm);
  } else {
    ledcWrite(RPWM_PIN, 0);
    ledcWrite(LPWM_PIN, pwm);
  }
  direction = d;
  speedPwm = pwm;
}

// ---------------- 6. UDP ----------------
void reply(const char* text) {
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(reinterpret_cast<const uint8_t*>(text), strlen(text));
  Udp.endPacket();
  Serial.printf("[PHAN HOI] %s\n", text);
}

void sendEvent(const char* text) {
  Serial.printf("[SU KIEN] %s\n", text);
  if (!haveController) return;
  Udp.beginPacket(controllerIp, EVENT_PORT);
  Udp.write(reinterpret_cast<const uint8_t*>(text), strlen(text));
  Udp.endPacket();
}

// Doc so nguyen 0..max tu chuoi, tra false neu khong hop le.
bool parseUint(const char* text, long maxValue, long* out) {
  if (text == nullptr || *text == '\0') return false;
  char* end = nullptr;
  long value = strtol(text, &end, 10);
  if (*end != '\0' || value < 0 || value > maxValue) return false;
  *out = value;
  return true;
}

void handleCommand(char* cmd) {
  char buffer[160];
  // Tach "ACTION:ARG1:ARG2"
  char* action = strtok(cmd, ":");
  char* arg1 = strtok(nullptr, ":");
  char* arg2 = strtok(nullptr, ":");
  if (action == nullptr) { reply("ERR:EMPTY"); return; }

  if (strcmp(action, "PING") == 0 && arg1 == nullptr) {
    reply("PONG");
  } else if ((strcmp(action, "FWD") == 0 || strcmp(action, "REV") == 0) && arg2 == nullptr) {
    long pwm;
    if (!parseUint(arg1, 255, &pwm) || pwm < 1) { reply("ERR:PWM_1_255"); return; }
    runMotor(strcmp(action, "FWD") == 0 ? DIR_FWD : DIR_REV, (int)pwm);
    snprintf(buffer, sizeof(buffer), "ACK:%s:%ld (State: %s Speed: %d)", action, pwm,
             dirName(direction), speedPwm);
    reply(buffer);
  } else if (strcmp(action, "STOP") == 0 && arg2 == nullptr) {
    stopMotor();
    reply("ACK:STOP:0 (State: STOP Speed: 0)");
  } else if (strcmp(action, "STATUS") == 0 && arg1 == nullptr) {
    snprintf(buffer, sizeof(buffer),
             "STATUS:%s:%d:SENSOR=%d:COUNT=%lu:AUTO=%d:DELAY=%lu:ISR=%d:ISL=%d:UP=%lu:ASTOP=%d",
             dirName(direction), speedPwm, sensorActive ? 1 : 0, (unsigned long)objectCount,
             autoStopEnabled ? 1 : 0, (unsigned long)autoStopDelayMs,
             (int)analogReadMilliVolts(R_IS_PIN), (int)analogReadMilliVolts(L_IS_PIN),
             (unsigned long)(millis() / 1000), autoStopped ? 1 : 0);
    reply(buffer);
  } else if (strcmp(action, "AUTO") == 0) {
    long enabled, delayMs = DEFAULT_AUTO_DELAY_MS;
    if (!parseUint(arg1, 1, &enabled) || (arg2 != nullptr && !parseUint(arg2, 10000, &delayMs))) {
      reply("ERR:AUTO_FORMAT_AUTO:<0|1>:<0..10000ms>");
      return;
    }
    autoStopEnabled = enabled == 1;
    autoStopDelayMs = (uint32_t)delayMs;
    autoStopPending = false;
    snprintf(buffer, sizeof(buffer), "ACK:AUTO:%d:%lu", autoStopEnabled ? 1 : 0,
             (unsigned long)autoStopDelayMs);
    reply(buffer);
  } else if (strcmp(action, "RESETCOUNT") == 0 && arg1 == nullptr) {
    objectCount = 0;
    reply("ACK:RESETCOUNT");
  } else {
    reply("ERR:UNKNOWN_COMMAND");
    return;  // lenh sai khong duoc tinh la heartbeat
  }

  // Chi goi hop le moi nuoi watchdog va xac dinh host nhan su kien.
  controllerIp = Udp.remoteIP();
  haveController = true;
  lastControllerPacket = millis();
}

void pollUdp() {
  int size = Udp.parsePacket();
  if (size <= 0) return;
  char packet[96];
  int len = Udp.read(packet, sizeof(packet) - 1);
  if (len < 0) len = 0;
  packet[len] = '\0';
  // Bo khoang trang/CR/LF hai dau.
  while (len > 0 && isspace((unsigned char)packet[len - 1])) packet[--len] = '\0';
  char* cmd = packet;
  while (*cmd && isspace((unsigned char)*cmd)) cmd++;
  Serial.printf("[UDP] %s:%u -> '%s'\n", Udp.remoteIP().toString().c_str(), Udp.remotePort(), cmd);
  if (size >= (int)sizeof(packet)) { reply("ERR:TOO_LONG"); return; }
  bool fromController = !haveController || Udp.remoteIP() == controllerIp;
  bool wasRunning = direction != DIR_STOP;
  handleCommand(cmd);
  if (wasRunning && !fromController) {
    Serial.println("[CANH BAO] Host moi dieu khien khi motor dang chay");
  }
}

// ---------------- 7. CAM BIEN ----------------
bool readSensorRaw() {
  bool level = digitalRead(SENSOR_PIN) == HIGH;
  return SENSOR_ACTIVE_LOW ? !level : level;
}

void pollSensor() {
  bool raw = readSensorRaw();
  uint32_t now = millis();
  if (raw != sensorRawLast) {
    sensorRawLast = raw;
    sensorChangedAt = now;
  }
  if (raw == sensorActive || now - sensorChangedAt < SENSOR_DEBOUNCE_MS) return;

  sensorActive = raw;
  char buffer[64];
  if (sensorActive) {
    objectCount++;
    // Chi kien DAU kich dung: kien sau cham cam bien trong luc dang dem thi khong
    // day lui thoi diem dung (kien dau se troi qua diem gap).
    if (autoStopEnabled && direction == DIR_FWD && !autoStopPending) {
      autoStopPending = true;
      autoStopAt = now + autoStopDelayMs;
    }
  }
  snprintf(buffer, sizeof(buffer), "EVT:SENSOR:%d:%lu:%lu", sensorActive ? 1 : 0,
           (unsigned long)objectCount, (unsigned long)now);
  sendEvent(buffer);
}

void pollAutoStop() {
  if (!autoStopPending || (int32_t)(millis() - autoStopAt) < 0) return;
  autoStopPending = false;
  if (direction != DIR_FWD) return;
  stopMotor();
  autoStopped = true;
  char buffer[40];
  snprintf(buffer, sizeof(buffer), "EVT:AUTOSTOP:%lu", (unsigned long)objectCount);
  sendEvent(buffer);
}

// ---------------- 8. GIAM SAT AN TOAN ----------------
void pollSafety() {
  uint32_t now = millis();
  if (direction != DIR_STOP && now - lastControllerPacket > WATCHDOG_MS) {
    stopMotor();
    sendEvent("EVT:WATCHDOG_STOP");
  }

  if (OVERCURRENT_MV > 0 && direction != DIR_STOP) {
    int mv = max((int)analogReadMilliVolts(R_IS_PIN), (int)analogReadMilliVolts(L_IS_PIN));
    if (mv < OVERCURRENT_MV) {
      overcurrentSince = 0;
    } else if (overcurrentSince == 0) {
      overcurrentSince = now ? now : 1;
    } else if (now - overcurrentSince >= OVERCURRENT_HOLD_MS) {
      stopMotor();
      overcurrentSince = 0;
      char buffer[40];
      snprintf(buffer, sizeof(buffer), "EVT:OVERCURRENT_STOP:%d", mv);
      sendEvent(buffer);
    }
  }

  if (now - lastMonitor < MONITOR_PERIOD_MS) return;
  lastMonitor = now;
  EthernetLinkStatus link = Ethernet.linkStatus();
  if (link != lastLink) {
    lastLink = link;
    Serial.printf(">>> [CAP MANG] %s\n", link == LinkON ? "LinkON" : (link == LinkOFF ? "LinkOFF" : "Unknown"));
  }
  if (link == LinkOFF && direction != DIR_STOP) {
    stopMotor();
    Serial.println("[SU KIEN] EVT:LINK_DOWN_STOP (khong gui duoc vi mat cap)");
  }
  Serial.printf("[MONITOR %4lus] %-4s PWM %3d | cam bien %s | dem %lu | auto %s %lu ms | IS R/L %d/%d mV\n",
                (unsigned long)(now / 1000), dirName(direction), speedPwm,
                sensorActive ? "CO VAT " : "trong  ", (unsigned long)objectCount,
                autoStopEnabled ? "ON " : "OFF", (unsigned long)autoStopDelayMs,
                (int)analogReadMilliVolts(R_IS_PIN),
                (int)analogReadMilliVolts(L_IS_PIN));
}

// ---------------- 9. SETUP / LOOP ----------------
void setup() {
  // Dong co tat ngay tu dau, truoc moi thu khac.
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);
  digitalWrite(R_EN_PIN, LOW);
  digitalWrite(L_EN_PIN, LOW);
  ledcAttach(RPWM_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(LPWM_PIN, PWM_FREQ, PWM_RES);
  stopMotor();

  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== BANG TAI + CAM BIEN | ESP32 + W5500 + BTS7960 | FW v2.1 ===");
  Serial.printf("BTS7960: RPWM=%d LPWM=%d R_EN=%d L_EN=%d R_IS=%d L_IS=%d\n",
                RPWM_PIN, LPWM_PIN, R_EN_PIN, L_EN_PIN, R_IS_PIN, L_IS_PIN);

  // GPIO13 co pull-up noi ~45 kOhm len 3.3V, du cho ngo ra NPN cuc thu ho.
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  analogSetPinAttenuation(R_IS_PIN, ADC_11db);
  analogSetPinAttenuation(L_IS_PIN, ADC_11db);
  sensorRawLast = sensorActive = readSensorRaw();
  Serial.printf("Cam bien GPIO%d: muc %s -> %s (active %s)\n", SENSOR_PIN,
                digitalRead(SENSOR_PIN) ? "HIGH" : "LOW", sensorActive ? "CO VAT" : "trong",
                SENSOR_ACTIVE_LOW ? "LOW" : "HIGH");

  if (W5500_RST >= 0) {
    pinMode(W5500_RST, OUTPUT);
    digitalWrite(W5500_RST, LOW);
    delay(20);
    digitalWrite(W5500_RST, HIGH);
    delay(200);
  }
  SPI.begin(W5500_SCLK, W5500_MISO, W5500_MOSI, W5500_CS);
  Ethernet.init(W5500_CS);
  Ethernet.begin(mac, ip, dns, gateway, subnet);

  while (Ethernet.hardwareStatus() != EthernetW5500) {
    // Van theo doi cam bien qua Serial de kiem tra phan cung khi chua co mang.
    Serial.println("[LOI] Khong thay W5500: kiem tra MOSI23 MISO19 SCK18 CS5 RST4, 3.3V, GND");
    uint32_t until = millis() + 3000;
    while ((int32_t)(millis() - until) < 0) { pollSensor(); delay(5); }
    Ethernet.begin(mac, ip, dns, gateway, subnet);
  }

  Udp.begin(LOCAL_PORT);
  Serial.print("IP "); Serial.print(Ethernet.localIP());
  Serial.printf(" | lenh UDP %u | su kien UDP %u | watchdog %lu ms\n", LOCAL_PORT, EVENT_PORT,
                (unsigned long)WATCHDOG_MS);
  Serial.println("San sang.");
}

void loop() {
  pollUdp();
  pollSensor();
  pollAutoStop();
  pollSafety();
}
