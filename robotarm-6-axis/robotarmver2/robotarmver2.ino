#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <TMCStepper.h>
#include <Preferences.h>
#include "soc/gpio_reg.h"

// ================= CẤU HÌNH MẠNG ETHERNET W5500 =================
// 24/11/2026: schematic Robotarm V1.0 (EasyEDA, 24/11/2026) KHÔNG có W5500; GPIO10, GPIO11 nối IN1, IN3 của module
// L298N (bơm hút, van điện từ). SPI W5500 dùng đúng 2 chân này → TẮT Ethernet (USE_W5500 0), chỉ điều khiển qua USB.
// Muốn dùng lại LAN phải dời bơm/van sang chân khác rồi đặt USE_W5500 1.
#define USE_W5500 0
#define W5500_CS_PIN 10
#define SPI_MOSI_PIN 11
#define SPI_MISO_PIN 12
#define SPI_SCK_PIN  13

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 177);
unsigned int localPort = 8888;

EthernetUDP Udp;

// ================= CẤU HÌNH GÓI TIN NHỊ PHÂN & CRC16 =================
#define HEADER_COMMAND  0xAA55
#define HEADER_FEEDBACK 0x55AA

#define CMD_STOP      0
#define CMD_MOVE_PC   1
#define CMD_REC_WP    2
#define CMD_PLAY_AUTO 3
#define CMD_TOOL_ONLY 4
#define CMD_SET_ZERO  5

struct __attribute__((packed)) RobotCommandPacket {
  uint16_t header;       // 0xAA55
  uint8_t  command_type; // 0: STOP, 1: MOVE_PC, 2: REC_WP, 3: PLAY_AUTO, 4: TOOL_ONLY, 5: SET_ZERO
  uint8_t  tool_state;   // 0: OFF, 1: SUCK, 2: RELEASE
  float    target_angles[6];
  uint16_t crc16;
};

struct __attribute__((packed)) RobotFeedbackPacket {
  uint16_t header;        // 0x55AA
  uint8_t  robot_mode;    // 0: MODE_PC, 1: MODE_AUTO
  uint8_t  current_tool;  // 0: OFF, 1: SUCK, 2: RELEASE
  float    actual_angles[6];
  uint16_t crc16;
};

uint16_t calc_crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc & 0xFFFF;
}

// ================= PIN CẤU HÌNH ĐỘNG CƠ =================
#define ENABLE_PIN 14
#define RX1_PIN 17
#define TX1_PIN 18
#define RX2_PIN 8
#define TX2_PIN 9

const uint8_t STEP_PINS[6] = {1, 4, 6, 15, 40, 38};
const uint8_t DIR_PINS[6]  = {2, 5, 7, 16, 41, 39};

// ================= BƠM HÚT / VAN ĐIỆN TỪ QUA MODULE L298N (24/11/2026, theo schematic) =================
// L298N U8: OUT1/OUT2 → bơm chân không (BOM+/BOM−), OUT3/OUT4 → van điện từ (VAN+/VAN−), nguồn +12V.
//   IN1 = GPIO10 (bơm), IN2 = GND;  IN3 = GPIO11 (van), IN4 = GND;  ENA/ENB giữ jumper trên module (luôn bật).
//   → mức CAO = cấp điện (khác rơ-le cũ kích mức THẤP). Không có W5500 trên mạch (xem USE_W5500).
// Logic hút (van 3 ngã thường đóng phía khí trời — loại hay đi kèm bơm hút):
//   HÚT  : van TẮT (giác hút thông bơm) → bơm CHẠY.
//   NHẢ  : bơm TẮT → van MỞ ra khí trời VALVE_RELEASE_MS rồi tự TẮT hết (cuộn van không bị nóng, kiện rơi hẳn).
//   TẮT  : bơm + van tắt.
// Nếu van của bạn là loại thường đóng nằm GIỮA bơm và giác hút (phải cấp điện mới hút) → đặt VALVE_OPEN_WHEN_SUCK 1.
#define PUMP_PIN 10
#define VALVE_PIN 11
#define OUT_ON  HIGH
#define OUT_OFF LOW
#define VALVE_OPEN_WHEN_SUCK 0
const uint32_t VALVE_RELEASE_MS = 600;   // thời gian mở van xả chân không khi NHẢ

#define TOOL_OFF 0
#define TOOL_SUCK 1
#define TOOL_RELEASE 2

uint8_t current_tool_state = TOOL_OFF;   // trạng thái thật của bơm/van (gửi về GUI qua telemetry)
uint8_t last_cmd_tool = 255;             // lệnh cuối nhận được — gói MOVE lặp cùng lệnh không kích lại xung xả
uint32_t release_t0 = 0;

// ================= THÔNG SỐ ĐỘNG CƠ & HỘP SỐ =================
// 06/11/2026: J1–J3 thay 42STH38-1684A + hộp số hành tinh 1:27 (5400 xung đủ/vòng trục ra). Đã giảm tốc rất
// mạnh nên dùng 8 vi bước (≈0.0083°/vi bước ở J2) để giảm một nửa tần số xung; J4–J6 giữ 16.
const uint16_t MICROSTEPS_J[6] = {8, 8, 8, 16, 16, 16};
// Số bước đủ/vòng của động cơ bước 1.8° (NEMA17). Bị xóa nhầm ở bản 16/10 → lỗi biên dịch.
const float STEPS_PER_REV = 200.0;
// Cân chỉnh dòng điện tối ưu sức kéo và chống quá nhiệt (kèm tản nhiệt TMC2209):
// J1: 1000mA, J2: 850mA (tăng từ 650mA), J3: 700mA (tăng từ 500mA), J4: 500mA, J5: 800mA, J6: 800mA
// J6 = 17HS3401-S (datasheet người dùng gửi 03/11: 1.3 A/pha, 2.4 Ω, 2.8 mH, giữ 28 N·cm):
// 900 mA RMS → đỉnh 1.27 A ≤ 1.3 A định mức (trước 800 mA ≈ 89% mô-men so với 900 mA).
// J1–J3 (42STH38-1684A: 1.68 A/pha, 1.6 Ω): 1100 mA RMS → đỉnh 1.56 A ≤ 1.68 A; driver PHẢI có tản nhiệt + quạt.
//
// 14/11/2026 — TĂNG MÔ-MEN (nguồn 24 V). Dòng định mức trên datasheet là dòng DC/pha khi 2 pha cùng bật; khi vi
// bước hình sin, mô-men giữ và nhiệt trên dây quấn bằng định mức khi dòng RMS = dòng định mức (đỉnh = √2·định mức).
// → mô-men ≈ (RMS đặt / định mức) × mô-men datasheet:
//   J1–J3 SY42STH38-1684A (1.68 A, 1.65 Ω, 36 N·cm): 1500 mA = 89 % (trước 1100 mA = 65 %). Tối đa 1680 mA;
//       module TMC2209 khuyến nghị ≤ 1.7 A và bắt buộc tản nhiệt + quạt, nên để 1500 làm mặc định.
//   J4 (đổi 17PM ngày 17/11, TRỞ LẠI 19/11/2026) 17HS13-0404S-PG5 (0.4 A, 27 Ω, 0.25 N·m × 5.18): 400 mA = 100 %. (500 mA cũ vượt định mức 25 %; ở 12 V
//       cuộn 27 Ω không đạt nổi đỉnh 0.57 A — 24 V mới đạt.)
//   J5 17PM-K4xx (MinebeaMitsumi, thân 48 mm; không rõ mã cuộn: 0.9 A/5.6 Ω hoặc 1.5 A/2.4 Ω): 900 mA an toàn cho
//       mọi mã. Đo điện trở 1 cuộn ≈ 2.4 Ω thì có thể nâng tới 1400.
//   J6 17HS3401 (1.3 A, 2.4 Ω, 28 N·cm): 900 mA = 69 %, tải nhẹ nên giữ.
//
// 15/11/2026 — J2, J3 LÊN MỨC TỐI ĐA của module (người dùng báo lắp vào tay vẫn không nâng nổi).
//   Với Rsense 0.11 Ω, nấc dòng cao nhất của TMC2209 là CS=31 → 0.325 V/(0.11+0.02 Ω)/√2 = 1.77 A RMS (đỉnh 2.5 A).
//   Đặt 1770 → CS=31 = 105 % dòng định mức động cơ (1.68 A) → mô-men ≈ +18 % so với 1500 mA. KHÔNG thể cao hơn
//   bằng firmware (muốn hơn phải đổi driver/động cơ/tỉ số truyền). Nhiệt động cơ khi chạy ≈ 110 % định mức,
//   driver BẮT BUỘC tản nhiệt + quạt thổi trực tiếp (quá nhiệt → driver tự ngắt → tay rơi).
//
// 17/11/2026 — J4 ĐỔI SANG 17PM-K4xx NỐI THẲNG (bỏ hộp số PG5 5.18). Người dùng báo J4 rung lắc tại chỗ ở 400 mA:
//   400 mA chỉ cho ≈ 0.2 N·m, khớp cần tới 0.39 N·m (J5 gập 90°, tải 200 g) → mất bước. Đặt 900 mA như J5 (cùng
//   dòng động cơ, an toàn cho mọi mã cuộn: K442B 0.9 A = 100 %, K448B 1.5 A = 60 %). Nếu đo 1 cuộn ≈ 2.4 Ω (K448B)
//   có thể nâng tới 1400 mA; nếu nhãn là K449U (4 dây đấu nối tiếp) hạ còn 700 mA.
//
// 19/11/2026 — J4 TRỞ LẠI ĐỘNG CƠ CŨ 17HS13-0404S-PG5 (người dùng bỏ phương án 17PM). 400 mA = 100 % định mức
//   (0.4 A, 27 Ω). KHÔNG đặt cao hơn: 27 Ω × đỉnh 0.57 A = 15.4 V đã chiếm 2/3 nguồn 24 V, dòng cao hơn driver cũng
//   không đẩy nổi khi quay nhanh, chỉ làm nóng động cơ.
//
// 21/11/2026 — J2 HẠ DÒNG ĐỂ BẢO VỆ ĐAI 2GT-6 mm (đai 240 đứt khi J3–J6 duỗi thẳng và J2 nâng từ dưới đất lên).
//   Đai J2 nằm SAU hộp số 1:27 → lực đai = mô-men trục hộp số / bán kính bước puly 20T (12.73/2 = 6.37 mm).
//   1770 mA: 0.36 N·m × 1.05 × 27 × 0.8 (hiệu suất hộp số) ≈ 8.2 N·m → ≈ 1280 N trên nhánh căng, gấp nhiều lần sức
//   chịu của đai cao su 6 mm. Khi khớp bị kẹt / chạm đất / đổi chiều đột ngột, toàn bộ lực này dồn vào đai.
//   Lực đai tối đa ≈ 0.72 N/mA × dòng RMS (đặt 1200 mA ≈ 870 N; 900 mA ≈ 650 N).
//   1200 mA làm mặc định (≈ 68 % mô-men cũ). Dòng J2 chỉnh được khi đang chạy: $I 2 <mA>, lưu bằng $S.
//   Cách chỉnh: duỗi thẳng J3–J6, nâng J2 thật chậm, dò dòng THẤP NHẤT mà J2 vẫn nâng được ở tư thế xấu nhất,
//   rồi cộng thêm 15–20 % dự phòng. Nếu cần > 1400 mA mới nâng nổi thì tải trọng lực đã vượt sức đai → phải bù trọng
//   lực bằng lò xo hoặc đổi đai lõi thép rộng hơn; firmware không thể làm đai khỏe hơn.
// 23/11/2026 — LẮP LẠI ĐAI 240-2GT-6 (đã căng lại). Đai này từng đứt ở 1770 mA (≈ 1280 N) → giới hạn lực đai THEO DÒNG:
//   Mô-men trọng lực J2 xấu nhất (J3–J6 duỗi ngang, ước tính từ CAD) ≈ 3.6 N·m ÷ bán kính puly 80T 25.5 mm ≈ 141 N
//   ≈ 196 mA. Đặt 500 mA → lực đai tối đa ≈ 360 N (gấp 2.5 lần cần thiết, = 28 % lực lúc đai đứt). Giữ 0.8 × 500 =
//   400 mA ≈ 290 N > 141 N nên vẫn giữ được tay duỗi ngang. Kẹt/chạm vật: động cơ TRƯỢT BƯỚC trước khi đai chịu quá 360 N.
//   Trần cứng J2_CURRENT_MAX_MA = 700 mA (≈ 500 N): $I 2 không vượt được, dòng cũ lưu Flash cao hơn bị bỏ.
//   Nếu J2 trượt bước khi nâng tay duỗi thẳng: tăng từng nấc 50 mA ($I 2 550, $I 2 600 …), tối đa 700.
// 23/11/2026 — J5 900 → 1200 mA (người dùng: nhiều lúc không nâng nổi đầu hút). Mô-men +33 %. Mã cuộn 17PM-K4xx chưa
//   rõ: nếu là bản 0.9 A thì 1200 mA = 133 % định mức khi CHẠY → động cơ ấm hơn; dòng GIỮ hạ còn 0.7 × 1200 = 840 mA
//   (≤ 0.9 A) để đứng yên lâu không quá nhiệt. Đo 1 cuộn ≈ 2.4 Ω (bản 1.5 A) thì nâng được tới 1400: $I 5 1400, $S.
//   Dòng J5 cũ lưu Flash thấp hơn 1200 (900 của bản trước) tự nâng lên 1200 khi khởi động.
// 24/11/2026 — J5 1200 → 1400 mA (người dùng: J5 vẫn không nâng được J6 + đầu hút ở tư thế 0 mới, cổ tay gập 90°).
//   1400 mA = trần an toàn cho bản cuộn 1.5 A/2.4 Ω (93 %). Nếu đo 1 cuộn J5 ≈ 5.6 Ω (bản 0.9 A) thì 1400 = 155 %:
//   động cơ nóng nhanh → hạ về $I 5 1000, $S. Dòng giữ 0.7 × 1400 = 980 mA. Tốc độ J5 hạ còn 45 °/s (xem JOINT_SPEED_DEG)
//   để giữ mô-men chạy gần mô-men giữ.
const uint16_t MOTOR_CURRENTS[6] = {1500, 500, 1770, 400, 1400, 900};
const uint16_t CURRENT_MIN_MA = 300;    // giới hạn cho lệnh $I
const uint16_t CURRENT_MAX_MA = 1770;   // nấc CS=31 của module (Rsense 0.11 Ω)
const uint16_t J2_CURRENT_MAX_MA = 700; // trần riêng J2 bảo vệ đai 240-2GT-6 (≈ 500 N)
uint16_t motor_current[6];              // dòng chạy đang dùng (mA RMS); mặc định = MOTOR_CURRENTS, đổi bằng $I

// 22/11/2026 — KHÓA J2 BẰNG ĐIỆN (động cơ J2 vẫn cấp dòng, giữ cứng tại một góc; chỉ dùng J1, J3–J6 để gắp).
//   Khi khóa, đích của J2 LUÔN là góc khóa, bất kể lệnh từ PC (slider, HOME, điểm dạy, IK) hay chế độ AUTO.
//   J2 đứng yên → đai chỉ chịu tải trọng lực tĩnh, không còn lực tăng tốc/giật khi nâng hạ tay.
//   Lệnh USB: $J <độ> đưa J2 tới góc đó (chậm, theo giới hạn 20 °/s) rồi khóa | $J khóa tại góc HIỆN TẠI
//             $J U mở khóa | $S lưu Flash (khóa còn sau khi khởi động lại).
// 24/11/2026 — KHÓA CỨNG J2 TRONG FIRMWARE (người dùng yêu cầu): J2 luôn khóa ở 0° = góc J2 của tư thế HOME THẬT
//   (lần 3: cánh tay trên CÚI VỀ TRƯỚC ≈12.5° so với đứng thẳng — "Lưu Gốc" đúng tư thế đó). Cánh tay trên đứng thẳng → mô-men trọng lực lên đai J2 gần 0. $J / $J U / $R / Flash
//   không mở khóa được. Muốn mở lại: đặt J2_HARD_LOCK 0 rồi nạp lại.
#define J2_HARD_LOCK 1
const float J2_HARD_LOCK_DEG = 0.0f;
bool j2_locked = J2_HARD_LOCK;
float j2_lock_deg = J2_HARD_LOCK_DEG;
// 24/11/2026 — GIỚI HẠN MỀM J3 (độ, quy ước HOME THẬT lần 3: 0 = cẳng tay chúc 25° dưới phương ngang, + = chúc thêm).
//   Lần 3: [-225, 45] → [-220, 20] (quét va chạm mesh cẳng tay ↔ cánh tay trên, dời theo home mới).
//   Firmware kẹp mọi lệnh J3 (PC, AUTO, điểm dạy) vào khoảng này. Đồng bộ với donghoc.joint_limits + slider GUI.
const float J3_MIN_DEG = -220.0f;
const float J3_MAX_DEG = 20.0f;
// Tỉ lệ dòng GIỮ (đứng yên) so với dòng chạy. Thư viện TMCStepper mặc định 0.5 → khớp chịu trọng lực chỉ còn
// một nửa mô-men khi dừng (tay tự tụt). J2, J3, J5 nâng tải nên giữ 0.7–0.8; J1, J6 không chịu trọng lực: 0.5.
// Nhiệt động cơ khi giữ ∝ I²: J2/J3 giữ 1.05 A ≈ 39 % tổn hao định mức (tăng nhiệt ≈ 30 °C) — chịu được với nhựa in.
// 15/11/2026: J2, J3 giữ 0.8 × 1.77 A = 1.42 A (trước 1.05 A, +35 % mô-men giữ). Không để 1.0 vì đứng yên lâu ở
// 100 % dòng làm vỏ động cơ lên 80–90 °C, mềm chi tiết PLA kẹp động cơ.
// 19/11/2026: J4 về động cơ cũ có hộp số 5.18 (hộp số tự giữ tốt) → giữ 0.7 như trước.
// 23/11/2026: J5 0.8 → 0.7 (dòng chạy lên 1200 mA, giữ 840 mA ≤ định mức bản cuộn 0.9 A).
// 24/11/2026 — KHÓA J4, J5, J6 SAU MỖI LỆNH (người dùng yêu cầu: dây bơm hút kéo xoay cổ tay khi đứng yên).
//   Driver luôn bật (EN = LOW), nhưng trước đây khi đứng yên dòng hạ còn 70 % (J4, J5) và 50 % (J6) → mô-men giữ yếu, J6
//   (1:1, không hộp số) bị dây kéo xoay mất bước. Nay J4–J6 giữ 100 % dòng chạy khi đứng yên = mô-men giữ tối đa
//   (J6 gấp 2 lần, J4/J5 gấp 1.43 lần). Đổi lại động cơ cổ tay nóng hơn khi chờ: sờ vỏ sau 10 phút, > 70 °C thì đặt
//   WRIST_HOLD_LOCK 0 hoặc hạ dòng chạy khớp đó bằng $I. Dòng giữ = dòng chạy, nên $I cũng đổi luôn dòng giữ.
#define WRIST_HOLD_LOCK 1
#if WRIST_HOLD_LOCK
const float HOLD_MULT[6] = {0.5, 0.8, 0.8, 1.0, 1.0, 1.0};
#else
const float HOLD_MULT[6] = {0.5, 0.8, 0.8, 0.7, 0.7, 0.5};
#endif
// Tỉ số truyền tổng = hộp số × đai (06/11/2026):
//   J1: 27 × đai 20T→60T (3.0) = 81      J2: 27 × đai 20T→80T (4.0) = 108     J3: 27 × đai 20T→60T (3.0) = 81
//   J4: PG5 5.18 (trực tiếp)              J5: đai 2.5 (đo thực tế 10/2026)    J6: 1:1
// Số răng puly lớn J1–J3 ƯỚC LƯỢNG từ đường kính rãnh trong Roboter.STEP (Ø37.6 ≈ 60T, Ø50.5 ≈ 80T) — ĐẾM RĂNG
// hoặc chạy tests/hieu_chinh_khop.py (nhập góc thật) để chốt. Nếu khớp không có đai: tỉ số = 27.
// 19/11/2026: J4 về 5.181818 (động cơ cũ 17HS13-0404S-PG5, hộp số hành tinh 5.18:1).
const float GEAR_RATIOS[6] = {81.0, 108.0, 81.0, 5.181818, 2.5, 1.0};
// Hệ số chiều quay thực tế động cơ (+1: Thuận, -1: Đảo chiều theo chiều đấu dây cuộn motor)
// Đặt DIR_SIGNS[4] = 1.0 để Khớp 5 (J5) gập xuống đúng hướng khi kéo slider xuống góc âm
// 04/11/2026: J3 = -1.0 — người dùng quan sát J3 thật quay NGƯỢC chiều J3 trên digital twin khi lệnh góc dương.
// Dấu nằm trong steps_per_degree nên góc telemetry (Act) vẫn cùng dấu với lệnh.
// 15/11/2026: J3 về +1.0 — sau khi thay động cơ mới (màu dây/cặp cuộn khác động cơ cũ) người dùng báo J3 lại ngược twin.
// 22/11/2026: tư thế 0 dựng thẳng đứng — người dùng đối chiếu twin: J1, J3, J4, J5 quay NGƯỢC (lệnh − ra +), J2, J6 đúng.
// → đảo J1, J3, J4, J5. Chiều đã lưu Flash ($S) cũng được đảo đúng 1 lần khi khởi động (khóa NVS "dir22").
const float DIR_SIGNS[6] = {-1.0, 1.0, -1.0, -1.0, -1.0, 1.0};
const bool DIR_FLIP_22[6] = {true, false, true, true, true, false};
float steps_per_degree[6];
// GEAR_RATIOS / DIR_SIGNS ở trên là MẶC ĐỊNH. Giá trị đang dùng nằm trong gear_ratio[] / dir_sign[], có thể hiệu chỉnh
// khi đang chạy bằng lệnh chữ qua USB (không cần nạp lại) và lưu Flash NVS — xem handleCalLine():
//   $P            in giá trị đang dùng          $K <khớp 1-6> <tỉ số>   đặt tỉ số truyền
//   $D <khớp> <1|-1>  đặt chiều quay            $S  lưu vào Flash        $R  xóa hiệu chỉnh, về mặc định
// tests/hieu_chinh_khop.py dùng các lệnh này sau khi người dùng nhập chiều quay và góc đo thật.
float gear_ratio[6];
float dir_sign[6];

// Vận tốc và gia tốc tối ưu hóa riêng biệt từng khớp:
// - J1 (Đế): Tốc độ 2500 steps/s, gia tốc 800 steps/s² (giảm từ 1000 để khử quán tính lật)
// - J2, J3 (Vai & Khuỷu): Khối lượng nặng, quán tính lớn -> hạ gia tốc xuống 450-500 steps/s² để không trượt bước
// - J4 (Cổ tay xoay): Tỉ số 5.18, tốc độ 2000 steps/s, gia tốc 800 steps/s²
// - J5 (Cổ tay gập): Tỉ số 2.5:1, tốc độ 2200 steps/s, gia tốc 1000 steps/s²
// - J6 (Đầu xoay trục cuối): Tỉ số 1:1 (~8.89 steps/độ), tải nhẹ -> gia tốc 1800 steps/s², tốc độ 3000 steps/s để đáp ứng tức thì, triệt tiêu hoàn toàn giật cục!
// J1–J3 (06/11): 4800 bước/s ở 8 vi bước = 3 vòng/s trục động cơ (an toàn mô-men ở 12 V) →
//   J1, J3 ≈ 13.3°/s; J2 ≈ 10°/s. Gia tốc 8000 bước/s² (đạt tốc độ tối đa sau 0.6 s).
// 15/11/2026: J2, J3 chậm lại còn 3200 bước/s (2 vòng/s trục động cơ → J2 ≈ 6.7°/s, J3 ≈ 8.9°/s) và gia tốc 4000:
//   động cơ bước mạnh nhất ở tốc độ thấp, khởi động êm hơn nên ít mất bước khi nâng tải nặng.
const uint32_t JOINT_MAX_ACCEL[6] = {8000, 4000, 4000, 800, 1000, 1800};
const uint32_t JOINT_MAX_SPEED[6] = {4800, 3200, 3200, 2000, 2200, 3000};
// 16/11/2026 — J2, J3 đặt theo ĐỘ (°/s, °/s²) thay vì xung, đổi ra xung theo tỉ số đang dùng (kể cả sau $K):
//   J2 30 °/s = 14400 xung/s = 540 v/ph trục động cơ; J3 40 °/s = 14400 xung/s = 540 v/ph (cùng tốc độ động cơ).
//   Ở 24 V, 1.77 A, 42STH38 (1.65 Ω, 3.2 mH) bắt đầu không đẩy đủ dòng từ ≈ 480 v/ph: mô-men còn ≈ 85 % ở 540 v/ph,
//   ≈ 70 % ở 630 v/ph, ≈ 60 % ở 720 v/ph (ước tính lý tưởng, thực tế thấp hơn) → J2 dừng ở 30 °/s; J3 tỉ số nhỏ hơn
//   nên lên 40 °/s cùng tốc độ động cơ. Trần bộ phát xung 18000 xung/s (J2 37.5 °/s, J3 50 °/s).
//   Gia tốc 60 °/s² (đạt 30 °/s sau 0.5 s): mô-men quán tính ≈ 0.4 N·m ở J2, 0.15 N·m ở J3 (≈ 5 % mô-men trọng lực).
//   0 = dùng giá trị xung ở JOINT_MAX_SPEED / JOINT_MAX_ACCEL.
// 17/11/2026: J4 nối thẳng → đặt theo độ: 45 °/s (≈ bằng tốc độ khớp cũ 43 °/s; = 400 xung/s = 7.5 v/ph động cơ,
//   vùng mô-men cao nhất), gia tốc 90 °/s². Nếu giữ xung 2000/s như cũ, khớp sẽ quay 225 °/s — quá nhanh.
// 19/11/2026: J4 động cơ cũ + PG5 5.18 — TĂNG TỐC: 43 °/s (2000 xung/s cũ) → 90 °/s, gia tốc 17 → 180 °/s².
//   90 °/s × 46.06 xung/° = 4146 xung/s (16 vi bước) = 259 bước đủ/s = 78 v/ph trục động cơ.
//   Ước tính 17HS13-0404S (27 Ω, L ≈ 40 mH — chưa có datasheet đo L) ở 24 V: điện áp cần ≈ R·I 15.4 V ⊕ ωL·I 9.3 V
//   + sức phản điện ≈ 5 V ≈ 23 V → vẫn đẩy gần đủ dòng ở 78 v/ph; nhanh hơn nữa mô-men tụt nhanh (≈ 60 % ở 120 v/ph).
//   Mô-men chạy khớp ≈ 0.82 N·m, cần tối đa 0.39 N·m (J5 gập 90°, tải 200 g) → còn dư ở 90 °/s.
//   Nếu J4 kêu/mất bước khi mang tải: hạ JOINT_SPEED_DEG[3] về 70.
// 21/11/2026: J2 bảo vệ đai — 30 → 20 °/s, gia tốc 60 → 20 °/s² (đạt tốc độ sau 1 s). Lực quán tính khi khởi động,
//   dừng, đổi chiều giảm 3 lần; cụm J3–J6 duỗi thẳng có mô-men quán tính lớn nhất nên đây là lúc đai bị giật mạnh nhất.
// 24/11/2026: J5 đặt theo độ: 45 °/s, gia tốc 90 °/s² (trước 2200 xung/s ≈ 99 °/s, 1000 xung/s² ≈ 45 °/s²). Chậm hơn →
//   động cơ quay 0.31 v/s, mô-men gần bằng mô-men giữ; gia tốc vừa đủ để không giật khi nhấc đầu hút.
const float JOINT_SPEED_DEG[6] = {0, 20.0, 40.0, 90.0, 45.0, 0};
const float JOINT_ACCEL_DEG[6] = {0, 20.0, 60.0, 180.0, 90.0, 0};
uint32_t joint_max_speed[6];   // xung/s đang dùng
uint32_t joint_max_accel[6];   // xung/s² đang dùng

void updateJointLimits(int i) {
  const float cap = 18000.0f;  // = max_rate của ProfileTask (0.45 × STEP_TICK_HZ)
  float spd = fabsf(steps_per_degree[i]);
  float v = JOINT_SPEED_DEG[i] > 0 ? JOINT_SPEED_DEG[i] * spd : (float)JOINT_MAX_SPEED[i];
  float a = JOINT_ACCEL_DEG[i] > 0 ? JOINT_ACCEL_DEG[i] * spd : (float)JOINT_MAX_ACCEL[i];
  joint_max_speed[i] = (uint32_t)(v < cap ? v : cap);
  joint_max_accel[i] = (uint32_t)a;
}
int32_t set_velocity = 2000;
int32_t set_accel = 500;

// ================= TRẠNG THÁI & BỘ NHỚ FLASH =================
enum RobotMode { MODE_PC, MODE_AUTO };
enum RobotState { IDLE, RUNNING, DELAY_WAIT };

RobotMode current_mode = MODE_PC;
RobotState current_state = IDLE;

Preferences preferences;
#define MAX_WAYPOINTS 20

struct Waypoint {
  float angles[6];
  uint8_t tool_state;
};

Waypoint sequence[MAX_WAYPOINTS];
int num_waypoints = 0;
int auto_index = 0;
uint32_t wait_timer = 0;

float target_angles[6] = {0, 0, 0, 0, 0, 0};
int32_t active_target_steps[6] = {0, 0, 0, 0, 0, 0};
uint8_t target_tool = 0;
bool new_command = false;
bool is_home_calibrated = false;
uint32_t last_feedback_time = 0;
IPAddress remoteIP;
uint16_t remotePort;

void saveZeroCalibrationToFlash();

// ================= BỘ PHÁT XUNG STEP BẰNG TIMER (THAY FastAccelStepper, 03/11/2026) =================
// FastAccelStepper 2.0.0 trên ESP32-S3 + IDF 5.5 cho kết quả sai khi chạy thật: J1, J3 (MCPWM/PCNT)
// không phát xung, J6 (RMT) chạy mãi sau lệnh STOP. Firmware thử bit-bang cùng chân thì J2..J5 chạy đúng
// → lỗi ở thư viện, không ở dây/driver. Bộ phát dưới đây chỉ dùng 1 hardware timer + ghi thanh ghi GPIO:
//  - ISR STEP_TICK_HZ (40 kHz, 25 µs): mỗi trục cộng dồn pha (DDA, Q16); tràn pha → 1 xung STEP
//    (mức cao 1 tick = 25 µs ≫ 100 ns tối thiểu của TMC2209). Tần số xung tối đa 20 kHz.
//  - ProfileTask 1 kHz: hình thang vận tốc v ← v + clamp(v_des − v, ±a·dt),
//    v_des = sign(d)·min(vmax, √(2·a·|d|)), d = đích − vị trí (bước). Đổi chiều chỉ khi v = 0 và ISR đã
//    ngừng phát ít nhất 1 chu kỳ (1 ms) → chân DIR luôn ổn định trước xung kế tiếp.
// Giao diện giữ tên hàm như FastAccelStepper để phần còn lại của firmware không phải đổi.
#define STEP_TICK_HZ 40000

class SoftStepper {
 public:
  // --- dùng chung với ISR (chỉ ghi 32-bit, nguyên tử trên Xtensa) ---
  volatile int32_t pos = 0;          // vị trí hiện tại (bước), ISR cập nhật khi phát xung
  volatile uint32_t rate_q16 = 0;    // pha cộng mỗi tick (bước/tick × 65536)
  volatile uint32_t phase = 0;
  volatile int8_t dir = 1;           // chiều đang đặt trên chân DIR (+1 = HIGH = đếm lên)
  volatile bool step_high = false;
  uint32_t step_mask = 0;            // bit của chân STEP trong thanh ghi OUT (pin < 32) hoặc OUT1 (pin ≥ 32)
  bool step_bank1 = false;
  uint8_t dir_pin = 0;
  // --- quy hoạch (ProfileTask, khóa bởi stepMux) ---
  volatile int32_t target = 0;
  volatile float v = 0;              // vận tốc có dấu (bước/s)
  volatile float vmax = 1000;
  volatile float accel = 500;

  void begin(uint8_t step_pin, uint8_t dir_pin_) {
    dir_pin = dir_pin_;
    step_bank1 = step_pin >= 32;
    step_mask = 1UL << (step_pin & 31);
    digitalWrite(step_pin, LOW);
    digitalWrite(dir_pin, HIGH);
    dir = 1;
  }
  int32_t getCurrentPosition() const { return pos; }
  bool isRunning() const { return v != 0.0f || rate_q16 != 0 || pos != target; }
  void setSpeedInHz(uint32_t hz) { vmax = (float)hz; }
  void setAcceleration(uint32_t a) { accel = (float)a; }
  void moveTo(int32_t t) { target = t; }
  void forceStopAndNewPosition(int32_t p);
  void setCurrentPosition(int32_t p) { forceStopAndNewPosition(p); }
};

SoftStepper stepperObjs[6];
SoftStepper *steppers[6];
portMUX_TYPE stepMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t *stepTimer = nullptr;

void SoftStepper::forceStopAndNewPosition(int32_t p) {
  portENTER_CRITICAL(&stepMux);
  rate_q16 = 0;
  v = 0;
  target = pos;      // ProfileTask không tăng tốc lại trong lúc chờ
  portEXIT_CRITICAL(&stepMux);
  delayMicroseconds(80);  // > 3 tick ISR: xung đang dở (nếu có) kết thúc, pos không còn đổi
  portENTER_CRITICAL(&stepMux);
  pos = p;
  target = p;
  phase = 0;
  v = 0;
  rate_q16 = 0;
  portEXIT_CRITICAL(&stepMux);
}

void IRAM_ATTR onStepTimer() {
  uint32_t set0 = 0, set1 = 0, clr0 = 0, clr1 = 0;
  for (int i = 0; i < 6; i++) {
    SoftStepper &s = stepperObjs[i];
    if (s.step_high) {  // kết thúc xung của tick trước
      s.step_high = false;
      if (s.step_bank1) clr1 |= s.step_mask; else clr0 |= s.step_mask;
      continue;
    }
    uint32_t r = s.rate_q16;
    if (r == 0) continue;
    uint32_t ph = s.phase + r;
    if (ph >= 65536UL) {
      ph -= 65536UL;
      s.pos += s.dir;
      s.step_high = true;
      if (s.step_bank1) set1 |= s.step_mask; else set0 |= s.step_mask;
    }
    s.phase = ph;
  }
  if (clr0) REG_WRITE(GPIO_OUT_W1TC_REG, clr0);
  if (clr1) REG_WRITE(GPIO_OUT1_W1TC_REG, clr1);
  if (set0) REG_WRITE(GPIO_OUT_W1TS_REG, set0);
  if (set1) REG_WRITE(GPIO_OUT1_W1TS_REG, set1);
}

void ProfileTask(void *pvParameters) {
  const float dt = 0.001f;
  const float max_rate = 0.45f * STEP_TICK_HZ;  // giới hạn cứng < 20 kHz (2 tick/xung)
  TickType_t last_wake = xTaskGetTickCount();
  while (true) {
    for (int i = 0; i < 6; i++) {
      SoftStepper &s = stepperObjs[i];
      portENTER_CRITICAL(&stepMux);
      int32_t d = s.target - s.pos;
      float v = s.v;
      float a = s.accel;
      float dv = a * dt;
      float v_des = 0;
      if (d != 0) {
        float v_brake = sqrtf(2.0f * a * (float)abs(d));
        float vm = s.vmax < max_rate ? s.vmax : max_rate;
        v_des = (d > 0 ? 1.0f : -1.0f) * (v_brake < vm ? v_brake : vm);
      }
      if (v_des > v + dv) v += dv;
      else if (v_des < v - dv) v -= dv;
      else v = v_des;
      if (d == 0 && fabsf(v) <= 2.0f * dv) v = 0;  // tới đích: dừng hẳn
      int8_t want_dir = v > 0 ? 1 : (v < 0 ? -1 : s.dir);
      if (want_dir != s.dir) {
        if (s.rate_q16 != 0) {
          v = 0;  // chu kỳ này chỉ dừng phát xung; đổi DIR ở chu kỳ sau
        } else {
          s.dir = want_dir;
          gpio_set_level((gpio_num_t)s.dir_pin, want_dir > 0 ? 1 : 0);
          v = 0;  // giữ 1 ms sau khi đổi DIR rồi mới phát xung
        }
        s.rate_q16 = 0;
      } else {
        s.rate_q16 = (uint32_t)(fabsf(v) * (65536.0f / STEP_TICK_HZ));
      }
      s.v = v;
      portEXIT_CRITICAL(&stepMux);
    }
    vTaskDelayUntil(&last_wake, 1);  // tick FreeRTOS = 1 ms
  }
}

TMC2209Stepper driver0(&Serial1, 0.11f, 0);
TMC2209Stepper driver1(&Serial1, 0.11f, 1);
TMC2209Stepper driver2(&Serial1, 0.11f, 2);
TMC2209Stepper driver3(&Serial2, 0.11f, 0);
TMC2209Stepper driver4(&Serial2, 0.11f, 1);
TMC2209Stepper driver5(&Serial2, 0.11f, 2);
TMC2209Stepper* drivers[6] = {&driver0, &driver1, &driver2, &driver3, &driver4, &driver5};

void MotorTask(void *pvParameters);
void loadFromFlash();
void saveToFlash();
void executeTool(uint8_t state);

bool ethernet_available = false;

// ================= DÒ TỰ ĐỘNG BUS UART TMC2209 (CHẨN ĐOÁN, CHỈ ĐỌC) =================
// TMC2209 dùng UART một dây: chân TX của ESP32 phải đi QUA điện trở 1 kΩ tới PDN_UART, chân RX nối
// THẲNG vào PDN_UART. Nếu mạch hàn ngược quy ước (TX/RX đảo) thì driver không trả lời.
// Hàm này thử cả 2 chiều chân và cả 4 địa chỉ (MS1/MS2), đọc thanh ghi IOIN:
// byte cao = VERSION, TMC2209 trả 0x21. Không ghi thanh ghi nào, động cơ không quay.
uint8_t probeUartBus(HardwareSerial& port, int8_t rx, int8_t tx, const char* name) {
  port.end();
  port.begin(115200, SERIAL_8N1, rx, tx);
  // Dòng ra TX thấp nhất (~5 mA): khi thử chiều đảo, TX nối thẳng PDN_UART (không qua 1 kΩ),
  // tránh tranh chấp mạnh với đầu ra của driver lúc nó trả lời.
  gpio_set_drive_capability((gpio_num_t)tx, GPIO_DRIVE_CAP_0);
  delay(30);
  uint8_t found_mask = 0;
  for (uint8_t addr = 0; addr < 4; addr++) {
    TMC2209Stepper probe(&port, 0.11f, addr);
    uint32_t ioin = probe.IOIN();
    uint8_t version = ioin >> 24;
    if (version == 0x21) {
      found_mask |= (1 << addr);
      // IOIN bit0 ENN, bit2 MS1, bit3 MS2 (mức logic thật trên chân của driver)
      Serial.printf("[DIAG] %s RX=GPIO%d TX=GPIO%d: addr%d TRA LOI (TMC2209 v0x%02X, EN=%d MS1=%d MS2=%d)\n",
                    name, rx, tx, addr, version, (int)(ioin & 1), (int)((ioin >> 2) & 1), (int)((ioin >> 3) & 1));
    }
  }
  if (!found_mask) {
    Serial.printf("[DIAG] %s RX=GPIO%d TX=GPIO%d: khong driver nao tra loi (addr 0-3)\n", name, rx, tx);
  }
  return found_mask;
}

// Chọn chiều chân hoạt động cho mỗi bus; giữ cấu hình gốc nếu không chiều nào trả lời.
void setupTmcUartBus(HardwareSerial& port, int8_t rx, int8_t tx, const char* name) {
  uint8_t normal = probeUartBus(port, rx, tx, name);
  if (normal == 0x07) {  // đủ địa chỉ 0,1,2 như firmware giả định
    Serial.printf("[DIAG] %s: OK — dung RX=GPIO%d TX=GPIO%d\n", name, rx, tx);
    return;
  }
  uint8_t swapped = probeUartBus(port, tx, rx, name);
  if (swapped != 0 && swapped >= normal) {
    Serial.printf("[DIAG] %s: CHAN TX/RX BI DAO tren mach — tam dung RX=GPIO%d TX=GPIO%d. "
                  "Nen sua mach hoac doi RX/TX trong code.\n", name, tx, rx);
    if ((swapped & 0x07) != 0x07) {
      Serial.printf("[DIAG] %s: dia chi tra loi = 0x%X (can 0x7 = addr0,1,2) -> kiem tra MS1/MS2\n", name, swapped);
    }
    return;  // cổng đang mở theo chiều đảo
  }
  port.end();
  port.begin(115200, SERIAL_8N1, rx, tx);
  if (normal == 0) {
    Serial.printf("[DIAG] %s: KHONG CO PHAN HOI o ca 2 chieu chan. Kiem tra: nguon dong co VM (12-24V) "
                  "da BAT truoc khi ESP32 khoi dong? dien tro 1k TX->PDN_UART? chung GND? PDN_UART cua module "
                  "da noi ra chan (mot so module can han cau)?\n", name);
  } else {
    Serial.printf("[DIAG] %s: dia chi tra loi = 0x%X (can 0x7 = addr0,1,2) -> kiem tra MS1/MS2\n", name, normal);
  }
}

// Ghi toàn bộ cấu hình cho driver i (gọi lúc khởi động và khi driver bị reset do mất/sụt nguồn VM).
void configureDriver(int i) {
  drivers[i]->toff(4);
  drivers[i]->blank_time(24);
  // GCONF.i_scale_analog mặc định = 1: dòng bị nhân thêm (VREF/2.5 V) theo BIẾN TRỞ trên module → dòng thật
  // thấp hơn giá trị đặt qua UART (biến trở ở giữa ≈ một nửa). Tắt đi để dòng chỉ do firmware quyết định.
  drivers[i]->I_scale_analog(false);
  drivers[i]->rms_current(motor_current[i], HOLD_MULT[i]);
  drivers[i]->iholddelay(4);   // hạ dòng từ chạy → giữ từ từ (tránh giật khi vừa dừng)
  drivers[i]->TPOWERDOWN(40);   // chờ ≈ 0.9 s sau khi dừng mới hạ về dòng giữ
  drivers[i]->microsteps(MICROSTEPS_J[i]);
  drivers[i]->mstep_reg_select(true);
  drivers[i]->internal_Rsense(false);
  drivers[i]->pdn_disable(true);
  drivers[i]->en_spreadCycle(true);
  drivers[i]->TPWMTHRS(0);
}

// ================= GIÁM SÁT DRIVER TMC2209 (03/11/2026) =================
// DRV_STATUS (TMC2209 datasheet §5.5.3): bit0 otpw, bit1 ot, bit2 s2ga, bit3 s2gb, bit4 s2vsa,
// bit5 s2vsb, bit6 ola, bit7 olb, bits16..20 CS_ACTUAL, bit31 stst. GSTAT: bit0 reset, bit1 drv_err, bit2 uv_cp.
// - ola/olb (hở tải cuộn A/B): đứt dây, lỏng jack, hoặc ĐẤU NHẦM CẶP CUỘN (mỗi đầu ra driver nối vào
//   2 cuộn khác nhau). Chỉ tin cậy ở spreadCycle và tốc độ thấp.
// - s2ga/s2gb/s2vsa/s2vsb: chạm mass/chạm nguồn → driver tự ngắt (drv_err), động cơ mất lực.
// - GSTAT.reset/uv_cp: driver vừa khởi động lại vì mất/sụt VM → cấu hình UART mất, vi bước quay về
//   theo MS1/MS2 (8/32/64) → khớp quay sai góc. Bộ giám sát tự ghi lại cấu hình.
const uint32_t DRV_ERR_MASK = 0xFF;
uint32_t drv_flags_last[6] = {0, 0, 0, 0, 0, 0};
bool drv_online[6] = {true, true, true, true, true, true};
uint8_t health_idx = 0;
uint32_t last_health_time = 0;

void printDriverFlags(int i, uint32_t ds) {
  Serial.printf("[DRV] J%d DRV_STATUS=0x%08lX cs=%lu%s%s%s%s%s%s%s%s%s\n", i + 1, (unsigned long)ds,
                (unsigned long)((ds >> 16) & 0x1F), (ds & (1UL << 31)) ? " dung_yen" : "",
                (ds & 0x01) ? " CANH_BAO_NHIET" : "", (ds & 0x02) ? " QUA_NHIET" : "",
                (ds & 0x04) ? " CHAM_MASS_A" : "", (ds & 0x08) ? " CHAM_MASS_B" : "",
                (ds & 0x30) ? " CHAM_NGUON" : "", (ds & 0x40) ? " HO_CUON_A" : "",
                (ds & 0x80) ? " HO_CUON_B" : "", (ds & DRV_ERR_MASK) ? "" : " OK");
}

// Kiểm tra 1 driver (gọi xoay vòng, ~4 lần đọc UART ≈ 5 ms). verbose = in kể cả khi không đổi.
void checkDriver(int i, bool verbose) {
  if (drivers[i]->test_connection() != 0) {
    if (drv_online[i] || verbose) {
      Serial.printf("[DRV] J%d KHONG TRA LOI UART (mat nguon dong luc VM?)\n", i + 1);
    }
    drv_online[i] = false;
    return;
  }
  uint8_t gstat = drivers[i]->GSTAT();
  if (!drv_online[i] || (gstat & 0b101) || drivers[i]->microsteps() != MICROSTEPS_J[i]) {
    configureDriver(i);
    Serial.printf("[DRV] J%d driver vua khoi dong lai/mat cau hinh (GSTAT=0x%X) -> DA CAU HINH LAI "
                  "(vi tri co the da lech: nen ve Home/Set Zero)\n", i + 1, gstat);
  }
  if (gstat) drivers[i]->GSTAT(0b111);
  drv_online[i] = true;
  uint32_t ds = drivers[i]->DRV_STATUS();
  uint32_t flags = ds & DRV_ERR_MASK;
  if (verbose || flags != drv_flags_last[i]) printDriverFlags(i, ds);
  drv_flags_last[i] = flags;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  setupTmcUartBus(Serial1, RX1_PIN, TX1_PIN, "UART1(J1-J3)");
  setupTmcUartBus(Serial2, RX2_PIN, TX2_PIN, "UART2(J4-J6)");

  // Khởi tạo Relay (nếu được kích hoạt)
#if (PUMP_PIN >= 0) && (VALVE_PIN >= 0)
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);
  executeTool(TOOL_OFF);
#endif

  // Kích hoạt toàn bộ 6 driver TMC2209 cố định mức LOW (Active LOW)
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

#if USE_W5500
  // Khởi tạo SPI & Ethernet W5500 (Kiểm tra an toàn nếu có cắm module phần cứng)
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, W5500_CS_PIN);
  Ethernet.init(W5500_CS_PIN);
  // SỬA LỖI 22/10: phải gọi Ethernet.begin() TRƯỚC. Thư viện Ethernet chỉ nhận diện chip trong
  // W5100.init() (gọi bên trong begin); hỏi hardwareStatus() trước đó luôn trả EthernetNoHardware
  // → bản trước tắt Ethernet vĩnh viễn, mọi lệnh LAN/UDP bị bỏ qua (firmware gốc gọi begin vô điều kiện).
  Ethernet.begin(mac, ip);
  if (Ethernet.hardwareStatus() != EthernetNoHardware) {
    Udp.begin(localPort);
    ethernet_available = true;
    Serial.print("W5500 Industrial Ethernet IP: ");
    Serial.println(Ethernet.localIP());
  } else {
    Serial.println("Chay che do USB Serial Type-C thuan (Khong phat hien W5500 SPI).");
  }
#else
  Serial.println("Ethernet W5500 TAT (GPIO10/11 dung cho bom/van L298N). Dieu khien qua USB Serial.");
#endif

  loadFromFlash();

  for (int i = 0; i < 6; i++) {
    steps_per_degree[i] = (STEPS_PER_REV * MICROSTEPS_J[i] * gear_ratio[i] * dir_sign[i]) / 360.0;
    updateJointLimits(i);

    drivers[i]->begin();
    configureDriver(i);
    drivers[i]->GSTAT(0b111);  // xóa cờ reset/drv_err/uv_cp của lần bật nguồn

    // CẤU HÌNH CÔNG SUẤT GPIO ESP32-S3 (KHẮC PHỤC "XUẤT XUNG KHÁ NHỎ"):
    // Nâng drive strength lên mức tối đa GPIO_DRIVE_CAP_3 (~40mA) trên cả 6 chân STEP và DIR.
    // Giúp triệt tiêu hoàn toàn hiện tượng suy hao sụt áp xung do tụ ký sinh và tải của đèn LED WS2812 trên GPIO 38.
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    gpio_set_drive_capability((gpio_num_t)STEP_PINS[i], GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)DIR_PINS[i], GPIO_DRIVE_CAP_3);

    // CHẨN ĐOÁN UART TMC2209: 0 = OK, 1 = không trả lời (đọc 0xFFFFFFFF), 2 = đọc toàn 0.
    // Khi UART lỗi, driver chạy độc lập: vi bước theo chân MS1/MS2 (= chân địa chỉ) → 8/32/64,
    // dòng theo biến trở VREF → khớp quay sai tỉ lệ hoặc không đủ lực.
    uint8_t tmc_status = drivers[i]->test_connection();
    Serial.printf("[DIAG] J%d TMC2209 UART%d addr%d: %s, microsteps doc lai=%u, dong chay=%umA, dong giu=%.0f%%, VREF bien tro=%s\n",
                  i + 1, (i < 3) ? 1 : 2, i % 3,
                  tmc_status == 0 ? "OK" : (tmc_status == 1 ? "KHONG TRA LOI" : "DOC TOAN 0"),
                  drivers[i]->microsteps(), drivers[i]->rms_current(), HOLD_MULT[i] * 100.0f,
                  drivers[i]->I_scale_analog() ? "CON DUNG (dong bi chia!)" : "da tat");

    // DIR HIGH = đếm lên (như setDirectionPin(pin, true) của FastAccelStepper trước đây)
    steppers[i] = &stepperObjs[i];
    steppers[i]->begin(STEP_PINS[i], DIR_PINS[i]);
    steppers[i]->setSpeedInHz(joint_max_speed[i]);
    steppers[i]->setAcceleration(joint_max_accel[i]);
    Serial.printf("[DIAG] J%d gioi han: %lu xung/s = %.1f do/s, %lu xung/s2 = %.0f do/s2\n", i + 1,
                  (unsigned long)joint_max_speed[i], joint_max_speed[i] / fabsf(steps_per_degree[i]),
                  (unsigned long)joint_max_accel[i], joint_max_accel[i] / fabsf(steps_per_degree[i]));
  }

  // Timer phần cứng 1 MHz, ngắt mỗi 25 µs (STEP_TICK_HZ) — API ESP32 Arduino core 3.x
  stepTimer = timerBegin(1000000);
  timerAttachInterrupt(stepTimer, &onStepTimer);
  timerAlarm(stepTimer, 1000000 / STEP_TICK_HZ, true, 0);
  xTaskCreatePinnedToCore(ProfileTask, "ProfileTask", 4096, NULL, 5, NULL, 0);
  Serial.printf("[DIAG] Bo phat xung: timer %d Hz + ProfileTask 1 kHz (khong dung FastAccelStepper)\n", STEP_TICK_HZ);

  xTaskCreatePinnedToCore(MotorTask, "MotorTask", 4096, NULL, 3, NULL, 0);
  Serial.println("System Ready. Dual Mode Serial/UDP Controller Active.");
}

void executeTool(uint8_t state) {
  if (state > TOOL_RELEASE) return;
  if (state == last_cmd_tool) return;      // gói MOVE_PC mang lại cùng trạng thái dụng cụ → không làm gì
  last_cmd_tool = state;
#if (PUMP_PIN >= 0) && (VALVE_PIN >= 0)
  if (state == TOOL_SUCK) {
    digitalWrite(VALVE_PIN, VALVE_OPEN_WHEN_SUCK ? OUT_ON : OUT_OFF);
    delay(20);
    digitalWrite(PUMP_PIN, OUT_ON);
  } else if (state == TOOL_RELEASE) {
    digitalWrite(PUMP_PIN, OUT_OFF);
    delay(20);
    digitalWrite(VALVE_PIN, OUT_ON);       // xả chân không ra khí trời
    release_t0 = millis();
  } else {
    digitalWrite(PUMP_PIN, OUT_OFF);
    digitalWrite(VALVE_PIN, OUT_OFF);
  }
#endif
  current_tool_state = state;
}

// Gọi trong loop(): hết VALVE_RELEASE_MS thì đóng van, báo trạng thái TẮT
void serviceToolTimer() {
  if (current_tool_state == TOOL_RELEASE && millis() - release_t0 >= VALVE_RELEASE_MS) {
#if (PUMP_PIN >= 0) && (VALVE_PIN >= 0)
    digitalWrite(PUMP_PIN, OUT_OFF);
    digitalWrite(VALVE_PIN, OUT_OFF);
#endif
    current_tool_state = TOOL_OFF;
  }
}

// ================= HIỆU CHỈNH TỈ SỐ TRUYỀN / CHIỀU QUAY KHI ĐANG CHẠY (15/11/2026) =================
void printCal() {
  Serial.printf("[CAL] ratio=%.4f,%.4f,%.4f,%.4f,%.4f,%.4f sign=%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\n",
                gear_ratio[0], gear_ratio[1], gear_ratio[2], gear_ratio[3], gear_ratio[4], gear_ratio[5],
                dir_sign[0], dir_sign[1], dir_sign[2], dir_sign[3], dir_sign[4], dir_sign[5]);
  Serial.printf("[CAL] dong_mA=%u,%u,%u,%u,%u,%u (J2 luc dai toi da ~%.0f N)\n", motor_current[0], motor_current[1],
                motor_current[2], motor_current[3], motor_current[4], motor_current[5], 0.72f * motor_current[1]);
  if (j2_locked) Serial.printf("[CAL] J2 KHOA o %.2f do (dong co giu cung)\n", j2_lock_deg);
  else Serial.println("[CAL] J2 khong khoa");
}

// Đổi tỉ số/chiều của khớp i: dừng khớp, giữ nguyên GÓC đang báo (đổi lại số bước tương ứng).
void applyCal(int i, float ratio, float sign) {
  float angle = (fabsf(steps_per_degree[i]) > 1e-6f) ? (float)steppers[i]->getCurrentPosition() / steps_per_degree[i] : 0.0f;
  gear_ratio[i] = ratio;
  dir_sign[i] = sign;
  steps_per_degree[i] = (STEPS_PER_REV * MICROSTEPS_J[i] * gear_ratio[i] * dir_sign[i]) / 360.0;
  updateJointLimits(i);
  int32_t p = (int32_t)lroundf(angle * steps_per_degree[i]);
  steppers[i]->forceStopAndNewPosition(p);
  active_target_steps[i] = p;
  target_angles[i] = angle;
}

// Khóa J2: đích J2 = góc khóa (chỉ phát xung nếu J2 chưa ở đó). Các khớp khác giữ đích hiện tại.
void applyJ2Lock() {
  target_angles[1] = j2_lock_deg;
  new_command = true;   // MotorTask gọi moveSynchronized → J2 đi tới góc khóa với tốc độ/gia tốc J2
}

void handleCalLine(const char *line) {
  int j = 0;
  float val = 0;
  char c = line[0];
  if (c == 'J' && J2_HARD_LOCK) {
    Serial.printf("[CAL] J2 KHOA CUNG trong firmware o %.1f do (J2_HARD_LOCK) - $J bi bo qua\n", J2_HARD_LOCK_DEG);
    return;
  } else if (c == 'J') {
    // $J <độ> khóa J2 tại góc đó | $J khóa tại góc hiện tại | $J U mở khóa
    const char *arg = line + 1;
    while (*arg == ' ') arg++;
    if (*arg == 'U' || *arg == 'u') {
      j2_locked = false;
      Serial.println("[CAL] J2 DA MO KHOA: J2 nhan lenh tu PC tro lai. Go $S de luu.");
    } else if (*arg == 0 || (sscanf(arg, "%f", &val) == 1 && val >= -90.0f && val <= 110.0f)) {
      if (*arg == 0) val = (float)steppers[1]->getCurrentPosition() / steps_per_degree[1];
      j2_locked = true;
      j2_lock_deg = val;
      applyJ2Lock();
      Serial.printf("[CAL] J2 DA KHOA o %.2f do: dong co giu cung, bo qua moi lenh J2. Go $S de luu.\n", val);
    } else {
      Serial.println("[CAL] LOI: $J <do -90..110> | $J | $J U");
      return;
    }
  } else if (c == 'P') {
  } else if ((c == 'K' || c == 'D') && sscanf(line + 1, "%d %f", &j, &val) == 2 && j >= 1 && j <= 6) {
    if (c == 'K' && val >= 0.2f && val <= 500.0f) applyCal(j - 1, val, dir_sign[j - 1]);
    else if (c == 'D' && (val == 1.0f || val == -1.0f)) applyCal(j - 1, gear_ratio[j - 1], val);
    else { Serial.println("[CAL] LOI: gia tri ngoai khoang"); return; }
  } else if (c == 'I' && sscanf(line + 1, "%d %f", &j, &val) == 2 && j >= 1 && j <= 6) {
    // $I <khớp> <mA>: đổi dòng chạy ngay (dòng giữ theo HOLD_MULT), áp dụng tức thì qua UART
    if (val < CURRENT_MIN_MA || val > CURRENT_MAX_MA) {
      Serial.printf("[CAL] LOI: dong %u..%u mA\n", CURRENT_MIN_MA, CURRENT_MAX_MA);
      return;
    }
    if (j == 2 && val > J2_CURRENT_MAX_MA) {
      Serial.printf("[CAL] LOI: J2 toi da %u mA (bao ve dai 240-2GT-6, luc dai ~%.0f N)\n", J2_CURRENT_MAX_MA,
                    0.72f * J2_CURRENT_MAX_MA);
      return;
    }
    motor_current[j - 1] = (uint16_t)val;
    drivers[j - 1]->rms_current(motor_current[j - 1], HOLD_MULT[j - 1]);
  } else if (c == 'S') {
    float cal[12];
    for (int i = 0; i < 6; i++) { cal[i] = gear_ratio[i]; cal[6 + i] = dir_sign[i]; }
    preferences.begin("robot_arm", false);
    preferences.putBytes("cal_v1", cal, sizeof(cal));
    preferences.putBytes("cur_v1", motor_current, sizeof(motor_current));
    preferences.putFloat("j2lock", j2_locked ? j2_lock_deg : NAN);
    preferences.end();
    Serial.println("[CAL] DA LUU vao Flash NVS");
  } else if (c == 'R') {
    preferences.begin("robot_arm", false);
    preferences.remove("cal_v1");
    preferences.remove("cur_v1");
    preferences.remove("j2lock");   // $R cũng mở khóa J2
    preferences.end();
    j2_locked = J2_HARD_LOCK;   // khóa cứng không bị $R gỡ
    j2_lock_deg = J2_HARD_LOCK_DEG;
    for (int i = 0; i < 6; i++) {
      applyCal(i, GEAR_RATIOS[i], DIR_SIGNS[i]);
      motor_current[i] = MOTOR_CURRENTS[i];
      drivers[i]->rms_current(motor_current[i], HOLD_MULT[i]);
    }
    Serial.println("[CAL] Da xoa hieu chinh, ve mac dinh trong firmware");
  } else {
    Serial.println("[CAL] LOI: lenh khong hop le ($P, $K j ratio, $D j sign, $I j mA, $J do|U, $S, $R)");
    return;
  }
  printCal();
}
char cal_buf[32];
uint8_t cal_len = 0;
bool cal_active = false;

// ================= BIẾN NHẬN DỮ LIỆU USB SERIAL STREAM =================
uint8_t serial_rx_buf[sizeof(RobotCommandPacket)];
size_t serial_rx_idx = 0;
// Bộ đếm chẩn đoán (in mỗi 2 s): gói lệnh hợp lệ / sai CRC, lệnh MOVE bị bỏ qua vì đang MODE_AUTO
uint32_t diag_rx_ok = 0, diag_crc_err = 0, diag_move_ignored = 0;
uint32_t last_diag_time = 0;
unsigned long last_serial_rx_time = 0;
unsigned long last_serial_byte_time = 0;

void processRobotCommand(const RobotCommandPacket& rx_packet) {
  if (rx_packet.command_type == CMD_STOP) {
    current_mode = MODE_PC;
    for (int i = 0; i < 6; i++) {
      if (steppers[i]) {
        steppers[i]->forceStopAndNewPosition(steppers[i]->getCurrentPosition());
        active_target_steps[i] = steppers[i]->getCurrentPosition();
      }
    }
    executeTool(TOOL_OFF);
    current_state = IDLE;
    if (rx_packet.tool_state == 255) { // Clear all waypoints
      num_waypoints = 0;
      saveToFlash();
    }
  } 
  else if (rx_packet.command_type == CMD_PLAY_AUTO) {
    if (num_waypoints > 0) {
      current_mode = MODE_AUTO;
      auto_index = 0;
      current_state = IDLE;
    }
  }
  else if (rx_packet.command_type == CMD_TOOL_ONLY) {
    executeTool(rx_packet.tool_state);
  }
  else if (rx_packet.command_type == CMD_REC_WP) {
    if (num_waypoints < MAX_WAYPOINTS) {
      for (int i = 0; i < 6; i++) sequence[num_waypoints].angles[i] = rx_packet.target_angles[i];
      sequence[num_waypoints].tool_state = rx_packet.tool_state;
      num_waypoints++;
      saveToFlash();
    }
  }
  else if (rx_packet.command_type == CMD_MOVE_PC && current_mode == MODE_PC) {
    for (int i = 0; i < 6; i++) target_angles[i] = rx_packet.target_angles[i];
    target_tool = rx_packet.tool_state;
    new_command = true;
  }
  else if (rx_packet.command_type == CMD_MOVE_PC) {
    diag_move_ignored++;  // đang MODE_AUTO: lệnh slider bị bỏ qua cho tới khi nhận CMD_STOP
  }
  else if (rx_packet.command_type == CMD_SET_ZERO) {
    current_mode = MODE_PC;
    for (int i = 0; i < 6; i++) {
      if (steppers[i]) {
        steppers[i]->forceStopAndNewPosition(0);
        steppers[i]->setCurrentPosition(0);
      }
      active_target_steps[i] = 0;
      target_angles[i] = 0.0f;
    }
    new_command = false;
    current_state = IDLE;
    saveZeroCalibrationToFlash(); // Lưu vĩnh viễn vào Flash NVS
    last_feedback_time = 0; // Kích hoạt gửi phản hồi Act: 0.0° ngay lập tức cho GUI
    Serial.println(">> ZERO CALIBRATION OK: Tat ca 6 truc da duoc reset ve goc 0.0 do va luu Flash NVS!");
    // 22/11/2026: tư thế 0 = DỰNG THẲNG ĐỨNG (cánh tay trên + cẳng tay + trục J6 hướng lên trời) — khớp digital twin.
    Serial.println("   (Tu the 0 = DUNG THANG DUNG: canh tay tren, cang tay, truc J6 cung huong len troi.)");
  }
}

RobotFeedbackPacket buildFeedbackPacket() {
  RobotFeedbackPacket packet;
  packet.header = HEADER_FEEDBACK;
  // Bit 7 của robot_mode biểu thị trạng thái đã lưu vị trí gốc trong Flash NVS (0x80: Đã lưu, 0x00: Chưa lưu)
  packet.robot_mode = ((uint8_t)current_mode & 0x7F) | (is_home_calibrated ? 0x80 : 0x00);
  packet.current_tool = current_tool_state;

  for (int i = 0; i < 6; i++) {
    if (steppers[i] && fabs(steps_per_degree[i]) > 0.0001f) {
      packet.actual_angles[i] = (float)(steppers[i]->getCurrentPosition()) / steps_per_degree[i];
    } else {
      packet.actual_angles[i] = 0.0f;
    }
  }

  packet.crc16 = calc_crc16((uint8_t*)&packet, sizeof(RobotFeedbackPacket) - 2);
  return packet;
}

void loop() {
  // --- 1. ĐỌC VÀ GIẢI MÃ DỮ LIỆU TỪ USB SERIAL (TYPE-C) ---
  while (Serial.available() > 0) {
    uint8_t b = Serial.read();
    unsigned long now = millis();

    // Reset buffer nếu ngắt quãng quá 100ms
    if (serial_rx_idx > 0 && (now - last_serial_byte_time > 100)) {
      serial_rx_idx = 0;
    }
    last_serial_byte_time = now;

    if (serial_rx_idx == 0) {
      if (b == 0x55) { // HEADER_COMMAND (0xAA55) byte đầu tiên trong Little Endian là 0x55
        serial_rx_buf[serial_rx_idx++] = b;
        cal_active = false;
      } else if (cal_active) {  // đang nhận dòng lệnh hiệu chỉnh "$..." tới ký tự xuống dòng
        if (b == '\n' || b == '\r') { cal_buf[cal_len] = 0; cal_active = false; handleCalLine(cal_buf); }
        else if (cal_len < sizeof(cal_buf) - 1 && b >= 32 && b < 127) cal_buf[cal_len++] = (char)b;
        else cal_active = false;
      } else if (b == '$') {
        cal_active = true;
        cal_len = 0;
      } else if (b == '?') {  // lệnh chữ chẩn đoán (ngoài khung gói): in trạng thái 6 driver
        for (int i = 0; i < 6; i++) checkDriver(i, true);
      }
    } else if (serial_rx_idx == 1) {
      if (b == 0xAA) { // Byte thứ hai là 0xAA
        serial_rx_buf[serial_rx_idx++] = b;
      } else {
        serial_rx_idx = (b == 0x55) ? 1 : 0;
      }
    } else {
      serial_rx_buf[serial_rx_idx++] = b;
      if (serial_rx_idx >= sizeof(RobotCommandPacket)) {
        RobotCommandPacket* cmd = (RobotCommandPacket*)serial_rx_buf;
        uint16_t calculated_crc = calc_crc16(serial_rx_buf, sizeof(RobotCommandPacket) - 2);
        if (cmd->crc16 == calculated_crc) {
          diag_rx_ok++;
          processRobotCommand(*cmd);
          last_serial_rx_time = millis();
          serial_rx_idx = 0;
        } else {
          diag_crc_err++;
          // Trượt tìm header 0x55, 0xAA tiếp theo trong buffer nếu CRC sai lệch nhịp
          size_t next_head = 0;
          for (size_t k = 1; k < sizeof(RobotCommandPacket) - 1; k++) {
            if (serial_rx_buf[k] == 0x55 && serial_rx_buf[k+1] == 0xAA) {
              next_head = k;
              break;
            }
          }
          if (next_head > 0) {
            size_t remaining = sizeof(RobotCommandPacket) - next_head;
            memmove(serial_rx_buf, &serial_rx_buf[next_head], remaining);
            serial_rx_idx = remaining;
          } else {
            serial_rx_idx = 0;
          }
        }
      }
    }
  }

  // --- 2. ĐỌC VÀ GIẢI MÃ GÓI TIN UDP NHỊ PHÂN TỪ MẠNG LAN W5500 ---
  if (ethernet_available) {
    int packetSize = Udp.parsePacket();
    if (packetSize == sizeof(RobotCommandPacket)) {
      RobotCommandPacket rx_packet;
      Udp.read((char*)&rx_packet, sizeof(RobotCommandPacket));

      uint16_t calculated_crc = calc_crc16((uint8_t*)&rx_packet, sizeof(RobotCommandPacket) - 2);
      if (rx_packet.header == HEADER_COMMAND && calculated_crc == rx_packet.crc16) {
        remoteIP = Udp.remoteIP();
        remotePort = Udp.remotePort();
        processRobotCommand(rx_packet);
      }
    }
  }

  serviceToolTimer();

  // --- 3. GỬI PHẢN HỒI ĐỊNH KỲ (MỖI 33ms ~ 30Hz) CHO CẢ 2 KÊNH ---
  if (millis() - last_feedback_time > 33) {
    RobotFeedbackPacket packet = buildFeedbackPacket();

    // Luôn phát phản hồi qua USB Serial để GUI Python bắt nhịp hiển thị Act tức thì
    Serial.write((uint8_t*)&packet, sizeof(RobotFeedbackPacket));

    // Phản hồi qua UDP nếu máy tính gửi qua Mạng LAN W5500
    if (ethernet_available && remotePort > 0) {
      Udp.beginPacket(remoteIP, remotePort);
      Udp.write((uint8_t*)&packet, sizeof(RobotFeedbackPacket));
      Udp.endPacket();
    }

    last_feedback_time = millis();
  }

  // --- 3b. GIÁM SÁT DRIVER: mỗi 250 ms kiểm tra 1 driver (mỗi driver ~1.5 s/lần) ---
  if (millis() - last_health_time > 250) {
    last_health_time = millis();
    checkDriver(health_idx, false);
    health_idx = (health_idx + 1) % 6;
  }

  // --- 4. DÒNG CHẨN ĐOÁN MỖI 2 GIÂY (chữ; GUI Python tự bỏ qua khi dò header gói nhị phân) ---
  if (millis() - last_diag_time > 2000) {
    last_diag_time = millis();
    Serial.printf("[DIAG] rx_ok=%lu crc_err=%lu move_bo_qua=%lu mode=%s steps=[%ld %ld %ld %ld %ld %ld]\n",
                  (unsigned long)diag_rx_ok, (unsigned long)diag_crc_err, (unsigned long)diag_move_ignored,
                  current_mode == MODE_PC ? "PC" : "AUTO",
                  steppers[0] ? (long)steppers[0]->getCurrentPosition() : -1L,
                  steppers[1] ? (long)steppers[1]->getCurrentPosition() : -1L,
                  steppers[2] ? (long)steppers[2]->getCurrentPosition() : -1L,
                  steppers[3] ? (long)steppers[3]->getCurrentPosition() : -1L,
                  steppers[4] ? (long)steppers[4]->getCurrentPosition() : -1L,
                  steppers[5] ? (long)steppers[5]->getCurrentPosition() : -1L);
  }
  delay(1);
}

// ================= HÀM NỘI SUY ĐỒNG BỘ BƯỚC =================
void moveSynchronized(float target_ang[6]) {
  int32_t target_steps[6];
  int32_t delta_steps[6];
  int32_t max_delta = 0;

  for (int i = 0; i < 6; i++) {
    if (!steppers[i]) continue;
    // J2 khóa: thay góc lệnh bằng góc khóa (lệnh J2 từ PC/AUTO bị bỏ qua)
    float ang = (j2_locked && i == 1) ? j2_lock_deg : target_ang[i];
    if (i == 2) ang = constrain(ang, J3_MIN_DEG, J3_MAX_DEG);   // giới hạn mềm J3 (24/11/2026)
    target_steps[i] = (int32_t)round(ang * steps_per_degree[i]);
    delta_steps[i] = abs(target_steps[i] - steppers[i]->getCurrentPosition());
    
    // Dải chết thích ứng (Adaptive Deadband):
    // Trục tỉ số truyền nhỏ 1:1 (J6, J1) chỉ có ~8.89 bước/độ -> deadband = 1 bước để không bị trơ/khựng
    // Trục tỉ số truyền lớn (J2, J3, J4, J5) -> deadband = 2 bước
    uint32_t deadband = (gear_ratio[i] <= 1.5f) ? 1 : 2;
    if (delta_steps[i] <= deadband) {
      delta_steps[i] = 0;
    }
    if (delta_steps[i] > max_delta) max_delta = delta_steps[i];
  }

  if (max_delta == 0) return;

  // Đồng bộ theo THỜI GIAN (06/11): T = thời gian của khớp chậm nhất ở tốc độ tối đa riêng của nó; mỗi khớp
  // chạy delta_i / T để cùng tới đích. (Bản cũ chia theo tỉ lệ số bước — sai khi bước/độ các khớp chênh nhau
  // 10 lần sau khi J1–J3 đổi hộp số 1:27.)
  float t_sync = 0.0f;
  for (int i = 0; i < 6; i++) {
    if (steppers[i] && delta_steps[i] > 0) {
      float t_i = (float)delta_steps[i] / joint_max_speed[i];
      if (t_i > t_sync) t_sync = t_i;
    }
  }

  for (int i = 0; i < 6; i++) {
    if (!steppers[i]) continue;
    if (delta_steps[i] > 0) {
      // Chỉ lập lại lệnh moveTo khi đích đến thực sự thay đổi hoặc động cơ đang dừng
      if (target_steps[i] != active_target_steps[i] || !steppers[i]->isRunning()) {
        active_target_steps[i] = target_steps[i];

        int32_t sync_speed = (int32_t)((float)delta_steps[i] / t_sync);
        if (sync_speed > (int32_t)joint_max_speed[i]) sync_speed = joint_max_speed[i];

        // Đảm bảo tốc độ tối thiểu ít nhất 150 bước/giây để trục không bị bò rùa
        if (sync_speed < 150) sync_speed = 150;

        steppers[i]->setSpeedInHz(sync_speed);
        // Duy trì gia tốc chuẩn riêng của từng khớp để thắng dứt khoát không bị vọt lố/xoay vô hạn
        steppers[i]->setAcceleration(joint_max_accel[i]);
        steppers[i]->moveTo(target_steps[i]);
      }
    }
  }
}

// ================= FREERTOS MOTOR TASK =================
void MotorTask(void *pvParameters) {
  while (true) {
    if (current_mode == MODE_PC) {
      if (new_command) {
        new_command = false;
        executeTool(target_tool);
        moveSynchronized(target_angles);
      }
    } 
    else if (current_mode == MODE_AUTO) {
      if (current_state == IDLE) {
        moveSynchronized(sequence[auto_index].angles);
        current_state = RUNNING;
      }
      else if (current_state == RUNNING) {
        bool all_stopped = true;
        for (int i = 0; i < 6; i++) {
          if (steppers[i] && steppers[i]->isRunning()) all_stopped = false;
        }
        if (all_stopped) {
          executeTool(sequence[auto_index].tool_state);
          wait_timer = millis();
          current_state = DELAY_WAIT;
        }
      }
      else if (current_state == DELAY_WAIT) {
        if (millis() - wait_timer > 1000) {
          auto_index++;
          if (auto_index >= num_waypoints) auto_index = 0;
          current_state = IDLE;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void saveZeroCalibrationToFlash() {
  preferences.begin("robot_arm", false);
  preferences.putBool("is_calib", true);
  preferences.end();
  is_home_calibrated = true;
  Serial.println(">> FLASH NVS: Da luu vinh vien moc goc 0.0 do! Moi lan bat len, canh tay se tu dong set ve vi tri nay.");
}

void saveToFlash() {
  preferences.begin("robot_arm", false);
  preferences.putInt("num_wp", num_waypoints);
  if (num_waypoints > 0) {
    preferences.putBytes("sequence", &sequence, sizeof(Waypoint) * num_waypoints);
  }
  preferences.end();
}

void loadFromFlash() {
  preferences.begin("robot_arm", false);
  // Hiệu chỉnh tỉ số truyền / chiều quay (nếu đã lưu bằng $S); dữ liệu hỏng hoặc ngoài khoảng → dùng mặc định
  for (int i = 0; i < 6; i++) { gear_ratio[i] = GEAR_RATIOS[i]; dir_sign[i] = DIR_SIGNS[i]; }
  float cal[12];
  if (preferences.getBytes("cal_v1", cal, sizeof(cal)) == sizeof(cal)) {
    bool ok = true;
    for (int i = 0; i < 6; i++) ok = ok && cal[i] >= 0.2f && cal[i] <= 500.0f && (cal[6 + i] == 1.0f || cal[6 + i] == -1.0f);
    if (ok) {
      for (int i = 0; i < 6; i++) { gear_ratio[i] = cal[i]; dir_sign[i] = cal[6 + i]; }
      Serial.println(">> FLASH NVS: Dung ti so truyen / chieu quay DA HIEU CHINH (lenh $P de xem, $R de xoa).");
      // 22/11/2026: chiều lưu trong Flash là chiều cũ (ngược twin ở J1, J3, J4, J5) → đảo 1 lần rồi ghi lại.
      if (!preferences.getBool("dir22", false)) {
        for (int i = 0; i < 6; i++) if (DIR_FLIP_22[i]) dir_sign[i] = -dir_sign[i];
        for (int i = 0; i < 6; i++) cal[6 + i] = dir_sign[i];
        preferences.putBytes("cal_v1", cal, sizeof(cal));
        Serial.println(">> FLASH NVS: Da DAO CHIEU J1, J3, J4, J5 trong hieu chinh da luu (1 lan, 22/11/2026).");
      }
    }
  }
  preferences.putBool("dir22", true);
  // 21/11/2026: dòng chạy đã chỉnh bằng $I + $S. Dòng J2 lưu cũ > mặc định (ví dụ 1770 trước khi đổi) vẫn được
  // dùng — muốn về mặc định bảo vệ đai thì gõ $I 2 1200 rồi $S, hoặc $R.
  for (int i = 0; i < 6; i++) motor_current[i] = MOTOR_CURRENTS[i];
  uint16_t cur[6];
  if (preferences.getBytes("cur_v1", cur, sizeof(cur)) == sizeof(cur)) {
    bool ok = true;
    for (int i = 0; i < 6; i++) ok = ok && cur[i] >= CURRENT_MIN_MA && cur[i] <= CURRENT_MAX_MA;
    if (ok) {
      for (int i = 0; i < 6; i++) motor_current[i] = cur[i];
      Serial.printf(">> FLASH NVS: Dung dong da chinh, J2 = %u mA ($P de xem).\n", motor_current[1]);
    }
  }
  // 23–24/11/2026: J5 lên 1400 mA — dòng cũ lưu Flash thấp hơn (900/1200) không đủ nâng đầu hút → dùng mặc định mới
  if (motor_current[4] < MOTOR_CURRENTS[4]) {
    Serial.printf(">> J5: dong luu %u mA < %u mA -> dung %u mA (nang dau hut). Go $S de luu.\n",
                  motor_current[4], MOTOR_CURRENTS[4], MOTOR_CURRENTS[4]);
    motor_current[4] = MOTOR_CURRENTS[4];
  }
  // 23/11/2026: đai 240 — dòng J2 lưu Flash vượt trần (vd 1200/1770 của bản cũ) → về mặc định bảo vệ đai
  if (motor_current[1] > J2_CURRENT_MAX_MA) {
    Serial.printf(">> J2: dong luu %u mA > tran %u mA (dai 240) -> dung %u mA. Go $S de luu.\n",
                  motor_current[1], J2_CURRENT_MAX_MA, MOTOR_CURRENTS[1]);
    motor_current[1] = MOTOR_CURRENTS[1];
  }
  float lock = preferences.getFloat("j2lock", NAN);
  if (J2_HARD_LOCK) {
    j2_locked = true;
    j2_lock_deg = J2_HARD_LOCK_DEG;
    Serial.printf(">> J2 KHOA CUNG o %.1f do (firmware 24/11/2026). Lenh J2 tu PC bi bo qua.\n", J2_HARD_LOCK_DEG);
  } else if (!isnan(lock) && lock >= -90.0f && lock <= 110.0f) {
    j2_locked = true;
    j2_lock_deg = lock;
    Serial.printf(">> FLASH NVS: J2 KHOA o %.2f do. Lenh dau tien tu PC se dua J2 toi goc nay. $J U de mo khoa.\n", lock);
  }
  // 19/11/2026: J4 về động cơ cũ có hộp số 5.18. Flash có thể còn tỉ số 1.0 của động cơ 17PM (do $K 4 1.0 + $S ngày
  // 17/11) → J4 chỉ quay 1/5 góc lệnh, rất chậm. Tỉ số J4 < 2.0 coi là dữ liệu cũ → dùng mặc định 5.18.
  if (gear_ratio[3] < 2.0f) {
    Serial.printf(">> FLASH NVS: ti so J4 %.3f (dong co 17PM) -> bo qua, dung %.3f (dong co cu + hop so PG5). Go $S de luu.\n",
                  gear_ratio[3], GEAR_RATIOS[3]);
    gear_ratio[3] = GEAR_RATIOS[3];
  }
  num_waypoints = preferences.getInt("num_wp", 0);
  if (num_waypoints > 0 && num_waypoints <= MAX_WAYPOINTS) {
    preferences.getBytes("sequence", &sequence, sizeof(Waypoint) * num_waypoints);
    Serial.printf("Loaded %d waypoints from Flash.\n", num_waypoints);
  } else {
    num_waypoints = 0;
  }
  is_home_calibrated = preferences.getBool("is_calib", false);
  if (is_home_calibrated) {
    Serial.println(">> FLASH NVS: Tim thay moc goc Home da luu! Tu dong kich hoat moc 0.0 do khi khoi dong.");
    Serial.println("   Robot PHAI dang o tu the 0 DUNG THANG DUNG khi bat nguon (firmware coi vi tri luc bat = 0 do).");
  } else {
    Serial.println(">> FLASH NVS: Chua luu vi tri goc Home. Vui long Set Zero lan dau.");
  }
  preferences.end();
}