
#include "Motion.h"
#include "DisplayOLED.h"
#include "Sensor.h"

// Biến đếm số lần gặp vạch ngang/vòng tròn
int crossLineCount = 0;

// Chứa mảng sensor và sensorValue
int servoPwm = 0;
int vitri = 0;
int baseSpeed = 170; // Dải PWM 0-255 (chỉnh theo yêu cầu)
volatile int calPID = 0;
volatile long lastPos = 0;
volatile float Kp = 2;
volatile float Ki = 0;
volatile float Kd = 13;
volatile int isLeft = 0;
volatile int isRight = 0;
volatile long posPID = 0;
int dashedLineCount = 0; // Đếm số vạch đứt

void updateCrossLineOLED() {
  display.clearDisplay();
  display.setTextSize(2); // In chữ to cho dễ nhìn
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("Vach gap:");

  display.setCursor(0, 30);
  display.print(crossLineCount); // Hiển thị số 1, 2, 3...

  display.display();
}

void motion_reset() {
  dashedLineCount = 0;

  vitri = 0;
  lastPos = 0;
  posPID = 0;
  calPID = 0;
}
void motion_init() {
  // Cài đặt các chân IN làm OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Mặc định cho xe đứng im
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Cài đặt băm xung PWM cho chân ENA và ENB (8-bit để dùng 0-255 dễ hơn)
  ledcSetup(ENA_CH, 1000, 8);
  ledcAttachPin(ENA, ENA_CH);

  ledcSetup(ENB_CH, 1000, 8);
  ledcAttachPin(ENB, ENB_CH);

  initOLED();

  speed_run(0, 0);
}

// --------------------------------------------------------
// HÀM ĐIỀU KHIỂN CHIỀU VÀ TỐC ĐỘ CHUẨN L298N
// --------------------------------------------------------
void speed_run(int speedDC_left, int speedDC_right) {
  // 1. Giới hạn dải tốc độ không vượt quá 255
  if (speedDC_left < -255)
    speedDC_left = -255;
  if (speedDC_right < -255)
    speedDC_right = -255;
  if (speedDC_left > 255)
    speedDC_left = 255;
  if (speedDC_right > 255)
    speedDC_right = 255;

  // 2. Điều khiển bánh TRÁI (Motor A)
  if (speedDC_left == 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW); // Phanh động cơ
    ledcWrite(ENA_CH, 0);
  } else if (speedDC_left > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW); // Chạy tới
    ledcWrite(ENA_CH, speedDC_left);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);          // Chạy lùi (khi phanh gắt)
    ledcWrite(ENA_CH, -speedDC_left); // Lấy số dương của tốc độ
  }

  // 3. Điều khiển bánh PHẢI (Motor B)
  if (speedDC_right == 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW); // Phanh động cơ
    ledcWrite(ENB_CH, 0);
  } else if (speedDC_right > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW); // Chạy tới
    ledcWrite(ENB_CH, speedDC_right);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);           // Chạy lùi (khi phanh gắt)
    ledcWrite(ENB_CH, -speedDC_right); // Lấy số dương của tốc độ
  }
}

// ========================================================
// CÁC HÀM BÊN DƯỚI GIỮ NGUYÊN 100% LOGIC CỦA BẠN
// ========================================================

void handleAndSpeed(int angle, int speed1) {
  int speedLeft;
  int speedRight;

  // Điều chỉnh theo dải 0-255, giữ nguyên ý tưởng bảo toàn lực bẻ lái
  // Nếu bánh ngoài sẽ vượt 255, hạ speed1 để bánh ngoài = 255
  if ((speed1 + angle) > 255)
    speed1 = 255 - angle;
  if ((speed1 - angle) > 255)
    speed1 = 255 + angle;

  speedLeft = (speed1 + angle);
  speedRight = (speed1 - angle);

  speed_run(speedLeft, speedRight);
}

void PID(uint8_t current_sensor) {
  if (calPID == 1) {
    // ============================================================
    // Tính vị trí line từ sensor byte nhị phân (ĐEN = 1)
    // Trọng số vị trí: j * 30 -> dải 0 (trái) đến 210 (phải)
    // Tâm line = 105
    // ============================================================
    long weightedSum = 0;
    long activeCount = 0;

    for (int j = 0; j < 8; j++) {
      if (bitRead(current_sensor, 7 - j)) {
        weightedSum += (long)j * 30;
        activeCount++;
      }
    }

    long i;
    if (activeCount == 0) {
      i = lastPos;
    } else {
      long position = weightedSum / activeCount; // 0..210
      i = position - 105;                        // lỗi: -105..+105
    }

    posPID = i;

    // Anti-windup
    static long sumError = 0;
    if (abs(i) < 30)
      sumError += i;
    if (sumError > 5000)
      sumError = 5000;
    if (sumError < -5000)
      sumError = -5000;

    long iP = (long)(Kp * i);
    long iI = (long)(Ki * sumError);
    long iD = (long)(Kd * (lastPos - i));
    long iRet = iP + iI - iD;

    // không cần chia 5 nữa vì error đã vừ tầm (±105 × Kp ≈ ±210)
    servoPwm = (int)iRet;
    lastPos = i;
    calPID = 0;
  }
}

void runforwardline(int tocdo) {
  // ── White line mode (line trắng nền đen) ──────────────────────────
  static bool whiteLineMode = false;
  int blackCount = __builtin_popcount(sensor); // Đếm số mắt ĐEN gốc

  // 1. Điều kiện VÀO vùng line trắng (Nền đen 5-7 mắt, có mắt trắng ở giữa)
  if (!whiteLineMode && blackCount >= 5 && blackCount <= 7) {
    uint8_t whiteMask = ~sensor;
    if (whiteMask & 0b00111100) { // Có mắt trắng ở các kênh 2,3,4,5
      whiteLineMode = true;
    }
  }

  // 2. Điều kiện THOÁT vùng line trắng (Về line đen khi thấy 1-3 mắt đen)
  if (whiteLineMode &&
      (blackCount == 1 || blackCount == 2 || blackCount == 3)) {
    whiteLineMode = false;
  }

  // 3. Xử lý tín hiệu cho PID và Switch Case
  uint8_t processed_sensor = sensor;
  if (whiteLineMode) {
    processed_sensor ^= 0xFF; // Đảo bit để các case 0b00011000... chạy đúng
  }
  // ──────────────────────────────────────────────────────────────────

  PID(processed_sensor);

  // Debounce mất line: reset timer khi còn thấy line
  static bool isLostLine = false;
  static unsigned long lostLineTimer = 0;

  // -- Biến cho nhận diện vạch đứt đích (FINISH) --
  static bool wasOnLine = false;
  static unsigned long lastDashedTime = 0;

  if (processed_sensor != 0) {
    isLostLine = false;
    wasOnLine = true; // Đang chạy trên vạch đen
  } else {
    // Rớt xuống khoảng trắng (Mất line hoàn toàn)
    if (wasOnLine) {                        // Nếu vừa mới từ vạch đen rớt xuống
      if (millis() - lastDashedTime > 60) { // Chống đếm nháy (debounce)
        dashedLineCount++;                  // Cộng 1 vạch đứt
        lastDashedTime = millis();
        // Serial.println(dashedLineCount); // Bật dòng này để xem đếm chuẩn
        // chưa
      }
      wasOnLine = false;
    }
  }

  switch (processed_sensor) {
  case 0b11111111:
  case 0b01111110: // 8 mắt đọc thấy vạch đen ngang hoặc vòng tròn
    // 1. Tăng biến đếm lên 1
    crossLineCount++;

    // 2. Cập nhật giá trị ngay lên màn hình OLED
    updateCrossLineOLED();
    speed_run(0, 0);
    delay(500);
    // 3. Hardcode tăng tốc để xe có lực kéo qua vạch đen ngang/vòng tròn
    speed_run(tocdo, tocdo);

    // Giữ nguyên tốc độ này trong 500 mili-giây.
    // Bạn cần tinh chỉnh số 500 này sao cho xe vừa đủ thoát khỏi cụm vạch đen
    // Nếu vạch quá to hoặc xe chạy chậm, hãy tăng lên 600, 700...
    delay(1000);

    // 5. Reset vị trí để sau khi thoát delay, thuật toán PID không bị giật mình
    vitri = 0;
    break;

  case 0b00011000:
  case 0b00111100:
    handleAndSpeed(servoPwm, tocdo);
    vitri = 0;
    break;

  case 0b00011100:
  case 0b00001000:
    handleAndSpeed(servoPwm, tocdo);
    vitri = 1;
    break;

  case 0b00010000:
  case 0b00111000:
    handleAndSpeed(servoPwm, tocdo);
    vitri = -1;
    break;

  case 0b00001100:
  case 0b00011110:
    handleAndSpeed(servoPwm, baseSpeed);
    vitri = 2;
    break;

  case 0b00000100:
  case 0b00001110:
    handleAndSpeed(servoPwm, (baseSpeed * 80 / 100));
    vitri = 3;
    break;

  case 0b00110000:
  case 0b01111000:
    handleAndSpeed(servoPwm, baseSpeed);
    vitri = -2;
    break;

  case 0b00100000:
  case 0b01110000:
    handleAndSpeed(servoPwm, (baseSpeed * 80 / 100));
    vitri = -3;
    break;

  case 0b00000110:
  case 0b00001111:
  case 0b00000010:
  case 0b00000111:
  case 0b00000011:
  case 0b00000001:
  case 0b00011111: // 5 mắt phải -> Cua gắt bằng PID
    // Nâng lên 55% (~ PWM 93) đảm bảo xe có lực kéo, PID sẽ tự ép bánh trong
    // quay lùi nếu angle đủ lớn
    handleAndSpeed(servoPwm, (baseSpeed * 65 / 100));
    vitri = 7;
    break;

  case 0b11111100:
  case 0b11111110:
    // Pattern 90 độ trái hoặc ngã ba -> Hardcode rẽ gắt
    speed_run(-120, 220);
    delay(150);
    vitri = -7;
    break;

  case 0b01100000:
  case 0b11110000:
  case 0b01000000:
  case 0b11100000:
  case 0b11000000:
  case 0b10000000:
  case 0b11111000: // 5 mắt trái -> Cua gắt bằng PID
    // Nâng lên 55% đảm bảo lực kéo qua cua
    handleAndSpeed(servoPwm, (baseSpeed * 55 / 100));
    vitri = -7;
    break;

  case 0b00111111:
  case 0b01111111:
    // Pattern 90 độ phải hoặc ngã ba -> Hardcode rẽ gắt
    speed_run(220, -120);
    delay(150);
    vitri = 7;
    break;

  case 0b11011000:
  case 0b10011000:
  case 0b11011100:
  case 0b11101000:
    // Các pattern nhiễu hoặc ngã ba chữ T một bên -> PID
    handleAndSpeed(servoPwm, (baseSpeed * 50 / 100));
    break;

  case 0b00000000:
    // Xử lý rẽ ngay lập tức (không lùi, không chờ 200ms) nếu mất line ở góc cua
    // gắt
    if (vitri <= -7) {
      // Line bên trái -> Rẽ trái: Trái lùi nhẹ/dừng, Phải tiến
      speed_run(0, 200);
      break;
    } else if (vitri >= 7) {
      // Line bên phải -> Rẽ phải: Trái tiến, Phải lùi nhẹ/dừng
      speed_run(200, 0);
      break;
    }

    // Xử lý vạch đứt (Zebra) hoặc mất line đường thẳng
    if (!isLostLine) {
      isLostLine = true;
      lostLineTimer = millis();
    }
    {
      unsigned long dt = millis() - lostLineTimer;
      if (dt < 300) {
        // < 200ms: có thể vạch zebra/đứt → tiếp tục PID
        handleAndSpeed(servoPwm, tocdo);
      } else {
        // >= 200ms: thực sự mất line

        // Phase 1 (200ms ~ 400ms): Lùi thẳng
        // Phase 2 (>= 400ms): Rẽ theo hướng line cuối cùng
        if (dt < 400) {
          speed_run(-150, -150); // Lùi thẳng
        } else {
          // Rẽ theo vitri để sweep tìm lại line
          if (vitri > 0) {
            speed_run(150, -100); // Line cuối ở phải → rẽ phải
          } else if (vitri < 0) {
            speed_run(-100, 150); // Line cuối ở trái → rẽ trái
          } else {
            speed_run(-150, -150); // Không rõ → tiếp tục lùi
          }
        }
      }
    }
    break;

  default:
    // Pattern không nhận ra (nhiễu/transition): giữ nguyên lái cũ, không phanh
    // đột ngột
    handleAndSpeed(servoPwm, tocdo);
    break;
  }
}
