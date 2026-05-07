#include "Motion.h"
#include "Sensor.h" // Chứa mảng sensor và sensorValue

// Khởi tạo các biến
int servoPwm = 0;
int vitri = 0;
int baseSpeed = 170; // Dải PWM 0-255 (chỉnh theo yêu cầu)
volatile int calPID = 0;
volatile long lastPos = 0;
volatile float Kp = 1.0;
volatile float Ki = 0;
volatile float Kd = 13;
volatile int isLeft = 0;
volatile int isRight = 0;
volatile long posPID = 0;
int count180 = 0;            // Biến đếm số lần đè 8 mắt đen
int current_stage = 0;       // Biến trạng thái chặng đường
bool is_final_stage = false; // Cờ báo hiệu chặng cuối (chuẩn bị về đích)
int dashedLineCount = 0;     // Đếm số vạch đứt

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

void PID() {
  if (calPID == 1) {
    // ============================================================
    // Tính vị trí line từ sensor byte nhị phân (ĐEN = 1)
    // Sensor vật lý j nằm ở bit (7-j) trong byte `sensor`
    // Trọng số vị trí: j * 3000 → dải 0 (trái) đến 21000 (phải)
    // Tâm line = 10500
    // ============================================================
    long weightedSum = 0;
    long activeCount = 0;

    for (int j = 0; j < 8; j++) {
      if (bitRead(sensor, 7 - j)) {
        weightedSum += (long)j * 30; // dải 0→210, tâm = 105
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
  int blackCount = __builtin_popcount(sensor);

  // Trở lại viết theo các case cụ thể như trước
  if (sensor == 0b11100111 || sensor == 0b11001111 || sensor == 0b11110011 ||
      sensor == 0b11101111 || sensor == 0b11110111 || // 1 mắt trắng
      sensor == 0b11000111 || sensor == 0b11100011 || // 3 mắt trắng
      sensor == 0b10011111 || sensor == 0b11111001) {
    whiteLineMode = true; // Nhận diện vào vùng line trắng (đổi logic)
  }
  // Vẫn chặn blackCount == 0 để xe không thoát nhầm khi qua checkpoint vạch
  // ngang
  if (whiteLineMode &&
      (blackCount == 1 || blackCount == 2 || blackCount == 3)) {
    whiteLineMode = false; // Ra khỏi vùng → về line đen
  }

  if (whiteLineMode) {
    sensor ^= 0xFF; // Đảo bit: PID + switch hoạt động đúng
  }
  // ──────────────────────────────────────────────────────────────────

  PID();

  // Debounce mất line: reset timer khi còn thấy line
  static bool isLostLine = false;
  static unsigned long lostLineTimer = 0;

  // -- Biến cho nhận diện vạch đứt đích (FINISH) --
  static bool wasOnLine = true;
  static unsigned long lastDashedTime = 0; // Chống nhiễu đếm vạch đứt

  if (sensor != 0) {
    isLostLine = false;
    wasOnLine = true;
  } else {
    // sensor == 0 (Mất line hoàn toàn)
    if (wasOnLine) {
      // CHỈ đếm vạch đứt nếu đã vào chặng cuối (tránh đếm nhầm ở các chặng
      // trước) Chống nhiễu: 2 lần đếm phải cách nhau ít nhất 150ms (phù hợp tốc
      // độ cao)
      if (is_final_stage && (millis() - lastDashedTime > 150)) {
        dashedLineCount++;
        lastDashedTime = millis();
      }
      wasOnLine = false;
    }
  }

  // Debounce chống đếm chập vạch 180 độ
  static bool is180Triggered = false;
  if (sensor != 0b11111111) {
    is180Triggered = false;
  }

  switch (sensor) {
  case 0b11111111: // 8 mắt cùng đọc thấy line đen (Vạch ngang 180 độ /
                   // Checkpoint)
    if (!is180Triggered) {
      count180++;
      current_stage++; // Tăng trạng thái mỗi khi qua checkpoint/vạch ngang
      is180Triggered = true;
      beep(200); // Kêu bíp báo hiệu đã đếm

      // KIỂM TRA CHUYỂN TRẠNG THÁI CHẶNG CUỐI
      if (current_stage >= 3) {
        is_final_stage = true;
      }
    }

    // LOGIC DỪNG FINISH KHI CHẠM VẠCH ĐEN SAU ĐOẠN VẠCH ĐỨT
    if (is_final_stage && dashedLineCount >= 4) {
      speed_run(0, 0); // Phanh chết
      while (true) {   // Vòng lặp khóa xe vĩnh viễn ở đích
        beep(500);
        delay(500);
      }
    } else {
      // Chưa đến đích -> Đi thẳng 300ms để vượt qua vạch ngang
      speed_run(tocdo, tocdo);
      delay(300);
    }
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
    handleAndSpeed(servoPwm, (baseSpeed * 70 / 100));
    vitri = 3;
    break;

  case 0b00110000:
  case 0b01111000:
    handleAndSpeed(servoPwm, baseSpeed);
    vitri = -2;
    break;

  case 0b00100000:
  case 0b01110000:
    handleAndSpeed(servoPwm, (baseSpeed * 70 / 100));
    vitri = -3;
    break;

  case 0b00000110:
  case 0b00001111:
  case 0b00000010:
  case 0b00000111:
  case 0b00000011:
  case 0b00000001:
  case 0b00011111: // 5 sensor xa phải
  case 0b00111111: // 6 sensor xa phải
  case 0b01111111: // 7 sensor xa phải
    if (vitri <= -7) {
      speed_run(0, 255);
      break;
    } else {
      handleAndSpeed(servoPwm, (baseSpeed * 50 / 100));
      vitri = 7;
    }
    break;

  case 0b11011000:
  case 0b10011000:
  case 0b11011100:
  case 0b11101000:

// 2 mắt trái + 2 mắt giữa → rẽ trái gắt
    speed_run(0, 255);
    delay(200);
    vitri = -7;
    break;

  case 0b01100000:
  case 0b11110000:
  case 0b01000000:
  case 0b11100000:
  case 0b11000000:
  case 0b10000000:
  case 0b11111000: // 5 sensor xa trái
  case 0b11111100: // 6 sensor xa trái
  case 0b11111110: // 7 sensor xa trái
    if (vitri >= 7) {
      speed_run(255, 0);
      break;
    } else {
      handleAndSpeed(servoPwm, (baseSpeed * 50 / 100));
      vitri = -7;
    }
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

        // LOGIC DỪNG FINISH KHI MẤT LINE HOÀN TOÀN SAU ĐOẠN VẠCH ĐỨT
        if (is_final_stage && dashedLineCount >= 4) {
          speed_run(0, 0); // Phanh chết ở đích
          while (true) {
            beep(500);
            delay(500);
          }
        }

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