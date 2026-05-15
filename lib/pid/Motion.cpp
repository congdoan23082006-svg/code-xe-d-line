
#include "Motion.h"
#include "DisplayOLED.h"
#include "Sensor.h"
#include "encoder.h"

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
unsigned long runStartTime = 0;
bool isRunTimerStarted = false;
bool whiteLineMode = false;

void updateInfoOLED(unsigned long timeMs) {
  if (!isOledOk)
    return;
  float currentDist = calculateDistance();

  display.clearDisplay();
  display.setTextSize(1); // In chữ nhỏ để hiển thị được nhiều dòng
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("PID Running...");

  display.setCursor(0, 20);
  display.print("Dist: ");
  display.print(currentDist);
  display.print(" cm");

  display.display();
}

void motion_reset() {
  dashedLineCount = 0;
  isRunTimerStarted = false;

  vitri = 0;
  lastPos = 0;
  posPID = 0;
  calPID = 0;

  resetEncoders(); // Reset lại quãng đường về 0
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
  // Bắt đầu đếm thời gian khi hàm chạy lần đầu
  if (!isRunTimerStarted) {
    runStartTime = millis();
    isRunTimerStarted = true;
  }

  unsigned long elapsedTime = millis() - runStartTime;

  // Cập nhật OLED mỗi 200ms để không làm lag vòng lặp PID
  static unsigned long lastOLEDTime = 0;
  if (millis() - lastOLEDTime > 200) {
    updateInfoOLED(elapsedTime);
    lastOLEDTime = millis();
  }

  // Lấy quãng đường hiện tại để quyết định chiến thuật
  float currentDistance = calculateDistance();

  // =========================================================================
  // LOGIC QUÃNG ĐƯỜNG
  // =========================================================================

  float dist_StartSprint = 250;
  float dist_EndSprint =
      300; // Mốc 2: Đi được 350cm thì kết thúc vút thẳng, quay lại PID
  float dist_StopAll = 420.0; // Mốc 3: Đi đủ 500cm thì dừng xe vĩnh viễn (Đích)

  if (currentDistance >= dist_StopAll) {
    // 3. Đến đích -> Phanh chết vĩnh viễn
    speed_run(0, 0);
    while (true) {
      delay(100);
    }
  } else if (currentDistance >= dist_StartSprint &&
             currentDistance < dist_EndSprint) {
    // 2. Nằm trong đoạn đường nước rút -> Vút thẳng max tốc, bỏ qua PID
    speed_run(200, 200);
    return; // Thoát hàm để chặn code PID bên dưới chạy
  }

  // ── White line mode (line trắng nền đen) ──────────────────────────
  // -- Biến "Thả PID" sau khi qua đoạn Line Trắng --
  static bool hasPassedWhiteLine = false;      // Đã kích hoạt boost chưa?
  static unsigned long whiteLineExitTime = 0;  // Thời điểm vừa thoát Line Trắng
  static bool waitingToBoost = false;          // Đang đếm ngược 2 giây không?
  static unsigned long boostTimer = 0;         // Hẹn giờ mỗi cú vọt
  static bool isBoosting = false;              // Đang trong cú vọt không?
  int blackCount = __builtin_popcount(sensor); // Đếm số mắt ĐEN gốc

  // 1. Điều kiện VÀO vùng line trắng (Nền đen, line trắng)
  if (!whiteLineMode) {
    // Rìa trái có đen, rìa phải có đen, và ở giữa có trắng
    bool blackOnLeft = (sensor & 0b11000000) != 0;
    bool blackOnRight = (sensor & 0b00000011) != 0;
    bool whiteInMiddle = (~sensor & 0b00111100) != 0;
    
    if (blackOnLeft && blackOnRight && whiteInMiddle && blackCount >= 4) {
      whiteLineMode = true;
      hasPassedWhiteLine = false; // Reset khi vào lại đoạn trắng
      waitingToBoost = false;
    }
  }

  // 2. Điều kiện THOÁT vùng line trắng (Trở về nền trắng)
  // Khôi phục lại điều kiện cũ của bạn: đếm 1-3 mắt đen là thoát.
  // Vì nếu vừa thoát ra gặp cua gắt ngay, 1 bên rìa sẽ đè vạch đen, điều kiện 2 bên cùng trắng sẽ bị sai!
  else if (whiteLineMode && (blackCount == 1 || blackCount == 2 || blackCount == 3)) {
    whiteLineMode = false;
    waitingToBoost = true;        // Bắt đầu đếm ngược 2 giây
    whiteLineExitTime = millis(); // Ghi nhớ thời điểm vừa thoát
    isBoosting = false;
  }

  // 3. Sau 2 giây kể từ lúc thoát Line Trắng -> Kích hoạt boost
  if (waitingToBoost && !hasPassedWhiteLine) {
    if (millis() - whiteLineExitTime >= 2000) { // Đợi đúng 2 giây
      hasPassedWhiteLine = true;                // Kích hoạt boost!
      waitingToBoost = false;
      boostTimer = millis(); // Bắt đầu đồng hồ boost
    }
  }

  // 3. Xử lý tín hiệu cho PID và Switch Case
  uint8_t processed_sensor = sensor;
  if (whiteLineMode) {
    processed_sensor ^= 0xFF; // Đảo bit để các case 0b00011000... chạy đúng
  }

  // =========================================================================
  // ƯU TIÊN LỌC NHIỄU NGÃ BA TRƯỚC KHI CHẠY PID
  // (Ngăn không cho các pattern rác này lọt vào hàm PID để tránh làm hỏng
  // biến lastPos và lịch sử thuật toán).
  // =========================================================================
  static unsigned long lastTurnTime = 0;

  if (processed_sensor == 0b11011000 || processed_sensor == 0b10011000 ||
      processed_sensor == 0b11011100 || processed_sensor == 0b11101000 ||
      processed_sensor == 0b10111000 || processed_sensor == 0b11001000 ||
      processed_sensor == 0b10101000 || processed_sensor == 0b11001100 ||
      processed_sensor == 0b11101100 || processed_sensor == 0b10001000 ||
      processed_sensor == 0b10001100 || processed_sensor == 0b10011100) {
    if (millis() - lastTurnTime > 300) {
      speed_run(0, 0);
      delay(15);
      speed_run(-120, 220);
      delay(135);
      vitri = -7;
      lastTurnTime = millis();
    }
    return;
  }

  if (processed_sensor == 0b00011011 || processed_sensor == 0b00011001 ||
      processed_sensor == 0b00111011 || processed_sensor == 0b00010111 ||
      processed_sensor == 0b00011101 || processed_sensor == 0b00010011 ||
      processed_sensor == 0b00010101 || processed_sensor == 0b00110011 ||
      processed_sensor == 0b00110111 || processed_sensor == 0b00010001 ||
      processed_sensor == 0b00110001 || processed_sensor == 0b00111001) {
    if (millis() - lastTurnTime > 300) {
      speed_run(0, 0);
      delay(15);
      speed_run(220, -120);
      delay(135);
      vitri = 7;
      lastTurnTime = millis();
    }
    return;
  }

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
    // Đã bỏ biến đếm vạch. Bây giờ ô đen chỉ là đoạn cần chạy thẳng qua
    handleAndSpeed(servoPwm, tocdo);
    vitri = 0;
    break;

  case 0b00011000:
  case 0b00111100:
  case 0b00011100:
  case 0b00001000:
  case 0b00010000:
  case 0b00111000:
    // Chạy PID bình thường
    handleAndSpeed(servoPwm, tocdo);
    vitri = 0;
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
    if (millis() - lastTurnTime > 300) {

      speed_run(-120, 220);
      delay(135);
      vitri = -7;
      lastTurnTime = millis();
    }
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
    if (millis() - lastTurnTime > 300) {
      speed_run(220, -120);
      delay(135);
      vitri = 7;
      lastTurnTime = millis();
    }
    break;

  case 0b00000000:
    // 1. Văng khỏi góc cua GẮT (Góc nhọn Z-turn / V-turn)
    if (vitri <= -7) {
      // Văng ở bên trái -> Quay tại chỗ gắt sang trái để tìm lại line
      speed_run(-150, 150);
      break;
    } else if (vitri >= 7) {
      speed_run(150, -150);
      break;
    }
    // 2. Văng khi cua VỪA (Cua gắt nhưng chưa đến mức quay văng)
    else if (vitri <= -3) {
      speed_run(-2, 0);
      break;
    } else if (vitri >= 3) {
      speed_run(0, -210);
      break;
    }

    // 3. Xử lý vạch đứt (Zebra) hoặc mất line đường thẳng (Khi vitri gần 0)
    if (!isLostLine) {
      isLostLine = true;
      lostLineTimer = millis();
    }
    {
      unsigned long dt = millis() - lostLineTimer;
      if (dt < 180) {
        // < 180ms: Giả định là vạch đứt ngang → Lao thẳng qua bằng tốc độ hiện
        // tại
        speed_run(tocdo, tocdo);
      } else if (dt < 400) {
        // Hết 180ms mà không thấy line -> Đích thị là lọt ra ngoài sa bàn ->
        // Lùi thẳng
        speed_run(-150, -150);
      } else {
        // Lùi mãi không thấy -> Xoay xe quét tìm
        if (vitri > 0) {
          speed_run(150, -150);
        } else if (vitri < 0) {
          speed_run(-150, 150);
        } else {
          speed_run(-150, -150);
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