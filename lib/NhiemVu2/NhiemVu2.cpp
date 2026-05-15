#include "NhiemVu2.h"
#include "DisplayOLED.h"
#include "Motion.h"
#include "Sensor.h"
#include "encoder.h"
#include "gripper.h"

extern volatile long enc1Count;

volatile float Kp_maze = 2;
volatile float Ki_maze = 0;
volatile float Kd_maze = 13;
volatile long lastPos_maze = 0;
int servoPwm_maze = 0;
int mazeSpeed = 150;

int leftTurnCount = 0;
int crossCount = 0;
bool isCountingEncoder = false;
int mazeState = 0;

// CÁC SỰ KIỆN TRÊN ĐƯỜNG
#define EVENT_NONE 0
#define EVENT_CROSS 1
#define EVENT_LEFT 2
#define EVENT_RIGHT 3

void nhiemvu2_init() {}

void nhiemvu2_start() {
  speed_run(0, 0);
  leftTurnCount = 0;
  enc1Count = 0;
  enc2Count = 0;
  crossCount = 0;
  isCountingEncoder = false;
  mazeState = 0;

  if (isOledOk) {
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("READY IN 3s...");
    display.display();
  }

  beep(200);
  delay(3000);
  beep(500);

  motion_reset();
  lastPos_maze = 0;
  calPID = 1;
}

void PID_maze(uint8_t current_sensor) {
  if (calPID == 1) {
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
      i = lastPos_maze;
    } else {
      long position = weightedSum / activeCount;
      i = position - 105;
    }

    static long sumError = 0;
    if (abs(i) < 30)
      sumError += i;
    if (sumError > 5000)
      sumError = 5000;
    if (sumError < -5000)
      sumError = -5000;

    long iP = (long)(Kp_maze * i);
    long iI = (long)(Ki_maze * sumError);
    long iD = (long)(Kd_maze * (lastPos_maze - i));
    long iRet = iP + iI - iD;

    servoPwm_maze = (int)iRet;
    lastPos_maze = i;
    calPID = 0;
  }
}

// BỘ LỌC SỰ KIỆN CHỐNG NHIỄU (DEBOUNCE 25ms)
int getMazeEvent(uint8_t sensor_val) {
  static int pendingEvent = EVENT_NONE;
  static unsigned long debounceTime = 0;

  int rawEvent = EVENT_NONE;
  int bCount = __builtin_popcount(sensor_val);

  if (bCount >= 6) {
    rawEvent = EVENT_CROSS;
  } else if (bCount >= 4) {
    if ((sensor_val & 0b11100000) && !(sensor_val & 0b00000011))
      rawEvent = EVENT_LEFT;
    else if ((sensor_val & 0b00000111) && !(sensor_val & 0b11000000))
      rawEvent = EVENT_RIGHT;
  }

  if (rawEvent != EVENT_NONE) {
    if (pendingEvent == EVENT_NONE) {
      pendingEvent = rawEvent;
      debounceTime = millis();
    } else {
      if (rawEvent == EVENT_CROSS)
        pendingEvent = EVENT_CROSS;

      if (millis() - debounceTime >= 25) {
        int confirmed = pendingEvent;
        pendingEvent = EVENT_NONE;
        return confirmed;
      }
    }
  } else {
    if (pendingEvent != EVENT_NONE) {
      if (sensor_val == 0b00011000 || sensor_val == 0b00111100 ||
          sensor_val == 0b00011100 || sensor_val == 0b00111000) {
        pendingEvent = EVENT_NONE;
      } else if (millis() - debounceTime >= 25) {
        int confirmed = pendingEvent;
        pendingEvent = EVENT_NONE;
        return confirmed;
      }
    }
  }
  return EVENT_NONE;
}

void nhiemvu2_loop() {
  static unsigned long lastTime_maze = 0;
  if (millis() - lastTime_maze >= 5) {
    lastTime_maze = millis();
    calPID = 1;
  }

  read_sensor_74hc4067();

  int blackCount = __builtin_popcount(sensor);
  if (!whiteLineMode && blackCount >= 5 && blackCount <= 7) {
    uint8_t whiteMask = ~sensor;
    if (whiteMask & 0b00111100)
      whiteLineMode = true;
  }
  if (whiteLineMode && blackCount >= 1 && blackCount <= 2) {
    uint8_t blackMask = sensor;
    if (blackMask & 0b00111100)
      whiteLineMode = false;
  }

  uint8_t processed_sensor = sensor;
  if (whiteLineMode) {
    processed_sensor ^= 0xFF;
  }

  int event = getMazeEvent(processed_sensor);

  // ========================================================
  // KỊCH BẢN: NGÃ TƯ 1 (ĐI THẲNG) -> NGÃ TƯ 2 (RẼ TRÁI) -> NGÃ TƯ 3 (PHÓNG 20CM) -> GẮP
  // ========================================================

  // STATE 0: TÌM NGÃ TƯ LẦN 1
  if (mazeState == 0) {
    if (event == EVENT_CROSS) {
      beep(100);
      // Lần 1: Đi thẳng qua ngã tư
      speed_run(160, 160);
      delay(150); // Phóng mù 150ms qua vạch ngang để không đếm đúp
      mazeState = 1;
      return;
    }
  }
  // STATE 1: TÌM NGÃ TƯ LẦN 2
  else if (mazeState == 1) {
    if (event == EVENT_CROSS) {
      beep(100);
      // Lần 2: Rẽ trái gắt
      speed_run(120, 120);
      delay(80); // Nhích lên một chút để tâm xoay nằm giữa ngã tư
      speed_run(-200, 180);
      delay(300); // Xoay trái gắt

      vitri = -7; // Ép ảo lệch trái để PID tự nắn lại
      mazeState = 2;
      return;
    }
  }
  // STATE 2: TÌM NGÃ TƯ LẦN 3
  else if (mazeState == 2) {
    if (event == EVENT_CROSS) {
      beep(100);
      resetEncoders(); // Bắt đầu tính quãng đường để phóng mù
      mazeState = 3;
      return;
    }
  }
  // STATE 3: PHÓNG MÙ 20CM
  else if (mazeState == 3) {
    if (calculateDistance() < 20.0) {
      // Phóng thẳng bằng encoder (giữ chênh lệch encoder = 0)
      handleAndSpeed((enc2Count - enc1Count) * 3, 160);
      return;
    } else {
      speed_run(0, 0);
      delay(800);
      gripper_grab(); // THỰC HIỆN LỆNH GẮP
      mazeState = 4;
      return;
    }
  }
  // STATE 4: KẾT THÚC
  else if (mazeState == 4) {
    speed_run(0, 0);
    return;
  }

  // --- CẬP NHẬT OLED THEO DÕI STATE ---
  static unsigned long lastOLED_NV2 = 0;
  if (millis() - lastOLED_NV2 > 200) {
    if (isOledOk) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("NV2 - State: ");
      display.print(mazeState);
      display.setCursor(0, 16);
      if (mazeState == 0)
        display.print("Tim NgaTu 1(Thang)");
      else if (mazeState == 1)
        display.print("Tim NgaTu 2(Trai)");
      else if (mazeState == 2)
        display.print("Tim NgaTu 3(Phong)");
      else if (mazeState == 3)
        display.print("Sprint 20cm...");
      else if (mazeState == 4)
        display.print("DONE & GRABBED");
      display.display();
    }
    lastOLED_NV2 = millis();
  }

  // Chạy PID bình thường
  PID_maze(processed_sensor);

  static bool isLostLine = false;
  static unsigned long lostLineTimer = 0;
  if (processed_sensor != 0) {
    isLostLine = false;
  }
  // Switch Case xử lý bẻ lái thông minh
  switch (processed_sensor) {
  case 0b11111111:
  case 0b01111110:
    handleAndSpeed(servoPwm_maze, mazeSpeed);
    vitri = 0;
    break;

  case 0b00011000:
  case 0b00111100:
  case 0b00011100:
  case 0b00001000:
  case 0b00010000:
  case 0b00111000:
    handleAndSpeed(servoPwm_maze, mazeSpeed);
    vitri = 0;
    break;

  case 0b00001100:
  case 0b00011110:
    handleAndSpeed(servoPwm_maze, mazeSpeed);
    vitri = 2;
    break;

  case 0b00000100:
  case 0b00001110:
    handleAndSpeed(servoPwm_maze, (mazeSpeed * 80 / 100));
    vitri = 3;
    break;

  case 0b00110000:
  case 0b01111000:
    handleAndSpeed(servoPwm_maze, mazeSpeed);
    vitri = -2;
    break;

  case 0b00100000:
  case 0b01110000:
    handleAndSpeed(servoPwm_maze, (mazeSpeed * 80 / 100));
    vitri = -3;
    break;

  case 0b00000110:
  case 0b00001111:
  case 0b00000010:
  case 0b00000111:
  case 0b00000011:
  case 0b00000001:
  case 0b00011111:
    handleAndSpeed(servoPwm_maze, (mazeSpeed * 65 / 100));
    vitri = 7;
    break;

  case 0b11111100:
  case 0b11111110:
  case 0b00111111:
  case 0b01111111:
    handleAndSpeed(servoPwm_maze, mazeSpeed);
    break;

  case 0b01100000:
  case 0b11110000:
  case 0b01000000:
  case 0b11100000:
  case 0b11000000:
  case 0b10000000:
  case 0b11111000:
    handleAndSpeed(servoPwm_maze, (mazeSpeed * 55 / 100));
    vitri = -7;
    break;

  case 0b00000000:
    if (vitri <= -7) {
      speed_run(-120, 120);
      break;
    } else if (vitri >= 7) {
      speed_run(120, -120);
      break;
    } else if (vitri <= -3) {
      speed_run(-110, 0);
      break;
    } else if (vitri >= 3) {
      speed_run(0, -110);
      break;
    }

    if (!isLostLine) {
      isLostLine = true;
      lostLineTimer = millis();
    }
    {
      unsigned long dt = millis() - lostLineTimer;
      if (dt < 180) {
        speed_run(mazeSpeed, mazeSpeed);
      } else if (dt < 400) {
        speed_run(-150, -150);
      } else {
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
    handleAndSpeed(servoPwm_maze, mazeSpeed);
    break;
  }
}
