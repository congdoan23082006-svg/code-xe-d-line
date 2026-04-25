#include <Arduino.h>
#include <EEPROM.h>
#include "Button.h"
#include "SensorMux.h"
#include "MotorControl.h"
#include "PID_Control.h"
#include "DisplayOLED.h"
#include "BLEControl.h"

#define EEPROM_SIZE 128

int mode = 0; // 0: READY, 1: CALIB, 2: SAVE/STOP, 3: PID RUN
int speedL = 0, speedR = 0;

void saveCalibToEEPROM() {
  // Lưu cảm biến (0-31)
  for (int i = 0; i < 8; i++) {
    EEPROM.write(2 * i,       sensorMin[i] / 16);
    EEPROM.write(2 * i + 1,   sensorMin[i] % 16);
    EEPROM.write(16 + 2 * i,     sensorMax[i] / 16);
    EEPROM.write(16 + 2 * i + 1, sensorMax[i] % 16);
  }
  // Lưu PID (32-47)
  EEPROM.put(32, Kp);
  EEPROM.put(36, Ki);
  EEPROM.put(40, Kd);
  EEPROM.put(44, baseSpeed);
  EEPROM.commit();
}

void loadCalibFromEEPROM() {
  // Load cảm biến
  for (int i = 0; i < 8; i++) {
    sensorMin[i] = EEPROM.read(2 * i) * 16 + EEPROM.read(2 * i + 1);
    sensorMax[i] = EEPROM.read(16 + 2 * i) * 16 + EEPROM.read(16 + 2 * i + 1);
    thresholdVal[i] = (sensorMin[i] + sensorMax[i]) / 2;
  }
  // Load PID
  float savedKp, savedKi, savedKd;
  int savedSpeed;
  EEPROM.get(32, savedKp);
  EEPROM.get(36, savedKi);
  EEPROM.get(40, savedKd);
  EEPROM.get(44, savedSpeed);
  
  // Chỉ dùng nếu giá trị hợp lệ (không phải quái đản do EEPROM mới)
  if (!isnan(savedKp) && savedKp >= 0 && savedKp < 10) Kp = savedKp;
  if (!isnan(savedKi) && savedKi >= 0) Ki = savedKi;
  if (!isnan(savedKd) && savedKd >= 0) Kd = savedKd;
  if (savedSpeed > 0 && savedSpeed <= 255) baseSpeed = savedSpeed;
}

void setup() {
  Serial.begin(115200);
  
  // Khởi tạo thiết bị
  initButtons();
  initSensors();
  initMotors();
  initOLED();
  resetPID();
  initBLEControl(); // Khởi tạo BLE thay vì Wifi

  EEPROM.begin(EEPROM_SIZE);
  loadCalibFromEEPROM();

  Serial.println("HE THONG SAN SANG!");
  
  display.clearDisplay();
  display.setCursor(0, 10);
  display.setTextSize(2);
  display.println("READY!");
  display.setTextSize(1);
  display.println("BLE: Robot-PID-BLE");
  display.println("Waiting for connect...");
  display.display();
}

void loop() {
  handleBLE(); // Xử lý BLE
  
  // Nút 1: Chuyển đổi giữa các chế độ
  if (isBtn1Clicked()) {
    mode++;
    if (mode > 3) mode = 1;

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);

    if (mode == 1) {
      initCalibValues();
      display.println("1: CALIB");
      Serial.println("MODE 1: Bat dau hoc mau...");
    } 
    else if (mode == 2) {
      saveCalibToEEPROM();
      motorControl(0, 0);
      display.println("2: LUU OK");
      display.setTextSize(1);
      display.setCursor(0, 25);
      display.print("Da luu vao EEPROM");

      display.setCursor(0, 35);
      display.print("Kp: "); display.println(Kp, 2);
      display.print("Ki: "); display.println(Ki, 3);
      display.print("Kd: "); display.println(Kd, 2);
      display.print("Spd: "); display.print(baseSpeed);

      Serial.println("MODE 2: Da luu calib.");
    } 
    else if (mode == 3) {
      resetPID();
      display.println("3: RUN PID");
      Serial.println("MODE 3: Dang do line...");
    }
    display.display();
    delay(500);
  }

  // Thực thi theo chế độ
  if (mode == 1) {
    updateCalib();
    updateOLED((unsigned int*)sensorValues);
  } 
  else if (mode == 3) {
    runPID(speedL, speedR);
    motorControl(speedL, speedR); // Đổi từ setMotor sang motorControl

    // Cập nhật OLED mỗi 100ms để không làm chậm vòng lặp PID
    static uint32_t lastOLEDUpdate = 0;
    if (millis() - lastOLEDUpdate > 100) {
      lastOLEDUpdate = millis();
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("MODE 3: RUN PID");
      
      display.setCursor(0, 15);
      display.print("Error: "); display.println((int)error);
      
      display.setCursor(0, 30);
      display.print("L: "); display.print(speedL);
      display.print(" | R: "); display.print(speedR);
      
      display.display();
    }
  }

  delay(10);
}