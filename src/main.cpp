#include <Arduino.h>
#include "Sensor.h" 
#include "Motion.h"
#include "DisplayOLED.h"

// Biến canh thời gian cho bộ PID
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  
  // Khởi tạo các module hệ thống
  sensor_init(); 
  motion_init();
  initOLED();
  
  // Mặc định khởi động vào Mode 2 (chờ)
  mode = 2;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println("He thong san sang!");
  display.setCursor(0, 25);
  display.println("Mode 2: Dung Hoc");
  display.display();
}

void loop() {
  // 1. Quản lý việc ấn nút chuyển Mode (1 -> 2 -> 3 -> 1...)
  if (isBtn1Clicked()) {
    mode++;
    if (mode > 3) mode = 1;

    if (mode == 1) {
      beep(100); 
      Serial.println("====================================");
      Serial.println("MODE 1: Bat dau hoc mau... Hay quet di quet lai!");
      
      for (int i = 0; i < 8; i++) {
        black_value[i] = 4095; 
        white_value[i] = 0;
      }
    } 
    else if (mode == 2) {
      beep(500); 
      Serial.println("MODE 2: Da hoc xong! Dang luu vao EEPROM...");
      
      display.clearDisplay();
      display.setCursor(0, 10);
      display.println("Dang Luu EEPROM...");
      display.display();
      
      for (int i = 0; i < 8; i++) {
        compare_value[i] = (black_value[i] + white_value[i]) / 2;
        
        EEPROM.write(2 * i, black_value[i] >> 8);
        EEPROM.write((2 * i) + 1, black_value[i] & 0xFF);
        EEPROM.write(16 + (2 * i), white_value[i] >> 8);
        EEPROM.write(17 + (2 * i), white_value[i] & 0xFF);
      }
      EEPROM.commit();
      Serial.println("Luu thanh cong! Chuyen sang che do doc.");
      Serial.println("====================================");
    }
    else if (mode == 3) {
      beep(100); delay(100); beep(100);
      Serial.println("MODE 3: Bat dau chay PID!");
      
      display.clearDisplay();
      display.setCursor(0, 10);
      display.println("Mode 3:");
      display.println("Running PID...");
      display.display();
      
      // Reset thời gian cho PID
      lastTime = millis();
      calPID = 1;
    }
  }

  // 2. Thực thi liên tục theo Mode hiện tại
  if (mode == 1) {
    for (int j = 0; j < 8; j++) {
      setMuxChannel(j);
      delayMicroseconds(5);
      sensorValue[j] = 4095 - analogRead(SIG_PIN);

      if (sensorValue[j] < black_value[j]) black_value[j] = sensorValue[j];
      if (sensorValue[j] > white_value[j]) white_value[j] = sensorValue[j];
    }
    
    // In màn hình OLED giá trị đang quyét
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 100) {
      updateOLED(sensorValue);
      lastUpdate = millis();
    }
    
    delay(5);
  } 
  else if (mode == 2) {
    read_sensor_74hc4067(); // Đọc và quy đổi ra nhi phân

    Serial.print("Gia tri nhi phan (BIN): ");
    for (int i = 7; i >= 0; i--) {
      Serial.print(bitRead(sensor, i));
    }
    Serial.println();
    
    // Hiển thị trạng thái lên OLED
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 100) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Mode 2: Dung Hoc");
      display.setCursor(0, 20);
      display.print("BIN: ");
      for (int i = 7; i >= 0; i--) {
        display.print(bitRead(sensor, i));
      }
      display.display();
      lastUpdate = millis();
    }
    
    speed_run(0, 0); // Phanh xe an toàn thiết lập trước
    delay(50); 
  }
  else if (mode == 3) {
    read_sensor_74hc4067(); // Liên tục đọc giá trị dùng cho PID
    
    // Bộ tạo nhịp để tính PID theo chu kì (khoảng 5ms)
    if (millis() - lastTime >= 5) {
      lastTime = millis();
      calPID = 1;
    }
    
    // Gọi hàm chạy bám line theo PID
    runforwardline(baseSpeed);
    
    // Hiển thị OLED (Cập nhật chậm để không làm lag PID)
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 500) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Mode 3: Chay PID");
      display.setCursor(0, 15);
      display.printf("Speed: %d\n", baseSpeed);
      display.printf("Kp:%.1f Kd:%.1f", Kp, Kd);
      display.display();
      lastUpdate = millis();
    }
  }
}