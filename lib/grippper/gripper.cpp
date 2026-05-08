#include "Gripper.h"
#include <ESP32Servo.h>

Servo gripperServo;
const int servoPin = 26; 

void gripper_init() {
  // BƯỚC QUAN TRỌNG: Cấp phép cho thư viện tự động tìm Timer đang rảnh.
  // Nhờ các lệnh này, nó sẽ né cái Timer mà Motion.cpp (động cơ DC) đang dùng.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  gripperServo.setPeriodHertz(50); // Tần số chuẩn cho Servo 50Hz
  
  // Khởi tạo chân Servo với dải xung chuẩn 500us - 2400us
  gripperServo.attach(servoPin, 500, 2400); 
  
  gripper_write(0);
}

void gripper_write(int angle) {
  if (angle >= 0 && angle <= 90) {
    gripperServo.write(angle); // Ghi thẳng góc, thư viện tự tính toán an toàn
  }
}

void gripper_grab() {
  gripper_write(90);
  Serial.println("=> Da giu Servo o goc: 90 do (Dung ESP32Servo)");
}

void gripper_release() {
  gripper_write(0);
  Serial.println("=> Da giu Servo o goc: 0 do (Dung ESP32Servo)");
}
