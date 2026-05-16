#include "gripper.h"
#include <Arduino.h> // Chỉ cần Arduino.h, không cần ESP32Servo.h nữa

const int servoPin = 14;

// --- ĐỊNH NGHĨA CÁC THÔNG SỐ PWM ---
// LƯU Ý QUAN TRỌNG: Chọn một kênh LEDC từ 0 đến 15.
// Bạn cần chắc chắn kênh này KHÔNG TRÙNG với các kênh mà Motion.cpp (động cơ
// DC) đang sử dụng.
const int servoChannel = 4; // Ví dụ chọn kênh 4
const int pwmFreq =
    50; // Tần số chuẩn cho Servo là 50Hz (chu kỳ 20ms = 20000us)
const int pwmResolution =
    16; // Độ phân giải 16-bit (giá trị Duty Cycle từ 0 đến 65535)

void gripper_write(int angle) {
  // 1. Giới hạn góc an toàn từ 0 đến 90 độ (theo như code cũ của bạn để tránh
  // quá góc)
  angle = constrain(angle, 0, 90);

  // 2. Chuyển đổi góc (0 - 180) sang dải xung chuẩn (500us - 2400us)
  // Lưu ý: Dù giới hạn góc dùng ở 90 độ, hàm map vẫn cần ánh xạ theo dải 0-180
  // để ra số microgiây chuẩn xác.
  int pulse_us = map(angle, 0, 180, 500, 2400);

  // 3. Tính toán giá trị Duty Cycle
  // Với tần số 50Hz, 1 chu kỳ = 20000us. Độ phân giải 16-bit có giá trị max là
  // 65535. Công thức quy đổi: duty = (pulse_us / 20000) * 65535
  int dutyCycle = map(pulse_us, 0, 20000, 0, 65535);

  // 4. Xuất tín hiệu PWM ra kênh
  ledcWrite(servoChannel, dutyCycle);
}

void gripper_init() {
  // 1. Khởi tạo kênh PWM phần cứng với tần số 50Hz và độ phân giải 16-bit
  ledcSetup(servoChannel, pwmFreq, pwmResolution);

  // 2. Kết nối chân servoPin với kênh PWM vừa tạo
  ledcAttachPin(servoPin, servoChannel);

  // 3. Di chuyển Servo về góc 0 độ mặc định
  gripper_write(30);
}

void gripper_grab() {
  gripper_write(30); // GIẢM GÓC XUỐNG 55 (hoặc 60) ĐỂ KHÔNG ÉP QUÁ CHẶT VÀO VẬT
  Serial.println("=> Da giu Servo o goc: 55 do");

  // MẸO CHỐNG KÊU È È: Đợi servo kẹp xong (khoảng nửa giây) rồi ngắt điện luôn!
  // Bánh răng servo có đủ ma sát để giữ vật nhẹ mà không cần cấp điện gồng liên
  // tục.
  delay(500);
  ledcWrite(servoChannel, 0);
}

void gripper_release() {
  gripper_write(90);
  Serial.println("=> Da giu Servo o goc: 0 do (Dung LEDC PWM)");
}
