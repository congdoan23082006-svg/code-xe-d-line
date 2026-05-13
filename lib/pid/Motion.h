#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

// ==========================================
// CẤU HÌNH CHÂN CHO MẠCH L298N CỦA BẠN
// ==========================================
// Động cơ trái (Motor A)
#define ENA 4       // Chân cấp PWM chỉnh tốc độ Trái
#define IN1 17      // Chân chỉnh chiều 1 Trái (Đã đảo ngược)
#define IN2 16      // Chân chỉnh chiều 2 Trái (Đã đảo ngược)

// Động cơ phải (Motor B)
#define IN3 18      // Chân chỉnh chiều 1 Phải
#define IN4 19      // Chân chỉnh chiều 2 Phải
#define ENB 23      // Chân cấp PWM chỉnh tốc độ Phải

// Cấu hình kênh băm xung (LEDC) cho ESP32
#define ENA_CH 0    // Kênh 0 phụ trách chân ENA (Chân 4)
#define ENB_CH 1   // Kênh 1 phụ trách chân ENB (Chân 23)

// ==========================================
// CÁC BIẾN TOÀN CỤC VÀ HÀM
// ==========================================
extern int turnAngle;
extern int vitri;
extern int baseSpeed;
extern volatile int calPID;
extern volatile long lastPos;
extern volatile float Kp;
extern volatile float Ki;
extern volatile float Kd;
extern volatile int isLeft;
extern volatile int isRight;
extern volatile long posPID;

void motion_init();
void speed_run(int speedDC_left, int speedDC_right);
void handleAndSpeed(int angle, int speed1);
void PID(uint8_t current_sensor);
void runforwardline(int tocdo);
void motion_reset();

#endif
