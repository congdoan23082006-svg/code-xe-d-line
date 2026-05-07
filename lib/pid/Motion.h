#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

// ==========================================
// CẤU HÌNH CHÂN CHO MẠCH L298N CỦA BẠN
// ==========================================
// Động cơ trái (Motor A)
#define ENA 4       // Chân cấp PWM chỉnh tốc độ Trái
#define IN1 16      // Chân chỉnh chiều 1 Trái
#define IN2 17      // Chân chỉnh chiều 2 Trái

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
extern int servoPwm;
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
extern int count180; // Biến đếm số lần gặp vạch 180 độ (8 mắt đen)
extern int current_stage; // Biến trạng thái chặng đường
extern bool is_final_stage; // Cờ báo hiệu chặng cuối (chuẩn bị về đích)
extern int dashedLineCount; // Đếm số vạch đứt

void motion_init();
void speed_run(int speedDC_left, int speedDC_right);
void handleAndSpeed(int angle, int speed1);
void PID();
void runforwardline(int tocdo);

#endif