#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// ======== L298N Motor Driver (Chế độ 6 chân) ========

// Động cơ A (Bánh trái)
#define ENA  4  // Chân cấp PWM chỉnh tốc độ
#define IN1  16 // Chân chỉnh chiều
#define IN2  17 // Chân chỉnh chiều

// Động cơ B (Bánh phải)
#define IN3  18 // Chân chỉnh chiều
#define IN4  19 // Chân chỉnh chiều
#define ENB  23 // Chân cấp PWM chỉnh tốc độ

// Cấu hình PWM cho ESP32
const int pwmChannelLeft = 0;
const int pwmChannelRight = 1;
const int pwmFreq = 5000;
const int pwmResolution = 8; // Dải 0-255

void initMotors();
void motorControl(int speedLeft, int speedRight);
#endif