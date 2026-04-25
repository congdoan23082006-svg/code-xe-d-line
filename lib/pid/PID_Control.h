#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <Arduino.h>

// Hệ số PID (extern để có thể thay đổi từ main nếu muốn)
extern float Kp, Ki, Kd;
extern int baseSpeed;

// Reset biến tích lũy PID
void resetPID();

// Tính PID và xuất tốc độ 2 bánh
void runPID(int &speedLeft, int &speedRight);

// Biến toàn cục để debug OLED
extern float error;

#endif