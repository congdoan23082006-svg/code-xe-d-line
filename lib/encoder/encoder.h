#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// Chia sẻ biến đếm để file main có thể đọc được (nếu cần)
extern volatile long enc1Count;
extern volatile long enc2Count;

// Khai báo hàm khởi tạo
void initEncoders();

// Khai báo các hàm ngắt
void IRAM_ATTR updateEncoder1();
void IRAM_ATTR updateEncoder2();

// Khai báo hàm tính toán quãng đường và reset
float calculateDistance();
void resetEncoders();

#endif // ENCODER_H