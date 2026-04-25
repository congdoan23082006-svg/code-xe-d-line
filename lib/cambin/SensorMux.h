#ifndef SENSOR_MUX_H
#define SENSOR_MUX_H

#include <Arduino.h>

// --- KHAI BÁO CHÂN 74HC4067 ---
#define MUX_SIG 35  
#define MUX_S0  5   
#define MUX_S1  13
#define MUX_S2  12

// Khai báo biến toàn cục để main.cpp và OLED có thể truy cập
extern int sensorMin[8]; 
extern int sensorMax[8];
extern int thresholdVal[8];
extern int sensorValues[8];

// Các hàm khởi tạo và đọc dữ liệu
void initSensors();
void updateCalib();      // Cập nhật min/max khi học màu
void initCalibValues();  // Reset giá trị calib
int readLinePosition();  // Trả về vị trí vạch (0 - 7000)

#endif