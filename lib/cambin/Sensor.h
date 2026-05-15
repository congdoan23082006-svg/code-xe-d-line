#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <EEPROM.h>

// 1. Cấu hình chân kết nối
#define S0 13
#define S1 26
#define S2 25
#define SIG_PIN 35

#define BUTTON1 32
#define BUZZER 2
#define EEPROM_SIZE 64

// 2. Khai báo biến toàn cục (Dùng extern để chia sẻ giữa các file)
extern unsigned int sensorValue[8];
extern unsigned int black_value[8];
extern unsigned int white_value[8];
extern unsigned int compare_value[8];
extern volatile unsigned char sensor;
extern int mode;
extern volatile bool btn1_clicked;

// 3. Khai báo các hàm
void sensor_init(); // Hàm gộp các lệnh setup phần cứng
void setMuxChannel(int channel);
void beep(int timer);
void readEeprom();
bool isBtn1Clicked_ISR();
void read_sensor_74hc4067();

#endif