#ifndef DISPLAY_OLED_H
#define DISPLAY_OLED_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Định nghĩa kích thước màn hình OLED (thường là 128x64)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Chân Reset của OLED (dùng -1 nếu màn hình không có chân reset chia sẻ)
#define OLED_RESET    -1
// Địa chỉ I2C của màn hình (thường là 0x3C, một số loại là 0x3D)
#define SCREEN_ADDRESS 0x3C 

// Khai báo đối tượng display để có thể dùng ở các file khác nếu cần
extern Adafruit_SSD1306 display;
extern bool isOledOk;

// Khai báo nguyên mẫu hàm
void initOLED();
void updateOLED(unsigned int *sensorVals);

#endif