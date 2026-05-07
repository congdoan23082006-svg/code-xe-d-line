#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

// Khai báo chân cho 2 nút bấm
// Dùng 'extern' để định nghĩa giá trị thực tế ở file .cpp
extern const int BTN1_PIN;
extern const int BTN2_PIN;

// Khai báo các hàm
void initButtons();
bool isBtn1Pressed();
bool isBtn2Pressed();

// (Tùy chọn) Hàm phát hiện "nhấn nhả" để tránh bị chạy lệnh liên tục trong loop
bool isBtn1Clicked();
bool isBtn2Clicked();

#endif