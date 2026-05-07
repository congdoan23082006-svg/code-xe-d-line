#include "Button.h"

// Định nghĩa chân thực tế (Bạn có thể đổi số chân tùy theo ESP32/Arduino của bạn)
const int BTN1_PIN = 32; // Ví dụ dùng nút BOOT trên ESP32
const int BTN2_PIN = 33; 

// Biến lưu trạng thái trước đó của nút (dùng cho hàm Click)
static bool lastBtn1State = HIGH;
static bool lastBtn2State = HIGH;

// Hàm khởi tạo
void initButtons() {
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
}

// Hàm 1: Kiểm tra xem nút có đang BỊ GIỮ HAY KHÔNG
// Trả về true nếu đang nhấn
bool isBtn1Pressed() {
  return digitalRead(BTN1_PIN) == LOW; 
}

bool isBtn2Pressed() {
  return digitalRead(BTN2_PIN) == LOW;
}

// Hàm 2: Nhận diện 1 lần "CLICK" (Nhấn vào và thả ra)
// Rất hữu ích khi làm menu cho OLED để số không bị nhảy liên tục
bool isBtn1Clicked() {
  bool currentState = digitalRead(BTN1_PIN);
  bool clicked = false;
  
  // Nếu trạng thái thay đổi từ HIGH (chưa nhấn) sang LOW (đang nhấn)
  if (lastBtn1State == HIGH && currentState == LOW) {
    delay(20); // Delay nhỏ để chống dội phím (debounce)
    if (digitalRead(BTN1_PIN) == LOW) {
      clicked = true;
    }
  }
  lastBtn1State = currentState;
  return clicked;
}

bool isBtn2Clicked() {
  bool currentState = digitalRead(BTN2_PIN);
  bool clicked = false;
  
  if (lastBtn2State == HIGH && currentState == LOW) {
    delay(20); // Chống dội
    if (digitalRead(BTN2_PIN) == LOW) {
      clicked = true;
    }
  }
  lastBtn2State = currentState;
  return clicked;
}