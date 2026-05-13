#include "encoder.h"

// --- CÁC HẰNG SỐ VẬT LÝ ---
const float WHEEL_RADIUS = 3.25; // cm (Bán kính bánh xe)
const float ENCODER_CPR = 991.0; // Số xung trên 1 vòng quay
const float PI_VAL = 3.14159;

// --- KHAI BÁO CHÂN CHO ENCODER 1 ---
const int enc1PinA = 34;
const int enc1PinB = 39;
volatile long enc1Count = 0;
volatile byte enc1PrevState = 0;

// --- KHAI BÁO CHÂN CHO ENCODER 2 ---
const int enc2PinA = 36;
const int enc2PinB = 27;
volatile long enc2Count = 0;
volatile byte enc2PrevState = 0;

// ==========================================
// HÀM KHỞI TẠO ENCODER
// ==========================================
void initEncoders() {
  pinMode(enc1PinA, INPUT_PULLUP);
  pinMode(enc1PinB, INPUT_PULLUP);
  pinMode(enc2PinA, INPUT_PULLUP);
  pinMode(enc2PinB, INPUT_PULLUP);

  enc1PrevState = (digitalRead(enc1PinA) << 1) | digitalRead(enc1PinB);
  enc2PrevState = (digitalRead(enc2PinA) << 1) | digitalRead(enc2PinB);

  attachInterrupt(digitalPinToInterrupt(enc1PinA), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc1PinB), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2PinA), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2PinB), updateEncoder2, CHANGE);
}

// ==========================================
// HÀM NGẮT (ISR) XỬ LÝ ENCODER 1 (CHẾ ĐỘ x4)
// ==========================================
void IRAM_ATTR updateEncoder1() {
  byte currentState = (digitalRead(enc1PinA) << 1) | digitalRead(enc1PinB);
  byte state = (enc1PrevState << 2) | currentState;

  if (state == 0b0001 || state == 0b0111 || state == 0b1110 ||
      state == 0b1000) {
    enc1Count++;
  } else if (state == 0b0010 || state == 0b1011 || state == 0b1101 ||
             state == 0b0100) {
    enc1Count--;
  }
  enc1PrevState = currentState;
}

// ==========================================
// HÀM NGẮT (ISR) XỬ LÝ ENCODER 2 (CHẾ ĐỘ x4)
// ==========================================
void IRAM_ATTR updateEncoder2() {
  byte currentState = (digitalRead(enc2PinA) << 1) | digitalRead(enc2PinB);
  byte state = (enc2PrevState << 2) | currentState;

  if (state == 0b0001 || state == 0b0111 || state == 0b1110 ||
      state == 0b1000) {
    enc2Count++;
  } else if (state == 0b0010 || state == 0b1011 || state == 0b1101 ||
             state == 0b0100) {
    enc2Count--;
  }
  enc2PrevState = currentState;
}

// ==========================================
// HÀM RESET ENCODER
// ==========================================
void resetEncoders() {
  noInterrupts();
  enc1Count = 0;
  enc2Count = 0;
  interrupts();
}

// ==========================================
// HÀM TÍNH TOÁN QUÃNG ĐƯỜNG
// ==========================================
float calculateDistance() {
  // 1. Lấy giá trị biến đếm (trên ESP32 32-bit, việc đọc biến long 32-bit là atomic nên không cần tắt ngắt)
  long currentTicksLeft = enc1Count;
  long currentTicksRight = enc2Count;

  // 2. Tính quãng đường từng bánh đi được (đơn vị: cm)
  float distanceLeft =
      (currentTicksLeft / ENCODER_CPR) * (2 * PI_VAL * WHEEL_RADIUS);
  float distanceRight =
      (currentTicksRight / ENCODER_CPR) * (2 * PI_VAL * WHEEL_RADIUS);

  // 3. Tính quãng đường trung tâm robot (đơn vị: cm)
  float distanceRobot = (distanceLeft + distanceRight) / 2.0;

  // 4. In kết quả (Comment lại để tránh lag vòng lặp PID)
  // Serial.print("Bánh trái: ");
  // Serial.print(distanceLeft);
  // Serial.print(" cm | Bánh phải: ");
  // Serial.print(distanceRight);
  // Serial.print(" cm | Robot đi được: ");
  // Serial.print(distanceRobot);
  // Serial.println(" cm");

  return distanceRobot;
}