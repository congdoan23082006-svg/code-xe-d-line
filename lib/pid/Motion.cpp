#include "Motion.h"
#include "Sensor.h" // Chứa mảng sensor và sensorValue

// Khởi tạo các biến
int servoPwm = 0;
int vitri = 0;
int baseSpeed = 4000;
volatile int calPID = 0;
volatile long lastPos = 0;
volatile float Kp = 1;
volatile float Ki = 0;
volatile float Kd = 13;
volatile int isLeft = 0;
volatile int isRight = 0;
volatile long posPID = 0;

void motion_init() {
  // Cài đặt các chân IN làm OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Mặc định cho xe đứng im
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Cài đặt băm xung PWM cho chân ENA và ENB
  ledcSetup(ENA_CH, 2000, 12); 
  ledcAttachPin(ENA, ENA_CH);
  
  ledcSetup(ENB_CH, 2000, 12);
  ledcAttachPin(ENB, ENB_CH);

  speed_run(0, 0); 
}

// --------------------------------------------------------
// HÀM ĐIỀU KHIỂN CHIỀU VÀ TỐC ĐỘ CHUẨN L298N
// --------------------------------------------------------
void speed_run(int speedDC_left, int speedDC_right) {
  // 1. Giới hạn dải tốc độ không vượt quá 4095
  if (speedDC_left < -4095) speedDC_left = -4095;
  if (speedDC_right < -4095) speedDC_right = -4095;
  if (speedDC_left > 4095) speedDC_left = 4095;
  if (speedDC_right > 4095) speedDC_right = 4095;

  // 2. Điều khiển bánh TRÁI (Motor A)
  if (speedDC_left == 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);   // Phanh động cơ
    ledcWrite(ENA_CH, 0);
  } 
  else if (speedDC_left > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);   // Chạy tới
    ledcWrite(ENA_CH, speedDC_left);
  } 
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);  // Chạy lùi (khi phanh gắt)
    ledcWrite(ENA_CH, -speedDC_left); // Lấy số dương của tốc độ
  }

  // 3. Điều khiển bánh PHẢI (Motor B)
  if (speedDC_right == 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);   // Phanh động cơ
    ledcWrite(ENB_CH, 0);
  } 
  else if (speedDC_right > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);   // Chạy tới
    ledcWrite(ENB_CH, speedDC_right);
  } 
  else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);  // Chạy lùi (khi phanh gắt)
    ledcWrite(ENB_CH, -speedDC_right); // Lấy số dương của tốc độ
  }
}

// ========================================================
// CÁC HÀM BÊN DƯỚI GIỮ NGUYÊN 100% LOGIC CỦA BẠN
// ========================================================

void handleAndSpeed(int angle, int speed1) {
  int speedLeft;
  int speedRight;
  
  if ((speed1 + angle) > 4095) speed1 = 4095 - angle;
  if ((speed1 - angle) > 4095) speed1 = 4095 + angle;
  
  speedLeft = (speed1 + angle);
  speedRight = (speed1 - angle);
  
  speed_run(speedLeft, speedRight);
}

void PID() {
  if (calPID == 1) {
    long sum = 0;
    long avg = 0;
    long i, iP, iD;
    long iRet;

    for (int j = 0; j < 8; j = j + 1) {
      avg = (avg + (sensorValue[j]) * (j * 3000));
      sum = sum + sensorValue[j];
    }

    if (sum == 0) sum = 1;

    if (isLeft == 1) i = ((avg / sum) - 15500);
    else if (isRight == 1) i = ((avg / sum) - 5500);
    else i = ((avg / sum) - 10500);
    
    posPID = i; 
    
    static long sumError = 0;
    sumError += i;
    
    iP = Kp * i;
    long iI = Ki * sumError;
    iD = Kd * (lastPos - i);
    iRet = (iP + iI - iD);
    
    if ((iRet < -12000)) iRet = 0;
    
    servoPwm = iRet / 5;
    lastPos = i;
    calPID = 0;
  }
}

void runforwardline(int tocdo) {
  PID();

  switch (sensor) {
    case 0b00011000:
    case 0b00111100:
      handleAndSpeed(servoPwm, tocdo);
      vitri = 0;
      break;

    case 0b00011100:
    case 0b00001000:
      handleAndSpeed(servoPwm, tocdo);
      vitri = 1;
      break;

    case 0b00010000:
    case 0b00111000:
      if (vitri >= 7) { speed_run(1200, 0); break; } 
      else { handleAndSpeed(servoPwm, tocdo); }
      vitri = -1;
      break;

    case 0b00001100:
    case 0b00011110:
      if (vitri < -7) { speed_run(0, 1200); break; } 
      else { handleAndSpeed(servoPwm, baseSpeed); vitri = 2; }
      break;

    case 0b00000100:
    case 0b00001110:
      if (vitri < -7) { speed_run(0, 1200); break; }
      else { handleAndSpeed(servoPwm, (baseSpeed * 70 / 100)); vitri = 3; }
      break;

    case 0b00110000:
    case 0b01111000:
      if (vitri >= 7) { speed_run(1200, 0); break; } 
      else { handleAndSpeed(servoPwm, baseSpeed); vitri = -2; }
      break;

    case 0b00100000:
    case 0b01110000:
      if (vitri >= 7) { speed_run(1200, 0); break; } 
      else { handleAndSpeed(servoPwm, (baseSpeed * 70 / 100)); vitri = -3; }
      break;

    case 0b00000110:
    case 0b00001111:
    case 0b00000010:
    case 0b00000111:
    case 0b00000011:
    case 0b00000001:
      if (vitri < -7) { speed_run(0, 1200); break; } 
      else { handleAndSpeed(servoPwm, (baseSpeed * 50 / 100)); vitri = 7; }
      break;

    case 0b01100000:
    case 0b11110000:
    case 0b01000000:
    case 0b11100000:
    case 0b11000000:
    case 0b10000000:
      if (vitri >= 7) { speed_run(1200, 0); break; } 
      else { handleAndSpeed(servoPwm, (baseSpeed * 50 / 100)); vitri = -7; }
      break;

    case 0b00000000:
      if (vitri <= -7) { speed_run(0, 1200); vitri = -8; } 
      else if (vitri >= 7) { speed_run(1200, 0); vitri = 8; } 
      else speed_run(0, 0);
      break;

    default:
      handleAndSpeed(0, 0);
      break;
  }
}