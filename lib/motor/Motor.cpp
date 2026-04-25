#include "MotorControl.h"

void initMotors() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Setup PWM (LEDc) cho ESP32
  ledcSetup(pwmChannelLeft, pwmFreq, pwmResolution);
  ledcSetup(pwmChannelRight, pwmFreq, pwmResolution);
  ledcAttachPin(ENA, pwmChannelLeft);
  ledcAttachPin(ENB, pwmChannelRight);
}

void motorControl(int speedLeft, int speedRight) {
  // --- BÁNH TRÁI ---
  if (speedLeft > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (speedLeft < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  ledcWrite(pwmChannelLeft, abs(speedLeft)); 

  // --- BÁNH PHẢI ---
  if (speedRight > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (speedRight < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  ledcWrite(pwmChannelRight, abs(speedRight));
}