#include "PID_Control.h"
#include "SensorMux.h"

float Kp = 0.2;  // Bộ thông số ổn định cho xe 1kg
float Ki = 0.0;   
float Kd = 4.0;  

int baseSpeed = 220; 

float error = 0;
static float pError = 0, integral = 0, derivative = 0;

void resetPID() {
  error = 0;
  pError = 0;
  integral = 0;
  derivative = 0;
}

void runPID(int &speedLeft, int &speedRight) {
  int position = readLinePosition();
  error = position - 3500; // Tâm chuẩn 3500 (Dải error từ -3500 đến +3500)

  if (error == 0) integral = 0; 
  else {
    integral += error;
    integral = constrain(integral, -10000, 10000); 
  }

  derivative = error - pError;
  pError = error;

  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Giảm tốc khi cua để tăng lực Spin Turn
  float speedReductionFactor = 1.0 - (abs(error) / 3500.0); 
  if (speedReductionFactor < 0) speedReductionFactor = 0;
  int currentBaseSpeed = baseSpeed * speedReductionFactor;

  speedLeft  = currentBaseSpeed + output;
  speedRight = currentBaseSpeed - output;

  // SPIN TURN BOOST: Đối xứng kẹp ở mốc 3000
  // Nếu quét vạch văng ra cực mép (Error quá -3000 hoặc quá +3000) -> ÉP xoay kịch kim
  if (error <= -3000) { speedLeft = -255; speedRight = 255; }
  if (error >=  3000) { speedLeft =  255; speedRight = -255; }

  // Bù lực Deadband 60
  int minPWM = 60;
  if (speedLeft > 0 && speedLeft < minPWM) speedLeft = minPWM;
  if (speedLeft < 0 && speedLeft > -minPWM) speedLeft = -minPWM;
  if (speedRight > 0 && speedRight < minPWM) speedRight = minPWM;
  if (speedRight < 0 && speedRight > -minPWM) speedRight = -minPWM;

  speedLeft  = constrain(speedLeft, -255, 255);
  speedRight = constrain(speedRight, -255, 255);
}