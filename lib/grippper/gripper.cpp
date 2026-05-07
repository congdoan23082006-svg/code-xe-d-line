#include "Gripper.h"
#include "driver/mcpwm.h"

const int servoPin = 26; 

void gripper_init() {
  // SỬ DỤNG MCPWM (Bộ băm xung chuyên dụng cho Động Cơ/Servo) thay vì LEDC.
  // Bộ này phần cứng hoàn toàn độc lập, đảm bảo không có bất kỳ xung đột nào với hệ thống.
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, servoPin);
  
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 50;  // Tần số 50Hz cho Servo
  pwm_config.cmpr_a = 0;      
  pwm_config.cmpr_b = 0;      
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  
  // Khởi tạo MCPWM (Dùng Timer riêng của MCPWM)
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  
  gripper_write(0);
}

void gripper_write(int angle) {
  if (angle >= 0 && angle <= 90) {
    // 0 độ = 500us, 180 độ = 2400us
    float pulse_us = map(angle, 0, 180, 500, 2400);
    // Tính % Duty Cycle (1 chu kỳ 50Hz = 20000us)
    float duty_percent = (pulse_us / 20000.0) * 100.0;
    
    // Xuất xung PWM
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_percent);
  }
}

void gripper_grab() {
  gripper_write(90);
  Serial.println("=> Da giu Servo o goc: 90 do (Dung MCPWM)");
}

void gripper_release() {
  gripper_write(0);
  Serial.println("=> Da giu Servo o goc: 0 do (Dung MCPWM)");
}
