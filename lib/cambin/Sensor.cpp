#include "Sensor.h"

// Khởi tạo các biến toàn cục
unsigned int sensorValue[8];
unsigned int black_value[8];
unsigned int white_value[8];
unsigned int compare_value[8];
volatile unsigned char sensor;
int mode = 2; // Khởi động lên mặc định là Mode 2

// Biến cho ngắt nút bấm
volatile bool btn1_clicked = false;
unsigned long lastBtn1Time = 0;

// Hàm ngắt (ISR) cho nút 1
void IRAM_ATTR handleBtn1() {
  unsigned long now = millis();
  if (now - lastBtn1Time > 300) { // Debounce 300ms
    btn1_clicked = true;
    lastBtn1Time = now;
  }
}

// Khởi tạo các chân GPIO và EEPROM
void sensor_init() {

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);

  // Gắn ngắt cho nút bấm
  attachInterrupt(digitalPinToInterrupt(BUTTON1), handleBtn1, FALLING);

  EEPROM.begin(EEPROM_SIZE);

  readEeprom();

  beep(100);
}

void setMuxChannel(int channel) {
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
}

void beep(int timer) {
  digitalWrite(BUZZER, 1);

  delay(timer);
  digitalWrite(BUZZER, 0);
}

void readEeprom() {
  for (int i = 0; i < 8; i++) {
    black_value[i] = (EEPROM.read(i * 2) << 8) | EEPROM.read((i * 2) + 1);
  }
  for (int i = 0; i < 8; i++) {
    white_value[i] =
        (EEPROM.read(16 + (2 * i)) << 8) | EEPROM.read(17 + (2 * i));
  }
  for (int i = 0; i < 8; i++) {
    compare_value[i] = ((black_value[i] + white_value[i]) / 2);
    if (compare_value[i] == 0)
      compare_value[i] = 2000;
  }
}

bool isBtn1Clicked_ISR() {
  if (btn1_clicked) {
    btn1_clicked = false; // Reset cờ sau khi đã xử lý
    return true;
  }
  return false;
}

void read_sensor_74hc4067() {
  unsigned char temp = 0;
  for (int j = 0; j < 8; j++) {
    setMuxChannel(j);
    delayMicroseconds(10);

    // ĐỌC BỎ (DUMMY READ): Xả điện dung của ADC ESP32 để chống nhiễu chéo kênh
    analogRead(SIG_PIN);
    delayMicroseconds(5);

    sensorValue[j] = 4095 - analogRead(SIG_PIN);

    temp = temp << 1;
    // Quy ước cuối cùng: Vạch ĐEN = 1, Nền TRẮNG = 0
    // Vì sensorValue ở đây đã là (4095 - raw), ta so sánh với ngưỡng so sánh.
    if (sensorValue[j] < compare_value[j]) {
      // ĐEN: set bit (Dành cho line đen)
      temp |= 0x01;
    } else {
      // TRẮNG: clear bit
      temp &= 0xFE;
    }
  }
  sensor = temp;
}
