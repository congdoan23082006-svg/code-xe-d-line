#include "DisplayOLED.h"

// Khởi tạo đối tượng màn hình
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void initOLED() {
  // Khởi tạo chuẩn giao tiếp I2C. 
  // Lưu ý: ESP32 mặc định dùng SDA = 21, SCL = 22.
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Lỗi: Không tìm thấy màn hình OLED SSD1306"));
    for(;;); // Khóa vòng lặp nếu lỗi phần cứng
  }
  
  // Xóa bộ đệm và hiển thị thông báo khởi động
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 25);
  display.println("OLED Ready!");
  display.display();
  
  delay(1000); // Dừng 1 giây để đọc thông báo
}

void updateOLED(unsigned int *sensorVals) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Vòng lặp chia 8 mắt cảm biến thành 2 cột, 4 hàng
  for (int i = 0; i < 8; i++) {
    // Nếu i từ 0-3 (cột trái, tọa độ x=0)
    // Nếu i từ 4-7 (cột phải, tọa độ x=64 - giữa màn hình)
    int col = (i < 4) ? 0 : 64; 
    
    // Tính toán hàng (y): mỗi hàng cách nhau 15 pixel
    int row = (i % 4) * 15;     

    display.setCursor(col, row);
    display.print("S");
    display.print(i);
    display.print(": ");
    
    // In giá trị, có thể căn lề (padding) thêm khoảng trắng nếu số dao động nhảy chữ
    if (sensorVals[i] < 1000) display.print(" ");
    if (sensorVals[i] < 100) display.print(" ");
    if (sensorVals[i] < 10) display.print(" ");
    display.print(sensorVals[i]);
  }

  // Đẩy dữ liệu từ RAM ra màn hình
  display.display();
}