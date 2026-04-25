#include "BLEControl.h"
#include "PID_Control.h"
#include <EEPROM.h>

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE Client Connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE Client Disconnected");
      // Restart advertising so it can be found again
      pServer->startAdvertising();
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
        Serial.print("Received BLE: ");
        Serial.println(value.c_str());

        // Cú pháp lệnh: P0.20,I0.00,D4.00,S220
        char buf[64];
        strncpy(buf, value.c_str(), sizeof(buf));
        
        char* p = strtok(buf, ",");
        while (p != NULL) {
          if (p[0] == 'P') Kp = atof(p + 1);
          if (p[0] == 'I') Ki = atof(p + 1);
          if (p[0] == 'D') Kd = atof(p + 1);
          if (p[0] == 'S') baseSpeed = atoi(p + 1);
          p = strtok(NULL, ",");
        }

        // Lưu vào EEPROM ngay lập tức
        EEPROM.put(32, Kp);
        EEPROM.put(36, Ki);
        EEPROM.put(40, Kd);
        EEPROM.put(44, baseSpeed);
        EEPROM.commit();
        
        Serial.printf("Updated -> Kp:%.2f Ki:%.2f Kd:%.2f Speed:%d\n", Kp, Ki, Kd, baseSpeed);
      }
    }
};

void initBLEControl() {
  BLEDevice::init("Robot-PID-BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE Advertising Started...");
}

void handleBLE() {
    // Không cần xử lý liên tục trong loop như WebServer, 
    // BLE hoạt động dựa trên Callbacks.
}
