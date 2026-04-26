#include "Mission1.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "Motion.h"

BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool isMission1Active = false;

#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE Connected!");
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE Disconnected!");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        char cmd = rxValue[0];
        Serial.print("Received Value: ");
        Serial.println(cmd);

        if (!isMission1Active) return; // Chỉ điều khiển khi đang ở Mode 4

        int speed = 3000; // Tốc độ điều khiển bằng tay (chỉnh từ 0-4095)
        switch(cmd) {
            case 'F': speed_run(speed, speed); break;
            case 'B': speed_run(-speed, -speed); break;
            case 'L': speed_run(-speed, speed); break;
            case 'R': speed_run(speed, -speed); break;
            case 'S': speed_run(0, 0); break;
            case 'G': 
                Serial.println("Action: GAP (Code dieu khien Servo tai day)"); 
                // Thêm code điều khiển động cơ servo gắp vật
                break;
            case 'D': 
                Serial.println("Action: THA (Code dieu khien Servo tai day)"); 
                // Thêm code điều khiển động cơ servo thả vật
                break;
            default: speed_run(0, 0); break;
        }
      }
    }
};

void mission1_init() {
    BLEDevice::init("ROBOT_2026");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_WRITE | 
                      BLECharacteristic::PROPERTY_WRITE_NR
                    );

    pTxCharacteristic->setCallbacks(new MyCallbacks());
    pTxCharacteristic->addDescriptor(new BLE2902());

    pService->start();
    
    // Thiết lập quảng bá BLE
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  
    pAdvertising->setMinPreferred(0x12);
}

void mission1_activate() {
    isMission1Active = true;
    BLEDevice::startAdvertising();
    Serial.println("Mode 4: BLE Controller Active");
}

void mission1_deactivate() {
    isMission1Active = false;
    speed_run(0, 0);
    // Dừng quảng bá khi thoát khỏi Mode 4
    BLEDevice::stopAdvertising();
}

void mission1_loop() {
    // Quản lý trạng thái đứt kết nối
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // Cho Bluetooth stack kịp phục hồi
        pServer->startAdvertising(); // Bật lại quảng bá để kết nối lại
        Serial.println("Restarting BLE Advertising...");
        oldDeviceConnected = deviceConnected;
        speed_run(0, 0); // Dừng xe an toàn
    }
    
    // Kết nối lại thành công
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
}
