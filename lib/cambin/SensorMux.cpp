#include "SensorMux.h"

int sensorMin[8] = {500, 500, 500, 500, 500, 500, 500, 500}; 
int sensorMax[8] = {3500, 3500, 3500, 3500, 3500, 3500, 3500, 3500};
int thresholdVal[8];
int sensorValues[8];

static int lastPosition = 3500; 

int readMux(int channel) {
  digitalWrite(MUX_S0, bitRead(channel, 0));
  digitalWrite(MUX_S1, bitRead(channel, 1));
  digitalWrite(MUX_S2, bitRead(channel, 2));
  delayMicroseconds(10);
  long sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(MUX_SIG);
    delayMicroseconds(5);
  }
  return sum / 8;
}

void initSensors() {
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_SIG, INPUT);
  for(int i=0; i<8; i++) thresholdVal[i] = 2048;
}

void initCalibValues() {
  for (int i = 0; i < 8; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }
}

void updateCalib() {
  for (int i = 0; i < 8; i++) {
    int val = readMux(i);
    sensorValues[i] = val; 
    if (val > sensorMax[i]) sensorMax[i] = val;
    if (val < sensorMin[i]) sensorMin[i] = val;
    thresholdVal[i] = (sensorMin[i] + sensorMax[i]) / 2;
  }
}

int readLinePosition() {
  long sumWeights = 0;
  long sumValues = 0;
  bool onLine = false;

  for (int i = 0; i < 8; i++) {
    int rawValue = readMux(i); 
    sensorValues[i] = rawValue;
    
    int value = map(rawValue, sensorMin[i], sensorMax[i], 0, 1000);
    value = constrain(value, 0, 1000);

    if (value > 200) onLine = true; 

    if (value > 50) {
      sumWeights += (long)value * (i * 1000); 
      sumValues += value;
    }
  }

  if (!onLine || sumValues == 0) {
    if (lastPosition < 2000) return 0;      
    if (lastPosition > 5000) return 7000; 
    return lastPosition; 
  }

  int position = sumWeights / sumValues;
  lastPosition = position; 
  return position;
}