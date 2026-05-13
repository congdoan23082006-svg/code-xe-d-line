#ifndef MISSION1_H
#define MISSION1_H

#include <Arduino.h>

extern bool isMission1Active; // Cho phép main.cpp truy cập trực tiếp

void mission1_init();
void mission1_activate();
void mission1_deactivate();
void mission1_loop();

#endif
