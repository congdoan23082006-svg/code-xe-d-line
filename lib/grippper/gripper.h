#ifndef GRIPPER_H
#define GRIPPER_H

#include <Arduino.h>

void gripper_init();
void gripper_write(int angle);
void gripper_grab();
void gripper_release();

#endif // GRIPPER_H