//
// Created by david on 7/22/15.
//

#ifndef ARDUINO_TEST_CAMERA_H
#define ARDUINO_TEST_CAMERA_H

#include <geometry_msgs/Twist.h>
#include "pins.h"

void initCameraTable();
void callbackCamera(const geometry_msgs::Twist & twist);
void update(float pan, float tilt);

#endif //ARDUINO_TEST_CAMERA_H
