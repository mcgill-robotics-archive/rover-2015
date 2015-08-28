//
// Created by David Lavoie-Boutin on 15-08-28.
//

#ifndef ARDUINO_TEST_LIDARTABLE_H
#define ARDUINO_TEST_LIDARTABLE_H

#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <tf/transform_broadcaster.h>

// Enable pins. Underscore prefix indicates that pin is active-low.
#define _DRE_UNUSED 32   // Encoder port 3

void initROS();
void setupSPI();
float readEncoder();
float readEncoderRAD();
void sendTransform();
geometry_msgs::Quaternion fromPRY(float pitch, float roll, float yaw);

#endif //ARDUINO_TEST_LIDARTABLE_H
