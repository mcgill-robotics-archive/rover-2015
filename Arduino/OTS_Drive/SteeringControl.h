//
// Created by David Lavoie-Boutin on 15-07-01.
//

#ifndef ARDUINO_TEST_STEERINGCONTROL_H
#define ARDUINO_TEST_STEERINGCONTROL_H

#include <Servo/Servo.h>


//initialize limits on what commands can be sent to servos

void attachServos();
void setWheelAngle(float LF_servo_angle, float RF_servo_angle, float LR_servo_angle, float RR_servo_angle);


#endif //ARDUINO_TEST_STEERINGCONTROL_H
