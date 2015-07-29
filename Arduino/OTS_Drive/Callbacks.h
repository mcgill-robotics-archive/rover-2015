//
// Created by David Lavoie-Boutin on 15-07-26.
//

#ifndef ARDUINO_TEST_CALLBACKS_H
#define ARDUINO_TEST_CALLBACKS_H

static unsigned long lastReset = 0;

float radToDeg(float rad);
void driveCallback( const control_systems::SetPoints& setPoints );
void mcMode(const rover_msgs::MotorControllerMode& msg);
void callbackMoving( const std_msgs::Bool& boolean);
void callbackResetWatchdog(const rover_msgs::ResetWatchDog::Request & request, rover_msgs::ResetWatchDog::Response & response);

#endif //ARDUINO_TEST_CALLBACKS_H
