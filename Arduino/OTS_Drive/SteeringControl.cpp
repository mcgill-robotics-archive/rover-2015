//
// Created by David Lavoie-Boutin on 15-07-01.
//

#include "SteeringControl.h"
#include "pins.h"
#include <Arduino.h>

//initialize limits on what commands can be sent to servos
int frontServoLowerLimit_cmd = 1000;
int frontServoUpperLimit_cmd = 2000;
int rearServoLowerLimit_cmd = 1000;
int rearServoUpperLimit_cmd = 2000;

int LF_servo_cmd = 0;
int RF_servo_cmd = 0;
int LR_servo_cmd = 0;
int RR_servo_cmd = 0;

Servo LF_servo, RF_servo, LR_servo, RR_servo;


void setWheelAngle(float LF_servo_angle, float RF_servo_angle, float LR_servo_angle, float RR_servo_angle)
{
    //convert angle in degrees to command
    LF_servo_cmd = (int) map((long) LF_servo_angle,0,180,1000,2000);
    RF_servo_cmd = (int) map((long) RF_servo_angle,0,180,1000,2000);
    LR_servo_cmd = (int) map((long) LR_servo_angle,0,180,1000,2000);
    RR_servo_cmd = (int) map((long) RR_servo_angle,0,180,1000,2000);

    //Fine tune servo calibration:
    //should not be greater than like 50 or 100
    //The larger the number, the smaller the range of motion of the servos
    LF_servo_cmd += 0;
    RF_servo_cmd += 0;
    LR_servo_cmd += 0;
    RR_servo_cmd += 0;

    LF_servo_cmd = constrain(LF_servo_cmd, frontServoLowerLimit_cmd, frontServoUpperLimit_cmd);
    RF_servo_cmd = constrain(RF_servo_cmd, frontServoLowerLimit_cmd, frontServoUpperLimit_cmd);
    LR_servo_cmd = constrain(LR_servo_cmd, rearServoLowerLimit_cmd, rearServoUpperLimit_cmd);
    RR_servo_cmd = constrain(RR_servo_cmd, rearServoLowerLimit_cmd, rearServoUpperLimit_cmd);

    //send signals to motors
    LF_servo.writeMicroseconds(LF_servo_cmd);
    RF_servo.writeMicroseconds(RF_servo_cmd);
    LR_servo.writeMicroseconds(LR_servo_cmd + 100);
    RR_servo.writeMicroseconds(RR_servo_cmd);

}

void attachServos()
{
    LF_servo.attach(FL_STEERING_PIN);
    RF_servo.attach(FR_STEERING_PIN);
    LR_servo.attach(BL_STEERING_PIN);
    RR_servo.attach(BR_STEERING_PIN);
}
