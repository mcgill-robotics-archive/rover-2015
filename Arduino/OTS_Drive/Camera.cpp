//
// Created by david on 7/22/15.
//

#include <Servo/Servo.h>
#include <Arduino.h>
#include "Camera.h"


Servo tiltServo, panServo;

float mPanAngle;
float mTiltAngle;

const int panLowLimit = 70;
const int panHighLimit = 110;

const int tiltLowLimit = 45;
const int tiltHighLimit = 170;

void initCameraTable()
{
    tiltServo.attach(CAMERA_TILT_SERVO);
    panServo.attach(CAMERA_PAN_SERVO);
    mTiltAngle = 90;
    mPanAngle = 90;


    update(mPanAngle, mTiltAngle);
}

void callbackCamera(const geometry_msgs::Twist & twist)
{
    mTiltAngle += twist.angular.y;
    float panCmd = mPanAngle + (twist.angular.z * 15);

    update(panCmd, mTiltAngle);
}

void update(float pan, float tilt) {

    int panCmd = (int) constrain(pan, panLowLimit, panHighLimit);
    int tiltCmd = (int) constrain( tilt, tiltLowLimit, tiltHighLimit); // TODO: fix not constrained

     panServo.write(panCmd);
     tiltServo.write(tiltCmd);
}