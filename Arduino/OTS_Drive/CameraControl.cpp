//
// Created by david on 7/21/15.
//

#include <Arduino.h>
#include "CameraControl.h"
#include "pins.h"


CameraControl::CameraControl() {
    mPanAngle = 0;
    mTiltAngle = 0;

    mPanServo.attach(CAMERA_PAN_SERVO);
    mTittServo.attach(CAMERA_TILT_SERVO);

    update(mPanAngle, mTiltAngle);
}

CameraControl::~CameraControl() {

}

void CameraControl::init() {

}

void CameraControl::update(int pan, int tilt) {

    long panCmd = map(pan, 0, 180, panLowLimit, panHighLimit);
    long tiltCmd = map(tilt, 0, 180, tiltLowLimit, tiltHighLimit);

    panCmd = constrain(panCmd, panLowLimit, panHighLimit);
    tiltCmd = constrain(tiltCmd, tiltLowLimit, tiltHighLimit);

    mPanServo.writeMicroseconds((int) panCmd);
    mTittServo.writeMicroseconds((int) tiltCmd);
}

void CameraControl::handleTwist(const geometry_msgs::Twist &twist) {
    mPanAngle += twist.angular.z;
    mTiltAngle += twist.angular.y;

    update(mPanAngle, mTiltAngle);
}