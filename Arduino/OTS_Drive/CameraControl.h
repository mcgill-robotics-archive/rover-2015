//
// Created by david on 7/21/15.
//

#ifndef ARDUINO_TEST_CAMERACONTROL_H
#define ARDUINO_TEST_CAMERACONTROL_H

#include "Servo.h"
#include "geometry_msgs/Twist.h"

class CameraControl {
public:
    CameraControl();
    ~CameraControl();

    void init();
    void update(int pan, int tilt);

    void handleTwist(const geometry_msgs::Twist& twist);

    const static int panLowLimit = 1000;
    const static int panHighLimit = 2000;

    const static int tiltLowLimit = 1000;
    const static int tiltHighLimit = 2000;

private:
    Servo mPanServo;
    Servo mTittServo;

    int mPanAngle;
    int mTiltAngle;

};


#endif //ARDUINO_TEST_CAMERACONTROL_H

