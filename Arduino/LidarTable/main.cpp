//
// Created by David Lavoie-Boutin on 15-08-28.
//

#include "LidarTable.h"
#include "Servo.h"

#define TOP_ANGLE 90
#define BOTTOM_ANGLE 20

#define PITCH_UP_SPEED 93
#define PITCH_DOWN_SPEED 87

#define SERVO_PIN 9
Servo tiltServo;

void setup()
{
    initROS();
    setupSPI();

    tiltServo.attach(SERVO_PIN);
    tiltServo.write(90);
}

void loop()
{
    tiltServo.write(PITCH_DOWN_SPEED);
    while (readEncoder() > BOTTOM_ANGLE)
    {
        sendTransform();
    }

    tiltServo.write(PITCH_UP_SPEED);
    while (readEncoder() < TOP_ANGLE)
    {
        sendTransform();
    }
}
