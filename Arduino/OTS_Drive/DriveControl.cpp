//
// Created by David Lavoie-Boutin on 15-07-01.
//

#include <Arduino.h>
#include "DriveControl.h"
#include "pins.h"


void FLsetSpeed(double speed) {
    // cw is gnd
    // ccw is vcc
    if (speed == 0)
    {
        analogWrite(FL_DRIVE_PIN, 0);
    }
    else
    {
        if (speed < 0)
            digitalWrite(FL_DIRECTION_PIN, LOW);
        else
            digitalWrite(FL_DIRECTION_PIN, HIGH);

        analogWrite(FL_DRIVE_PIN, (int) (abs(speed) * 10));
    }
}


void FRsetSpeed(double speed) {
    // cw is gnd
    // ccw is vcc
    if (speed == 0)
    {
        analogWrite(FR_DRIVE_PIN, 0);
    }
    else
    {
        if (speed < 0)
            digitalWrite(FR_DIRECTION_PIN, HIGH);
        else
            digitalWrite(FR_DIRECTION_PIN, LOW);

        analogWrite(FR_DRIVE_PIN, (int) (abs(speed) * 10));
    }
}


void MLsetSpeed(double speed) {
    // cw is gnd
    // ccw is vcc
    if (speed == 0)
    {
        analogWrite(ML_DRIVE_PIN, 0);
    }
    else
    {
        if (speed < 0)
            digitalWrite(ML_DIRECTION_PIN, LOW);
        else
            digitalWrite(ML_DIRECTION_PIN, HIGH);

        analogWrite(ML_DRIVE_PIN, (int) (abs(speed) * 10)); //TODO: Fix over flow error with max steering
    }
}


void MRsetSpeed(double speed) {
    // cw is gnd
    // ccw is vcc
    if (speed == 0)
    {
        analogWrite(MR_DRIVE_PIN, 0);
    }
    else
    {
        if (speed < 0)
            digitalWrite(MR_DIRECTION_PIN, HIGH);
        else
            digitalWrite(MR_DIRECTION_PIN, LOW);

        analogWrite(MR_DRIVE_PIN, (int) (abs(speed) * 10));
    }
}


void BLsetSpeed(double speed) {
    // cw is gnd
    // ccw is vcc
    if (speed == 0)
    {
        analogWrite(BL_DRIVE_PIN, 0);
    }
    else
    {
        if (speed < 0)
            digitalWrite(BL_DIRECTION_PIN, LOW);
        else
            digitalWrite(BL_DIRECTION_PIN, HIGH);

        analogWrite(BL_DRIVE_PIN, (int) (abs(speed) * 10));
    }
}


void BRsetSpeed(double speed) {
    // cw is gnd
    // ccw is vcc
    if (speed == 0)
    {
        analogWrite(BR_DRIVE_PIN, 0);
    }
    else
    {
        if (speed < 0)
            digitalWrite(BR_DIRECTION_PIN, HIGH);
        else
            digitalWrite(BR_DIRECTION_PIN, LOW);

        analogWrite(BR_DRIVE_PIN, (int) (abs(speed) * 10));
    }
}

void enableMotors()
{
    digitalWrite(FL_ENABLE_PIN, HIGH);
    digitalWrite(FR_ENABLE_PIN, HIGH);
    digitalWrite(ML_ENABLE_PIN, HIGH);
    digitalWrite(MR_ENABLE_PIN, HIGH);
    digitalWrite(BL_ENABLE_PIN, HIGH);
    digitalWrite(BR_ENABLE_PIN, HIGH);

}

void disableMotors()
{
    digitalWrite(FL_ENABLE_PIN, LOW);
    digitalWrite(FR_ENABLE_PIN, LOW);
    digitalWrite(ML_ENABLE_PIN, LOW);
    digitalWrite(MR_ENABLE_PIN, LOW);
    digitalWrite(BL_ENABLE_PIN, LOW);
    digitalWrite(BR_ENABLE_PIN, LOW);
}
