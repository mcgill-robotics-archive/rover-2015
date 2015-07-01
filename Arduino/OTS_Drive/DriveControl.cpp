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
        //   digitalWrite(FL_ENABLE_PIN, LOW);
    }
    else
    {
        digitalWrite(FL_ENABLE_PIN, HIGH);
        if (speed < 0)
            digitalWrite(FL_DIRECTION_PIN, HIGH); // confirm direction polarity
        else
            digitalWrite(FL_DIRECTION_PIN, LOW);

        analogWrite(FL_DRIVE_PIN, (int) (abs(speed) * 10));
    }
}


void FRsetSpeed(double speed) {
    // cw is gnd
    // ccw is vcc
    if (speed == 0)
    {
        analogWrite(FR_DRIVE_PIN, 0);
        //    digitalWrite(FR_ENABLE_PIN, LOW);
    }
    else
    {
        digitalWrite(FR_ENABLE_PIN, HIGH);
        if (speed < 0)
            digitalWrite(FR_DIRECTION_PIN, HIGH); // confirm direction polarity
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
        //    digitalWrite(ML_ENABLE_PIN, LOW);
    }
    else
    {
        digitalWrite(ML_ENABLE_PIN, HIGH);
        if (speed < 0)
            digitalWrite(ML_DIRECTION_PIN, HIGH); // confirm direction polarity
        else
            digitalWrite(ML_DIRECTION_PIN, LOW);

        analogWrite(ML_DRIVE_PIN, (int) (abs(speed) * 10));
    }
}


void MRsetSpeed(double speed) {
    // cw is gnd
    // ccw is vcc
    if (speed == 0)
    {
        analogWrite(MR_DRIVE_PIN, 0);
        //    digitalWrite(MR_ENABLE_PIN, LOW);
    }
    else
    {
        digitalWrite(MR_ENABLE_PIN, HIGH);
        if (speed < 0)
            digitalWrite(MR_DIRECTION_PIN, HIGH); // confirm direction polarity
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
        //    digitalWrite(BL_ENABLE_PIN, LOW);
    }
    else
    {
        digitalWrite(BL_ENABLE_PIN, HIGH);
        if (speed < 0)
            digitalWrite(BL_DIRECTION_PIN, HIGH); // confirm direction polarity
        else
            digitalWrite(BL_DIRECTION_PIN, LOW);

        analogWrite(BL_DRIVE_PIN, (int) (abs(speed) * 10));
    }
}


void BRsetSpeed(double speed) {
    // cw is gnd
    // ccw is vcc
    if (speed == 0)
    {
        analogWrite(BR_DRIVE_PIN, 0);
        //   digitalWrite(BR_ENABLE_PIN, LOW);
    }
    else
    {
        digitalWrite(BR_ENABLE_PIN, HIGH);
        if (speed < 0)
            digitalWrite(BR_DIRECTION_PIN, HIGH); // confirm direction polarity
        else
            digitalWrite(BR_DIRECTION_PIN, LOW);

        analogWrite(BR_DRIVE_PIN, (int) (abs(speed) * 10));
    }
}