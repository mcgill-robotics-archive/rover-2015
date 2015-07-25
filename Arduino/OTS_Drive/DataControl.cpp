//
// Created by David Lavoie-Boutin on 15-07-01.
//

#include <Arduino.h>
#include <ros.h>
#include "DataControl.h"
#include "pins.h"

void data::setModeOpenLoop()
{
    digitalWrite(FL_DATA1_PIN, LOW);
    digitalWrite(FL_DATA2_PIN, LOW);
    digitalWrite(FR_DATA1_PIN, LOW);
    digitalWrite(FR_DATA2_PIN, LOW);
    digitalWrite(ML_DATA1_PIN, LOW);
    digitalWrite(ML_DATA2_PIN, LOW);
    digitalWrite(MR_DATA1_PIN, LOW);
    digitalWrite(MR_DATA2_PIN, LOW);
    digitalWrite(BL_DATA1_PIN, LOW);
    digitalWrite(BL_DATA2_PIN, LOW);
    digitalWrite(BR_DATA1_PIN, LOW);
    digitalWrite(BR_DATA2_PIN, LOW);
}

void data::setModeLowSpeed()
{
    digitalWrite(FL_DATA1_PIN, HIGH);
    digitalWrite(FL_DATA2_PIN, LOW);
    digitalWrite(FR_DATA1_PIN, HIGH);
    digitalWrite(FR_DATA2_PIN, LOW);
    digitalWrite(ML_DATA1_PIN, HIGH);
    digitalWrite(ML_DATA2_PIN, LOW);
    digitalWrite(MR_DATA1_PIN, HIGH);
    digitalWrite(MR_DATA2_PIN, LOW);
    digitalWrite(BL_DATA1_PIN, HIGH);
    digitalWrite(BL_DATA2_PIN, LOW);
    digitalWrite(BR_DATA1_PIN, HIGH);
    digitalWrite(BR_DATA2_PIN, LOW);
}

void data::setModeMediumSpeed()
{
    digitalWrite(FL_DATA1_PIN, LOW);
    digitalWrite(FL_DATA2_PIN, HIGH);
    digitalWrite(FR_DATA1_PIN, LOW);
    digitalWrite(FR_DATA2_PIN, HIGH);
    digitalWrite(ML_DATA1_PIN, LOW);
    digitalWrite(ML_DATA2_PIN, HIGH);
    digitalWrite(MR_DATA1_PIN, LOW);
    digitalWrite(MR_DATA2_PIN, HIGH);
    digitalWrite(BL_DATA1_PIN, LOW);
    digitalWrite(BL_DATA2_PIN, HIGH);
    digitalWrite(BR_DATA1_PIN, LOW);
    digitalWrite(BR_DATA2_PIN, HIGH);
}

void data::setModeHighSpeed()
{
    digitalWrite(FL_DATA1_PIN, HIGH);
    digitalWrite(FL_DATA2_PIN, HIGH);
    digitalWrite(FR_DATA1_PIN, HIGH);
    digitalWrite(FR_DATA2_PIN, HIGH);
    digitalWrite(ML_DATA1_PIN, HIGH);
    digitalWrite(ML_DATA2_PIN, HIGH);
    digitalWrite(MR_DATA1_PIN, HIGH);
    digitalWrite(MR_DATA2_PIN, HIGH);
    digitalWrite(BL_DATA1_PIN, HIGH);
    digitalWrite(BL_DATA2_PIN, HIGH);
    digitalWrite(BR_DATA1_PIN, HIGH);
    digitalWrite(BR_DATA2_PIN, HIGH);
}

void data::sendMotorStatus(ros::Publisher &publisher) {
    rover_msgs::MotorStatus message;
    message.ok_length = 6;

    bool status[] = {
            (bool) digitalRead(FL_READY_PIN),
            (bool) digitalRead(FR_READY_PIN),
            (bool) digitalRead(ML_READY_PIN),
            (bool) digitalRead(MR_READY_PIN),
            (bool) digitalRead(BL_READY_PIN),
            (bool) digitalRead(BR_READY_PIN)
    };

    memcpy(message.ok, status,6);

    publisher.publish(&message);

}
