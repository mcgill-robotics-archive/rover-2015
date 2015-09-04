//
// Created by David Lavoie-Boutin on 15-08-28.
//

#ifndef ARDUINO_TEST_ARMCONTROLLER_H
#define ARDUINO_TEST_ARMCONTROLLER_H

#include "Servo.h"

void armSetup();
void armLoop();
void setPID_ON(bool val);


void setShoulderVel(int vel);
void setShoulderPos(int pos);
double getShoulderPos();

void setElbowVel(int vel);
void setElbowPos(int pos);
double getElbowPos();

void setWristVel(int vel);
void setWristPos(int pos);
double getWristPos();

void setBaseVel(int vel);
void setRollVel(int vel);
void setClawDisp(float disp);
void setSciencePos(int pos);
double getVoltage();
float singlePHRead();
void setPHTemp(int temp);
float singleHumidRead();

// private stuff
float readEncoder(int pin);
void stepPID();
void setClawDisplacement(float disp);
void setWristRotVelocity(int vel);
void setLinkVelocity(int vel, Servo servo);
void setServoVelocity(int vel, Servo servo, int minim, int maxim);

#endif //ARDUINO_TEST_ARMCONTROLLER_H
