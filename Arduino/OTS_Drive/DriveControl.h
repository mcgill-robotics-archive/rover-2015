//
// Created by David Lavoie-Boutin on 15-07-01.
//

#ifndef ARDUINO_TEST_DRIVECONTROL_H
#define ARDUINO_TEST_DRIVECONTROL_H


void FLsetSpeed(double speed);
void FRsetSpeed(double speed);
void MLsetSpeed(double speed);
void MRsetSpeed(double speed);
void BLsetSpeed(double speed);
void BRsetSpeed(double speed);

void enableMotors(bool watchDog);
void disableMotors();

#endif //ARDUINO_TEST_DRIVECONTROL_H
