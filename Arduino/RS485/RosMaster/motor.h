#ifndef MOTOR_H
#define MOTOR_H

#include <control_systems/ArmAngles.h>
#include <std_msgs/Int16.h>
#include <control_systems/PanTiltZoom.h>

int aBase = A6;
int bBase = A11;
int speedBase = 2;
int aShoulder = A5;
int bShoulder = A10;
int speedShoulder = 3;
int aElbow = A4;
int bElbow = A9;
int speedElbow = 4;
int aWrist = A3;
int bWrist = A8;
int speedWrist = 5;

enum DriveAddress {
  sfsa = 34,
  pfsa = 35,
  srsa = 36,
  prsa = 37,
  pfrv = 1,
  sfrv = 2,
  pmrv = 3,
  smrv = 4,
  prrv = 5,
  srrv = 6
};

enum ArmAddress {
  base = 11,
  elevation = 12,
  elbow = 13,
  wrist_elevation = 14
};

enum CamMotor {
  pan = 32,
  tilt = 31
};

void claw(const std_msgs::Int16& boole);
void drive_motor(const std_msgs::Int16& setPoints);
void arm_motor(const control_systems::ArmAngles& setPoints);
void camera_motor(const control_systems::PanTiltZoom& setPoints);
#endif
