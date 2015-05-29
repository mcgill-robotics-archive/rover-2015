#ifndef MOTOR_H
#define MOTOR_H

#include <control_systems/ArmAngles.h>
#include <control_systems/SetPoints.h>
#include <control_systems/PanTiltZoom.h>

int aBase = 1;
int bBase = 2;
int speedBase = 3;
int aShoulder = 1;
int bShoulder = 2;
int speedShoulder= 3;
int aElbow = 1;
int bElbow = 2;
int speedElbow = 3;
int aWrist = 1;
int bWrist = 2;
int speedWrist = 3;


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

void drive_motor(const control_systems::SetPoints& setPoints);
void arm_motor(const control_systems::ArmAngles& setPoints);
void camera_motor(const control_systems::PanTiltZoom& setPoints);

#endif
