#ifndef MOTOR_H
#define MOTOR_H

#include <control_systems/ArmAngles.h>
#include <control_systems/SetPoints.h>

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

void drive_motor(const control_systems::SetPoints& setPoints);
void arm_motor(const control_systems::ArmAngles& setPoints);

#endif
