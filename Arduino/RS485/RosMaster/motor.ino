#include "motor.h"


void drive_motor(const control_systems::SetPoints& setPoints)
{
    setMotorSpeed(pfrv, setPoints.speedFL);
    setMotorSpeed(srrv, setPoints.speedFR);
    setMotorSpeed(sfrv, setPoints.speedML);
    setMotorSpeed(pmrv, setPoints.speedMR);
    setMotorSpeed(smrv, setPoints.speedRL);
    setMotorSpeed(prrv, setPoints.speedRR);

    setAngle(sfsa, setPoints.thetaFL);
    setAngle(pfsa, setPoints.thetaFR);
    setAngle(srsa, setPoints.thetaRL);
    setAngle(prsa, setPoints.thetaRR);
}

void arm_motor(const control_systems::ArmAngles& setPoints)
{
  setMotorSpeed(base, setPoints.shoulderOrientation);
  setAngle(elevation, setPoints.shoulderElevation);
  setAngle(elbow, setPoints.elbow);
  setAngle(wrist_elevation, setPoints.wristElevation);
}

