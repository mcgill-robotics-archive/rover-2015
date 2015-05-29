#include "motor.h"
#include <Servo.h>


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
  // base link
  int base = setPoints.shoulderOrientation;
  if (base > 0)
  {
    digitalWrite(aBase, HIGH);
    digitalWrite(bBase, LOW);
  }
  else if (base < 0)
  {
    digitalWrite(aBase, LOW);
    digitalWrite(bBase, HIGH);
  }
  else
  {
    digitalWrite(aBase, LOW);
    digitalWrite(bBase, LOW);
  }
  analogWrite(speedBase, 255 * base);

  // shoulder
  int shoulder = setPoints.shoulderElevation;
  if (shoulder > 0)
  {
    digitalWrite(aShoulder, HIGH);
    digitalWrite(bShoulder, LOW);
  }
  else if (shoulder < 0)
  {
    digitalWrite(aShoulder, LOW);
    digitalWrite(bShoulder, HIGH);
  }
  else
  {
    digitalWrite(aShoulder, LOW);
    digitalWrite(bShoulder, LOW);
  }
  analogWrite(speedShoulder, 255 * shoulder);

  // elbow link
  int elbow = setPoints.elbow;
  if (elbow > 0)
  {
    digitalWrite(aElbow, HIGH);
    digitalWrite(bElbow, LOW);
  }
  else if (elbow < 0)
  {
    digitalWrite(aElbow, LOW);
    digitalWrite(bElbow, HIGH);
  }
  else
  {
    digitalWrite(aElbow, LOW);
    digitalWrite(bElbow, LOW);
  }
  analogWrite(speedElbow, 255 * base);

  // wrist
  int wrist = setPoints.wristElevation;
  if (wrist > 0)
  {
    digitalWrite(aWrist, HIGH);
    digitalWrite(bWrist, LOW);
  }
  else if (wrist < 0)
  {
    digitalWrite(aWrist, LOW);
    digitalWrite(bWrist, HIGH);
  }
  else
  {
    digitalWrite(aWrist, LOW);
    digitalWrite(bWrist, LOW);
  }
  analogWrite(speedWrist, 255 * shoulder);
  

}

void camera_motor(const control_systems::PanTiltZoom& setPoints)
{
  panServo.write(setPoints.pan);
  tiltServo.write(setPoints.tilt);  
}
