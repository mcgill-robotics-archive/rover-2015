#include "motor.h"
#include <Servo.h>


void claw(const std_msgs::Int16& boole)
{
 if (boole.data == -1){
    //close  
    myStepper.setSpeed(motorSpeed);
    
 }
 else  if (boole.data == 1){
    // open
    myStepper.setSpeed(-motorSpeed);
    
 }
 else {
  // stop 
  myStepper.setSpeed(0);
    
 }
 
}


void drive_motor(const std_msgs::Int16& setPoints)
{
    ENB = 1;
    BRK = 0;
    DRV = setPoints.data;

    if (ENB==1){
      digitalWrite(EN, HIGH);
    }
    else {
      digitalWrite(EN, LOW);
    }
    
    if (BRK==1){
      digitalWrite(BK, HIGH);
    }
    else {
      digitalWrite(BK, LOW);
    }

    if (DRV<5000){
      digitalWrite(DR, LOW);
      int DRVd= map(DRV, 5000,1000,0,4000);
      WriteRegister(ADDB, DRVd,1);
      WriteRegister(ADD0, REG01,1);
      WriteRegister(ADD2, REG21,1);
      WriteRegister(ADD3, REG31,1);
      WriteRegister(ADD4, REG41,1);
      WriteRegister(ADDA, REGA1,1);
    }
    else{
      digitalWrite(DR, HIGH);
      int DRVd=map(DRV, 5000, 9000, 0, 4000);
      WriteRegister(ADDB, DRVd,1);
      WriteRegister(ADD0, REG01,1);
      WriteRegister(ADD2, REG21,1);
      WriteRegister(ADD3, REG31,1);
      WriteRegister(ADD4, REG41,1);
      WriteRegister(ADDA, REGA1,1);
    
  }
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
