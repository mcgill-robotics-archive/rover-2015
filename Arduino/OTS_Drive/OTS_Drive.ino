#include <Servo.h>
#include <ros.h>
#include "pins.h"
#include <control_systems/SetPoints.h>

Servo LF_servo, RF_servo, LR_servo, RR_servo;

//SET STRAIGHT AS DEFAULT
float LF_servo_angle = 90.0;
float RF_servo_angle = 90.0;
float LR_servo_angle = 90.0;
float RR_servo_angle = 90.0;

//initialize limits on what commands can be sent to servos
int frontServoLowerLimit_cmd = 1000;
int frontServoUpperLimit_cmd = 2000;
int rearServoLowerLimit_cmd = 1000;
int rearServoUpperLimit_cmd = 2000;

int LF_servo_cmd = 0;
int RF_servo_cmd = 0;
int LR_servo_cmd = 0;
int RR_servo_cmd = 0;  

void driveCallback( const control_systems::SetPoints& setPoints )
{
    FLsetSpeed(setPoints.speedFL);
    FRsetSpeed(setPoints.speedFR);
    MLsetSpeed(setPoints.speedML);
    MRsetSpeed(setPoints.speedMR);
    BLsetSpeed(setPoints.speedRL);
    BRsetSpeed(setPoints.speedRR);
}

void FLsetSpeed(double speed) 
{
    // cw is gnd
    // ccw is vcc
    if (speed == 0)
        digitalWrite(FL_ENABLE_PIN, LOW);
    else
    {
        digitalWrite(FL_ENABLE_PIN, HIGH);
        if (speed < 0)
            digitalWrite(FL_DIRECTION_PIN, HIGH); // confirm direction polarity
        else 
            digitalWrite(FL_DIRECTION_PIN, LOW);

        analogWrite(FL_DRIVE_PIN, abs(speed));
    }
}

void FRsetSpeed(double speed) 
{
    // cw is gnd
    // ccw is vcc
    if (speed == 0)
        digitalWrite(FR_ENABLE_PIN, LOW);
    else
    {
        digitalWrite(FR_ENABLE_PIN, HIGH);
        if (speed < 0)
            digitalWrite(FR_DIRECTION_PIN, HIGH); // confirm direction polarity
        else 
            digitalWrite(FR_DIRECTION_PIN, LOW);

        analogWrite(FR_DRIVE_PIN, abs(speed));
    }
}

void MLsetSpeed(double speed) 
{
    // cw is gnd
    // ccw is vcc
    if (speed == 0)
        digitalWrite(ML_ENABLE_PIN, LOW);
    else
    {
        digitalWrite(ML_ENABLE_PIN, HIGH);
        if (speed < 0)
            digitalWrite(ML_DIRECTION_PIN, HIGH); // confirm direction polarity
        else 
            digitalWrite(ML_DIRECTION_PIN, LOW);

        analogWrite(ML_DRIVE_PIN, abs(speed));
    }
}

void MRsetSpeed(double speed) 
{
    // cw is gnd
    // ccw is vcc
    if (speed == 0)
        digitalWrite(MR_ENABLE_PIN, LOW);
    else
    {
        digitalWrite(MR_ENABLE_PIN, HIGH);
        if (speed < 0)
            digitalWrite(MR_DIRECTION_PIN, HIGH); // confirm direction polarity
        else 
            digitalWrite(MR_DIRECTION_PIN, LOW);

        analogWrite(MR_DRIVE_PIN, abs(speed));
    }
}

void BLsetSpeed(double speed) 
{
    // cw is gnd
    // ccw is vcc
    if (speed == 0)
        digitalWrite(BL_ENABLE_PIN, LOW);
    else
    {
        digitalWrite(BL_ENABLE_PIN, HIGH);
        if (speed < 0)
            digitalWrite(BL_DIRECTION_PIN, HIGH); // confirm direction polarity
        else 
            digitalWrite(BL_DIRECTION_PIN, LOW);

        analogWrite(BL_DRIVE_PIN, abs(speed));
    }
}

void BRsetSpeed(double speed) 
{
    // cw is gnd
    // ccw is vcc
    if (speed == 0)
        digitalWrite(BR_ENABLE_PIN, LOW);
    else
    {
        digitalWrite(BR_ENABLE_PIN, HIGH);
        if (speed < 0)
            digitalWrite(BR_DIRECTION_PIN, HIGH); // confirm direction polarity
        else 
            digitalWrite(BR_DIRECTION_PIN, LOW);

        analogWrite(BR_DRIVE_PIN, abs(speed));
    }
}

void setWheelAngle()
{ 
  //convert angle in degrees to command
  LF_servo_cmd = map(LF_servo_angle,0,180,1000,2000); 
  RF_servo_cmd = map(RF_servo_angle,0,180,1000,2000);
  LR_servo_cmd = map(LR_servo_angle,0,180,1000,2000);
  RR_servo_cmd = map(RR_servo_angle,0,180,1000,2000);
  
  //Fine tune servo calibration:
  //should not be greater than like 50 or 100
  //The larger the number, the smaller the range of motion of the servos
  LF_servo_cmd += 0;
  RF_servo_cmd += 0;
  LR_servo_cmd += 0;
  RR_servo_cmd += 0;  
    
  LF_servo_cmd = constrain(LF_servo_cmd, frontServoLowerLimit_cmd, frontServoUpperLimit_cmd);
  RF_servo_cmd = constrain(RF_servo_cmd, frontServoLowerLimit_cmd, frontServoUpperLimit_cmd);
  LR_servo_cmd = constrain(LR_servo_cmd, rearServoLowerLimit_cmd, rearServoUpperLimit_cmd);
  RR_servo_cmd = constrain(RR_servo_cmd, rearServoLowerLimit_cmd, rearServoUpperLimit_cmd);

  //send signals to motors
  LF_servo.writeMicroseconds(LF_servo_cmd);
  RF_servo.writeMicroseconds(RF_servo_cmd);
  LR_servo.writeMicroseconds(LR_servo_cmd);
  RR_servo.writeMicroseconds(RR_servo_cmd);

}

ros::NodeHandle nh;
control_systems::SetPoints setPoints;
ros::Subscriber<control_systems::SetPoints> driveSubscriber("toggle_led", &driveCallback );

void setup()
{
    // set drive output pin mode
    pinMode(FL_DRIVE_PIN, OUTPUT);
    pinMode(FR_DRIVE_PIN, OUTPUT);
    pinMode(ML_DRIVE_PIN, OUTPUT);
    pinMode(MR_DRIVE_PIN, OUTPUT);
    pinMode(BL_DRIVE_PIN, OUTPUT);
    pinMode(BR_DRIVE_PIN, OUTPUT);

    // set direction pins to output
    pinMode(FL_DIRECTION_PIN, OUTPUT);
    pinMode(FR_DIRECTION_PIN, OUTPUT);
    pinMode(ML_DIRECTION_PIN, OUTPUT);
    pinMode(MR_DIRECTION_PIN, OUTPUT);
    pinMode(BL_DIRECTION_PIN, OUTPUT);
    pinMode(BR_DIRECTION_PIN, OUTPUT);

    // set enable pins to output
    pinMode(FL_ENABLE_PIN, OUTPUT);
    pinMode(FR_ENABLE_PIN, OUTPUT);
    pinMode(ML_ENABLE_PIN, OUTPUT);
    pinMode(MR_ENABLE_PIN, OUTPUT);
    pinMode(BL_ENABLE_PIN, OUTPUT);
    pinMode(BR_ENABLE_PIN, OUTPUT);

    // set steering pins to output
    pinMode(FL_STEERING_PIN, OUTPUT);
    pinMode(FR_STEERING_PIN, OUTPUT);
    pinMode(BL_STEERING_PIN, OUTPUT);
    pinMode(BR_STEERING_PIN, OUTPUT);

    LF_servo.attach(FL_STEERING_PIN);
    RF_servo.attach(FR_STEERING_PIN);
    LR_servo.attach(BL_STEERING_PIN);
    RR_servo.attach(BR_STEERING_PIN);

    nh.initNode();
    nh.subscribe(driveSubscriber);
}

void loop()
{
    nh.spinOnce();
    delay(1);
}
