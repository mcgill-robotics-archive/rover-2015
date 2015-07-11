#include <Servo.h>
#include <ros.h>
#include "pins.h"
#include <control_systems/SetPoints.h>
#include "SteeringControl.h"
#include "DriveControl.h"
#include "std_msgs/Bool.h"
#include "rover_msgs/MotorControllerMode.h"
#include "DataControl.h"

ros::NodeHandle nh;

float radToDeg(float rad)
{
    return rad / PI * 180.0;
}

void driveCallback( const control_systems::SetPoints& setPoints )
{
    FLsetSpeed(setPoints.speedFL);
    FRsetSpeed(setPoints.speedFR);
    MLsetSpeed(setPoints.speedML);
    MRsetSpeed(setPoints.speedMR);
    BLsetSpeed(setPoints.speedRL);
    BRsetSpeed(setPoints.speedRR);

    float flAngle = 90.0 + radToDeg(setPoints.thetaFL);
    float frAngle = 90.0 + radToDeg(setPoints.thetaFR);
    float blAngle = 90.0 + radToDeg(setPoints.thetaRL);
    float brAngle = 90.0 + radToDeg(setPoints.thetaRR);

    setWheelAngle(flAngle, frAngle, blAngle, brAngle);
}

void callbackMoving( const std_msgs::Bool& boolean)
{
    if (boolean.data)
        enableMotors();
    else
        disableMotors();
}

void mcMode(const rover_msgs::MotorControllerMode& msg)
{
    nh.loginfo("called in subscriber");
    if (msg.highSpeed)
    {
        nh.loginfo("high");
        data::setModeHighSpeed();
    }
    else if (msg.medSpeed)
    {
        nh.loginfo("med");
        data::setModeMediumSpeed();
    }
    else if (msg.lowSpeed)
    {
        nh.loginfo("low");
        data::setModeLowSpeed();
    }
    else
    {
        nh.loginfo("open");
        data::setModeOpenLoop();
    }
}

ros::Subscriber<control_systems::SetPoints> driveSubscriber("/wheels", &driveCallback );
ros::Subscriber<std_msgs::Bool> movingSubscriber("/is_moving", &callbackMoving);
ros::Subscriber<rover_msgs::MotorControllerMode> modeSubscriber("/mc_mode", &mcMode);

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

    pinMode(FL_DATA1_PIN, OUTPUT);
    pinMode(FL_DATA2_PIN, OUTPUT);
    pinMode(FR_DATA1_PIN, OUTPUT);
    pinMode(FR_DATA2_PIN, OUTPUT);
    pinMode(ML_DATA1_PIN, OUTPUT);
    pinMode(ML_DATA2_PIN, OUTPUT);
    pinMode(MR_DATA1_PIN, OUTPUT);
    pinMode(MR_DATA2_PIN, OUTPUT);
    pinMode(BL_DATA1_PIN, OUTPUT);
    pinMode(BL_DATA2_PIN, OUTPUT);
    pinMode(BR_DATA1_PIN, OUTPUT);
    pinMode(BR_DATA2_PIN, OUTPUT);

    attachServos();

    nh.initNode();
    nh.subscribe(driveSubscriber);
    nh.subscribe(movingSubscriber);
    nh.subscribe(modeSubscriber);
}

void loop()
{
    nh.spinOnce();
    delay(1);
}
