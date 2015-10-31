#include "Arduino.h"
#include <Servo.h>
#include <ros.h>
#include <control_systems/SetPoints.h>
#include <geometry_msgs/Twist.h>
#include "rover_msgs/ResetWatchDog.h"
#include "std_msgs/Bool.h"
#include "rover_msgs/MotorControllerMode.h"
#include "pins.h"
#include "SteeringControl.h"
#include "DriveControl.h"
#include "DataControl.h"
#include "Camera.h"
#include "Callbacks.h"

#define MOTOR_STATUS_UPDATE_RATE 100

ros::NodeHandle nh;
rover_msgs::MotorStatus motorStatus;

unsigned long lastSend = 0;

ros::Subscriber<control_systems::SetPoints> driveSubscriber("/wheels", &driveCallback );
ros::Subscriber<std_msgs::Bool> movingSubscriber("/is_moving", &callbackMoving);
ros::Subscriber<rover_msgs::MotorControllerMode> modeSubscriber("/mc_mode", &mcMode);
ros::Subscriber<geometry_msgs::Twist> cameraSubscriber("/camera_motion", &callbackCamera);
ros::Publisher motorStatusPublisher("motor_status", &motorStatus);
ros::ServiceServer<rover_msgs::ResetWatchDog::Request, rover_msgs::ResetWatchDog::Response>
        voltageServiceServer("reset_watchdog",&callbackResetWatchdog);

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
    initCameraTable();

    nh.initNode();
    nh.subscribe(driveSubscriber);
    nh.subscribe(movingSubscriber);
    nh.subscribe(modeSubscriber);
    nh.subscribe(cameraSubscriber);
    nh.advertiseService(voltageServiceServer);
    nh.advertise(motorStatusPublisher);

}

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void loop()
{
//    if ((millis() - lastReset) > 500)
//    {
//        disableMotors();
//        watchDog = true;
//    }

    if ((millis() - lastSend > MOTOR_STATUS_UPDATE_RATE))
    {
        data::sendMotorStatus(motorStatusPublisher);
        lastSend = millis();
    }
    char message[10];
    int ram = freeRam();
    sprintf(message, "Free ram: %d", ram);
    nh.logdebug(message);
    nh.spinOnce();
    delay(1);
}
