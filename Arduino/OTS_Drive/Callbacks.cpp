//
// Created by David Lavoie-Boutin on 15-07-26.
//

#include "std_msgs/Bool.h"
#include <ros.h>
#include "rover_msgs/ResetWatchDog.h"
#include "DataControl.h"
#include "rover_msgs/MotorControllerMode.h"
#include <control_systems/SetPoints.h>
#include <DriveControl.h>
#include <SteeringControl.h>
#include "Callbacks.h"
#include "Arduino.h"

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

float radToDeg(float rad)
{
    return rad / PI * 180.0;
}

void mcMode(const rover_msgs::MotorControllerMode& msg)
{
    if (msg.highSpeed)
    {
        data::setModeHighSpeed();
    }
    else if (msg.medSpeed)
    {
        data::setModeMediumSpeed();
    }
    else if (msg.lowSpeed)
    {
        data::setModeLowSpeed();
    }
    else
    {
        data::setModeOpenLoop();
    }
}

void callbackMoving( const std_msgs::Bool& boolean)
{
    if (boolean.data)
        enableMotors();
    else
        disableMotors();
}

void callbackResetWatchdog(const rover_msgs::ResetWatchDog::Request & request, rover_msgs::ResetWatchDog::Response & response)
{
    if (request.OpCode == 1)
    {
        // running fine
        lastReset = millis(); // TODO: check if token matches sequence
        response.Response = 555;
        watchDog = false;
    }
    else if (request.OpCode == 2)
    {
        // reset, should use token to get seed parameters
        watchDog = false;
        lastReset = millis();
        response.Response = request.Token;
    }
    else
    {
        // whatever the OpCode, trigger fail safe
        lastReset = 0;
        disableMotors();

    }
}