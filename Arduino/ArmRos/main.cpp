//
// Created by David Lavoie-Boutin on 15-08-28.
//

#include "Arduino.h"
#include "ArmController.h"
#include "ros.h"

#include "control_systems/ArmAngles.h"
#include "control_systems/ArmMotion.h"

void handleAngles(const control_systems::ArmAngles& armAngles)
{
    setPID_ON(true);
    setShoulderPos((int) degrees(armAngles.shoulderElevation));
    setElbowPos((int) (180 - degrees(armAngles.elbow)));
//    setWristPos((int) (180 + degrees(armAngles.wristElevation))); // will not work since encoder board broke
    setBaseVel((int) armAngles.shoulderOrientation * 50); //TODO: Scale properly
    //TODO: claw and roll
}

ros::NodeHandle nh;
ros::Subscriber<control_systems::ArmAngles> angleSubscriber("/arm", &handleAngles);

void setup()
{
    armSetup();
    nh.initNode();
    nh.subscribe(angleSubscriber);
}

void loop()
{
    armLoop();
    nh.spinOnce();
    delay(1);
}
