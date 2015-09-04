//
// Created by David Lavoie-Boutin on 15-08-28.
//

#include "Arduino.h"
#include "ArmController.h"
#include "ros.h"

#include "control_systems/ArmAngles.h"
#include "rover_msgs/JointSpeedArm.h"
#include "rover_msgs/GetVoltageRead.h"
#include "rover_msgs/ArmModeControl.h"

#define WRIST_SPEED_FACTOR 100
#define ELBOW_SPEED_FACTOR 100
#define SHOUL_SPEED_FACTOR 100
#define ROLL_SPEED_FACTOR 60
#define CLAW_SPEED_FACTOR 0.5
#define BASE_SPEED_FACTOR 25

ros::NodeHandle nh;
bool positionControl = false;
bool velocityControl = false;

void handleChangeArmMode(const rover_msgs::ArmModeControl & armModeControl)
{
    nh.loginfo("changing arm control mode");
    if (armModeControl.PositionControl == true)
    {
        positionControl = true;
        velocityControl = false;
    }
    else if (armModeControl.VelocityControl == true)
    {
        positionControl = false;
        velocityControl = true;
    }
}

void handleAngles(const control_systems::ArmAngles& armAngles) {
    if (positionControl)
    {
        nh.logdebug("Handeling wrist position message");
        setPID_ON(true);
        setShoulderPos((int) degrees(armAngles.shoulderElevation));
        setElbowPos((int) (180 - degrees(armAngles.elbow)));
//    setWristPos((int) (180 + degrees(armAngles.wristElevation))); // will not work since encoder board broke
        setBaseVel((int) armAngles.shoulderOrientation * 50); //TODO: Scale properly
        //TODO: claw and roll
    }
}

void handleJointSpeed(const rover_msgs::JointSpeedArm& jointSpeedArm)
{
    static char buffer[64];
    if (velocityControl)
    {
        nh.logdebug("Handeling joint speed message");
        setPID_ON(false);
        if (jointSpeedArm.wrist.Enable)
            setWristVel((int) (jointSpeedArm.wrist.Value * WRIST_SPEED_FACTOR));
        else if (jointSpeedArm.elbow.Enable)
            setElbowVel((int) (jointSpeedArm.elbow.Value * ELBOW_SPEED_FACTOR));
        else if (jointSpeedArm.shoulder.Enable)
            setShoulderVel((int) (jointSpeedArm.shoulder.Value * SHOUL_SPEED_FACTOR));
        else if (jointSpeedArm.roll.Enable)
            setRollVel((int) (jointSpeedArm.roll.Value * ROLL_SPEED_FACTOR));
        else if (jointSpeedArm.grip.Enable) {
            int disp = (int) (jointSpeedArm.grip.Value * CLAW_SPEED_FACTOR);
            sprintf(buffer, "Grip message: %f, Claw steps: %i", jointSpeedArm.grip.Value, disp);
            nh.loginfo(buffer);
            setClawDisp(disp);
        }
        else if (jointSpeedArm.base.Enable)
            setBaseVel((int) (jointSpeedArm.base.Value * BASE_SPEED_FACTOR));
    }
}

void handleVoltageRequest(const rover_msgs::GetVoltageRead::Request &request,
                          rover_msgs::GetVoltageRead::Response &response)
{
    nh.loginfo("Received voltage service request");
    response.Voltage = getVoltage();
}

ros::Subscriber<control_systems::ArmAngles> angleSubscriber("/arm", &handleAngles);
ros::Subscriber<rover_msgs::JointSpeedArm> jointSubscriber("/arm_joint_speed", &handleJointSpeed);
ros::Subscriber<rover_msgs::ArmModeControl> modeSub("/arm_mode", &handleChangeArmMode);
ros::ServiceServer<rover_msgs::GetVoltageRead::Request, rover_msgs::GetVoltageRead::Response>
        voltageServiceServer("get_voltage",&handleVoltageRequest);

void setup()
{
    armSetup();
    nh.initNode();
    nh.subscribe(angleSubscriber);
    nh.subscribe(jointSubscriber);
    nh.subscribe(modeSub);
    nh.advertiseService(voltageServiceServer);
    nh.loginfo("Completed arm interfce setup, ready for commands");
}

void loop()
{
    armLoop();
    nh.spinOnce();
    delay(1);
}
