//
// Created by david on 7/21/15.
//

#include <Ahrs.h>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <signal.h>
#include <rover_msgs/AhrsStatusMessage.h>
#include "ros/ros.h"

void mySigintHandler(int sig)
{
    ROS_INFO("ahrs_node terminated");
    ros::shutdown();
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "ahrs_node");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);

    lineranger::ahrs::AhrsConfig config;
    config.setSimulation(false);
    boost::scoped_ptr<lineranger::ahrs::Ahrs> ahrs;
    try
    {
        ahrs.reset(lineranger::ahrs::Ahrs::createAhrs(config));
    }
    catch (const std::runtime_error& error)
    {
        ROS_ERROR(error.what());
        return (-1);
    }

    ros::Publisher ahrsPublisher = nh.advertise<rover_msgs::AhrsStatusMessage>("ahrs_status", 100);
    lineranger::ahrs::AhrsStatus ahrsStatus;

    ros::Rate loopRate(10);
    rover_msgs::AhrsStatusMessage msg;
    ROS_INFO("ahrs_node starting acquisition");
    while (ros::ok())
    {
        ahrsStatus = ahrs->getStatus();
        msg.gpsLongitude = ahrsStatus.gpsLongitude;
        msg.gpsAltitude = ahrsStatus.gpsAltitude;
        msg.gpsLatitude = ahrsStatus.gpsLatitude;

        msg.heading = ahrsStatus.heading;
        msg.pitch = ahrsStatus.pitch;
        msg.roll = ahrsStatus.roll;
        msg.yaw = ahrsStatus.yaw;

        msg.velocity.x = ahrsStatus.velocity[0];
        msg.velocity.y = ahrsStatus.velocity[1];
        msg.velocity.z = ahrsStatus.velocity[2];

        ahrsPublisher.publish(msg);
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}