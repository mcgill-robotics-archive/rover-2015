/**
 * Odometry computation from middle wheels
 */
 

/* Include Files */
#include "mw_odo.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float32.h"
#include "odometry/DisplacementMiddleWheels.h"


/** Odometry computation for position reporting based on middle wheel rotation
 *
 * All distances are in meters 
 * All angles are in radians 
 * Tacho counts should be degrees travelled since on 
 * 
 * Angle 0 is full forward, positive angle counter-clockwise 
 * Positive X axis goes front, positive Y is left 
 */
bool MW_odo(odometry::DisplacementMiddleWheels::Request  &req,
            odometry::DisplacementMiddleWheels::Response &res)
{
  double diffLeft;
  double diffRight;
  double diffCenter;
  double diffTheta;

  double pi = 3.1415926535897931;
  geometry_msgs::Pose pose;

  diffLeft = rLeft * pi * req.diffTachoLeft / 180.0;
  diffRight = rRight * pi * req.diffTachoRight / 180.0;
  diffCenter = (diffLeft + diffRight) / 2.0;
  diffTheta = (diffRight - diffLeft) / whBase;
  pose.position.x = diffCenter * cos(diffTheta);
  pose.position.y = diffCenter * sin(diffTheta);
  pose.orientation.z = diffTheta;

  res.diffPosition = pose;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mw_odometry");

  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("odo_mw_disp", MW_odo);

  n.param<double>("wheel_radius/left/middle", rLeft, 0.20);
  n.param<double>("wheel_radius/right/middle", rRight, 0.20);
  n.param<double>("controls/wh_base", whBase, 0.40);
  whBase*=2;
  ros::spin();

  return 0;
}

/*
 * File trailer for MW_odo.c
 */
