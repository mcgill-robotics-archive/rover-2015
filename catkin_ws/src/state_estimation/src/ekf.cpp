/*
  Version 0.1
*/

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "include/Eigen/Dense"
#include "ekf.h"

using namespace ekf;

SquareStateMatrix EKF::Fcalc(){
	return null;
}

StateVector EKF::XPredict(StateVector &previous_X){
	return f * previous_X;
}

SquareStateMatrix EKF::PPredict(SquareStateMatrix &previous_P, SquareStateMatrix &F){
	return F * previous_P * F.transpose() + Q;
}

StateToSensorMatrix HCalc(){
	return null;
}

SensorVector yUpdate(StateVector &X){
	return z - h * X;
}

SensorToStateMatrix KUpdate(){
	return null;
}

StateVector XUpdate(){
	return null;
}

SquareStateMatrix PUpdate(){
	return null;
}

int main(int argc, char **argv){


}
// int main (int argc, char **argv) {								 //
//     ros::init(argc, argv, "ekf");								 //
//     ros::NodeHandle node;										 //
// 																	 //
//     pub = node.advertise<geometry_msgs::PoseStamped>("ekf", 100); //
//     sub = node.subscribe("imu_data", 100, dataCallback);			 //
// 																	 //
//     ros::spin();													 //
// 																	 //
//     return 0;													 //
// }																 //
