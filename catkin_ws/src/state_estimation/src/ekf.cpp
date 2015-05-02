/*
  Version 0.1
*/

#include <ros/ros.h>
#include <ros/time.h>

#include "ekf.h"

using namespace ekf;

SquareStateMatrix EKF::Fcalc(StateVector previous_X){
	return null;
}

StateVector EKF::XPredict(StateVector previous_X){
	return f * previous_X;
}

SquareStateMatrix EKF::PPredict(SquareStateMatrix previous_P, SquareStateMatrix F){
	return F * previous_P * F.transpose() + Q;
}

StateToSensorMatrix HCalc(StateVector previous_X){
	return null;
}

SensorVector yUpdate(StateVector X){
	return z - h * X;
}

SensorToStateMatrix KUpdate(SquareStateMatrix P, StateToSensorMatrix H){
	return ( P * H.transpose() * (H * P * H.transpose() + R));
}

StateVector XUpdate(StateVector X, SensorToStateMatrix K, SensorVector y){
	return (X + K * y);
}

SquareStateMatrix PUpdate(SquareStateMatrix P, SensorToStateMatrix K, StateToSensorMatrix H){
	return null;
}

int main(int argc, char **argv){


}
int main (int argc, char **argv) {
    // ros::init(argc, argv, "ekf");								 //
    // ros::NodeHandle node;										 //
	// 																 //
    // pub = node.advertise<geometry_msgs::PoseStamped>("ekf", 100); //
    // sub = node.subscribe("imu_data", 100, dataCallback);			 //
	// 																 //
    // ros::spin();													 //

	SensorVector z;
	SquareStateMatrix P;
	SquareStateMatrix Q;
	SquareStateMatrix R;

	EKF ekf = new EKF(&z, P, Q, R);
	
    return 0;
}
