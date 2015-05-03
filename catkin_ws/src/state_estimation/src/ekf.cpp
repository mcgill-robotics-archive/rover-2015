/*
  Version 0.1
*/

#include <ros/ros.h>
#include <ros/time.h>

#include "ekf.h"

using namespace ekf;

SquareStateMatrix EKF::FCalc(StateVector previous_X){
	SquareSensorMatrix s;
	return s;
}

StateVector EKF::XPredict(StateVector previous_X){
	return f * previous_X;
}

SquareStateMatrix EKF::PPredict(SquareStateMatrix previous_P, SquareStateMatrix F){
	return F * previous_P * F.transpose() + Q;
}

StateToSensorMatrix EKF::HCalc(StateVector previous_X){
	StateToSensorMatrix s;
	return s;
}

SensorVector EKF::yUpdate(StateVector X){
	SensorVector hX = (h * X);
	return *sensorInput - hX;
}

SensorToStateMatrix EKF::KUpdate(SquareStateMatrix P, StateToSensorMatrix H){
	return ( P * H.transpose() * (H * P * H.transpose() + R));
}

StateVector EKF::XUpdate(StateVector X, SensorToStateMatrix K, SensorVector y){
	return (X + K * y);
}

SquareStateMatrix EKF::PUpdate(SquareStateMatrix P, SensorToStateMatrix K, StateToSensorMatrix H){
	SquareStateMatrix s;
	return s;
}

int main (int argc, char **argv) {
    // ros::init(argc, argv, "ekf");								 //
    // ros::NodeHandle node;										 //
	// 																 //
    // pub = node.advertise<geometry_msgs::PoseStamped>("ekf", 100); //
    // sub = node.subscribe("imu_data", 100, dataCallback);			 //
	// 																 //
    // ros::spin();													 //
	SensorVector *sensorInput;
	SquareStateMatrix P;
	SquareStateMatrix Q;
	SquareStateMatrix R;

	EKF ekf(sensorInput, P, Q, R);
	
    return 0;
}
