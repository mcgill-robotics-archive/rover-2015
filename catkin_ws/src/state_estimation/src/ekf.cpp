/*
  Version 0.1
*/

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <rover_msgs/GPS.h>
#include "ekf.h"

using namespace ekf;

SquareStateMatrix EKF::fUpdate(double dt){
	SquareStateMatrix f;	
    Matrix<double,3,3> I = Matrix<double,3,3>::Identity();
	MatrixXd DT = I * dt;

	Matrix<double,3,6> ZERO = Matrix<double,3,6>::Zero();
	f << I, DT, ZERO;
}

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
	SquareStateMatrix I = SquareStateMatrix::Identity();
	return (I - K*H) * P;
}
