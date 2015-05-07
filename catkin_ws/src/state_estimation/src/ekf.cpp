/*
  Version 0.1
*/

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <rover_msgs/GPS.h>
#include "ekf.h"
#include <cmath>

using namespace ekf;

SquareStateMatrix EKF::fUpdate(double dt){
	SquareStateMatrix f;	
    Matrix<double,STATE_DIMS/2,STATE_DIMS/2> I = Matrix<double,STATE_DIMS/2,STATE_DIMS/2>::Identity();
	MatrixXd DT = I * dt;
	Matrix<double,STATE_DIMS/2,STATE_DIMS> ZERO = Matrix<double, STATE_DIMS/2,STATE_DIMS>::Zero();
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
	Matrix<double, STATE_DIMS, STATE_DIMS/2> bigH;
	StateToSensorMatrix H;
	Matrix<double, STATE_DIMS/2, STATE_DIMS> tempZero = Matrix<double, STATE_DIMS/2, STATE_DIMS>::Zero();

	//assume the state vector is <x, y, z, Vx, Vy, Vz>
	double x = previous_X(0,0);
	double y = previous_X(1,0);
	double z = previous_X(2,0);
	double coordinate3D = (double)pow(x, 2) + (double)pow(y,2) + (double)pow(z, 2);
	double coordinate2D = (double)pow(x, 2) + (double)pow(y,2);
	double aBigOperation = sqrt(1 - pow(z, 2)/(coordinate3D));

	//temporary vectors
	Matrix<double, STATE_DIMS/2, STATE_DIMS/2> a1;
	Matrix<double, STATE_DIMS/2, STATE_DIMS/2> a2 = Matrix<double, STATE_DIMS/2, STATE_DIMS/2>::Zero();
	Matrix<double, STATE_DIMS/2, STATE_DIMS/2> a3;
	Matrix<double, STATE_DIMS/2, STATE_DIMS/2> a4 = Matrix<double, STATE_DIMS/2, STATE_DIMS/2>::Zero();
	Matrix<double, STATE_DIMS/2, STATE_DIMS/2> a5;
	Matrix<double, STATE_DIMS/2, STATE_DIMS/2> a6 = Matrix<double, STATE_DIMS/2, STATE_DIMS/2>::Zero();;
	
	a1(0, 0) = x/sqrt(coordinate3D);
	a1(1, 0) = -y/sqrt(coordinate2D);
	a1(2, 0) = (x*y)/(pow(coordinate3D, (double)3.0/2) * aBigOperation);

	a3(0, 2) = y/sqrt(coordinate3D);
	a3(1, 2) = x/sqrt(coordinate2D);
	a3(2, 2) = (y*z)/(pow(coordinate3D, (double)3.0/2) * aBigOperation);

	a5(0, 4) = z/sqrt(coordinate3D);
	a5(1, 4) = 0;
	a5(2, 4) = ((-1/coordinate3D) + pow(z, 2)/pow(coordinate3D, (double)(3.0/2) ))/(aBigOperation);

	bigH << a1, a2, a3, a4, a5, a6;

	H << bigH.transpose(), tempZero;

	return H;
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
