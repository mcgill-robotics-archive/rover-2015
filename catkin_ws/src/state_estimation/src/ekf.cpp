/*
  Version 0.1
*/

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <rover_msgs/GPS.h>
#include <Eigen/Dense>
#include "ekf.h"
#include <cmath>


using namespace Eigen;

namespace ekf{
	
SquareStateMatrix EKF::fUpdate(double dt){
	SquareStateMatrix f;
    Matrix<double,STATE_DIMS/2,STATE_DIMS/2> I = Matrix<double,STATE_DIMS/2,STATE_DIMS/2>::Identity();
	MatrixXd DT = I * dt;
	Matrix<double,STATE_DIMS/2,STATE_DIMS> ZERO = Matrix<double, STATE_DIMS/2,STATE_DIMS>::Zero();
	f << I, DT, ZERO;
}

SquareStateMatrix EKF::FCalc(StateVector previous_X, SquareStateMatrix previous_f){
	SquareStateMatrix F;
	SquareStateMatrix df = this->f - previous_f;
	StateVector dX = this->X - previous_X;

	for(int j=0; j<STATE_DIMS; j++){
		for(int k=0; k<STATE_DIMS; k++){
			if(abs(dX(k,0)) > .000001){
				F(j,k) = df(j,k) / dX(k,0);
			}
		}
	}

	return F;
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
	SquareSensorMatrix S = (H * P * H.transpose() + R);
	if(S.determinant() != 0){
		SensorToStateMatrix K = (P * H.transpose() * S.inverse());
		return K;
	}

	SensorToStateMatrix I = SensorToStateMatrix::Identity();
	return I;
}

StateVector EKF::XUpdate(StateVector X, SensorToStateMatrix K, SensorVector y){
	return (X + K * y);
}

SquareStateMatrix EKF::PUpdate(SquareStateMatrix P, SensorToStateMatrix K, StateToSensorMatrix H){
	SquareStateMatrix I = SquareStateMatrix::Identity();
	return (I - K*H) * P;
}

/*
  Prediction step, X_k|k-1 and P_k|k-1 are predicted based on previous updated values.
*/	  
void EKF::predict(double t_current){
	this->dt = t_current - t_previous;
	t_previous = t_current;
	previous_X = X;
	previous_f = f;
	f = fUpdate(dt);
	X = XPredict(X);
	F = FCalc(previous_X, previous_f);
	P = PPredict(P, F);
}

/*
  Update step, X_k|k, y_k, K_k, P_k|k are all updated based on obsevation.
*/
void EKF::update(){
	H = HCalc(previous_X);
	y = yUpdate(X);
	K = KUpdate(P, H);
	X = XUpdate(X, K, y);
	P = PUpdate(P, K, H);
}

SquareStateMatrix generateP(double covariance){
	SquareStateMatrix P = SquareStateMatrix::Identity() * covariance;
	return P;
}

SquareStateMatrix generateP(double covariance[STATE_DIMS]){
	SquareStateMatrix P = SquareStateMatrix::Zero();
	for(int i=0; i<STATE_DIMS; i++){
		P(i,i) = covariance[i];
	}

	return P;
}

SquareStateMatrix generateQ(double covariance[STATE_DIMS]){
	SquareStateMatrix Q = SquareStateMatrix::Identity();
	for(int i=0; i<STATE_DIMS; i++){
		Q(i,i) = covariance[i];
	}
	
	return Q;
}

SquareSensorMatrix generateR(double covariance[SENSOR_DIMS]){
	SquareSensorMatrix R = SquareSensorMatrix::Identity();
	for(int i=0; i<SENSOR_DIMS; i++){
		R(i,i) = covariance[i];
	}
	
	return R;
}
};
