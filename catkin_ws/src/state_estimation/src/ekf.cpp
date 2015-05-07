/*
  Version 0.1
*/

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <rover_msgs/GPS.h>
#include <Eigen/Dense>
#include "ekf.h"


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
	SquareStateMatrix df = this.f - previous_f;
	StateVector dX = this.X - previous_X;

	if(dX.matrixLU().isInvertible()){
		F = df * dX.inverse();
		return F;	
	}

	return SquareSensorMatrix::Identity();

// 	for(int j=0; j<STATE_DIMS; j++){
// 		for(int k=0; k<STATE_DIMS; k++){
// 			F(j,k) = df(j,k) / dX(j,k);
// 		}
// 	}

}

StateVector EKF::XPredict(StateVector previous_X){
	return f * previous_X;
}

SquareStateMatrix EKF::PPredict(SquareStateMatrix previous_P, SquareStateMatrix F){
	return F * previous_P * F.transpose() + Q;
}

StateToSensorMatrix EKF::HCalc(StateVector previous_X){
	StateToSensorMatrix s = StateToSensorMatrix::Zero();
	return s;
}

SensorVector EKF::yUpdate(StateVector X){
	SensorVector hX = (h * X);

	return *sensorInput - hX;
}

SensorToStateMatrix EKF::KUpdate(SquareStateMatrix P, StateToSensorMatrix H){
	SquareSensorMatrix S = (H * P * H.transpose() + R);
	if(S.matrixLU().isInvertible()){
			SesnorToStateMatrix K = (P * H.transpose() * S.inverse());
			return K;
	}

	return SensorToStateMatrix::Identity;
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
	H = HCalc(X);
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
