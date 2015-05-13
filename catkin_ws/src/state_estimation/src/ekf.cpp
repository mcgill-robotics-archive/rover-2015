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

double LAT_TO_METERS  = 0.000009044;
double LONG_TO_METERS = 0.000008983;
	
SquareStateMatrix EKF::fUpdate(){
	SquareStateMatrix f = SquareStateMatrix::Identity();
    // Matrix<double,STATE_DIMS/2,STATE_DIMS/2> I = Matrix<double,STATE_DIMS/2,STATE_DIMS/2>::Identity();
	// MatrixXd DT = I * dt;
	// Matrix<double,STATE_DIMS/2,STATE_DIMS> ZERO = Matrix<double, STATE_DIMS/2,STATE_DIMS>::Zero();
	f(2,2) = 0;
	f(0,2) = dt * cos(X(3));
	f(1,2) = dt * sin(X(3));
	std::cout << "f: " << std::endl << f << std::endl;
}

// SquareStateMatrix EKF::FCalc(StateVector previous_X, SquareStateMatrix previous_f){
// 	SquareStateMatrix F;
// 	SquareStateMatrix df = this->f - previous_f;
// 	StateVector dX = this->X - previous_X;

// 	for(int j=0; j<STATE_DIMS; j++){
// 		for(int k=0; k<STATE_DIMS; k++){
// 			if(abs(dX(k,0)) > .000001){
// 				F(j,k) = df(j,k) / dX(k,0);
// 			}
// 		}
// 	}

// 	return F;
// }

StateVector EKF::XPredict(StateVector previous_X){
	B(3,1) = dt;
	std::cout << "B : " << std::endl << B << std::endl;	
	StateVector X = f * previous_X + B * u;
	std::cout << "X-predict: " << std::endl << X << std::endl;
	return X;
}

SquareStateMatrix EKF::PPredict(SquareStateMatrix previous_P, SquareStateMatrix F){
	return f * previous_P * f.transpose() + Q;
}

/*
  TEMPORARY, MUST BE PROPERLY FORMATTED!!!
  Updates the h-matrix, which maps the state vector into the sensor-space, so that
  the sensor offset (y) can be calculated, as well as H (The jacobian).
*/
StateToSensorMatrix EKF::hCalc(){
	StateToSensorMatrix h;
	SensorVector col0, col1, col2, col3;
	col0 << 1, 0, 0, 0, 0;
	col1 << 0, 1, 0, 0, 0;
	col2 << SensorVector::Zero();
	col3 << SensorVector::Zero();
	h << col0, col1, col2, col3;

	return h;
}

// StateToSensorMatrix EKF::HCalc(StateVector X, StateVector previous_X, StateToSensorMatrix h, StateToSensorMatrix previous_h){
// // 	Matrix<double, STATE_DIMS, STATE_DIMS/2> bigH;
// // 	StateToSensorMatrix H;
// // 	Matrix<double, STATE_DIMS/2, STATE_DIMS> tempZero = Matrix<double, STATE_DIMS/2, STATE_DIMS>::Zero();

// // 	//assume the state vector is <x, y, z, Vx, Vy, Vz>
// // 	double x = previous_X(0,0);
// // 	double y = previous_X(1,0);
// // 	double z = previous_X(2,0);
// // 	double coordinate3D = (double)pow(x, 2) + (double)pow(y,2) + (double)pow(z, 2);
// // 	double coordinate2D = (double)pow(x, 2) + (double)pow(y,2);
// // 	double aBigOperation = sqrt(1 - pow(z, 2)/(coordinate3D));

// // 	//temporary vectors
// // 	Matrix<double, STATE_DIMS/2, STATE_DIMS/2> a1;
// // 	Matrix<double, STATE_DIMS/2, STATE_DIMS/2> a2 = Matrix<double, STATE_DIMS/2, STATE_DIMS/2>::Zero();
// // 	Matrix<double, STATE_DIMS/2, STATE_DIMS/2> a3;
// // 	Matrix<double, STATE_DIMS/2, STATE_DIMS/2> a4 = Matrix<double, STATE_DIMS/2, STATE_DIMS/2>::Zero();
// // 	Matrix<double, STATE_DIMS/2, STATE_DIMS/2> a5;
// // 	Matrix<double, STATE_DIMS/2, STATE_DIMS/2> a6 = Matrix<double, STATE_DIMS/2, STATE_DIMS/2>::Zero();;
	
// // 	a1(0, 0) = x/sqrt(coordinate3D);
// // 	a1(1, 0) = -y/sqrt(coordinate2D);
// // 	a1(2, 0) = (x*y)/(pow(coordinate3D, (double)3.0/2) * aBigOperation);

// // 	a3(0, 2) = y/sqrt(coordinate3D);
// // 	a3(1, 2) = x/sqrt(coordinate2D);
// // 	a3(2, 2) = (y*z)/(pow(coordinate3D, (double)3.0/2) * aBigOperation);

// // 	a5(0, 4) = z/sqrt(coordinate3D);
// // 	a5(1, 4) = 0;
// // 	a5(2, 4) = ((-1/coordinate3D) + pow(z, 2)/pow(coordinate3D, (double)(3.0/2) ))/(aBigOperation);

// // 	bigH << a1, a2, a3, a4, a5, a6;

// // 	H << bigH.transpose(), tempZero;
// 	StateToSensorMatrix H;
// 	StateToSensorMatrix dh = h - previous_h;
// 	StateVector dX = X - previous_X;

// 	for(int j=0; j<SENSOR_DIMS; j++){
// 		for(int k=0; k<STATE_DIMS; k++){
// 			if(abs(dX(k,0)) > .000001){
// 				H(j,k) = dh(j,k) / dX(k,0);
// 			}
// 		}
// 	}

// 	return H;

// }

SensorVector EKF::yUpdate(StateVector X){
	SensorVector hX = (h * X);
	return *sensorInput - hX;
}

SensorToStateMatrix EKF::KUpdate(SquareStateMatrix P, StateToSensorMatrix h){
	SquareSensorMatrix S = (h * P * h.transpose() + R);

	if(S.determinant() != 0.0){
		SensorToStateMatrix K = (P * h.transpose() * S.inverse());
		return K;
	}

	SensorToStateMatrix I = SensorToStateMatrix::Zero();
	return I;
}

StateVector EKF::XUpdate(StateVector X, SensorToStateMatrix K, SensorVector y){
	StateVector XUpdate = (X + K * y);
	std::cout << "X-update: " << std::endl << XUpdate << std::endl;

	return XUpdate;
}

SquareStateMatrix EKF::PUpdate(SquareStateMatrix P, SensorToStateMatrix K, StateToSensorMatrix H){
	SquareStateMatrix I = SquareStateMatrix::Identity();
	return (I - K*h) * P;
}

/*
  Prediction step, X_k|k-1 and P_k|k-1 are predicted based on previous updated values.
*/	  
void EKF::predict(double t_current){
	dt = t_current - t_previous;
	t_previous = t_current;
	// previous_XUpdate = X;
	// previous_f = f;
	// previous_h = h;
	f = fUpdate();
	X = XPredict(previous_XUpdate);
	// F = FCalc(previous_XUpdate, previous_f);
	// P = PPredict(P, F);
	P = PPredict(P, f);
}

/*
  Update step, X_k|k, y_k, K_k, P_k|k are all updated based on obsevation.
*/
void EKF::update(){
	h = hCalc();
	// H = HCalc(X, previous_XPredict, h, previous_h);
	// previous_XPredict = X;
	y = yUpdate(X);
	K = KUpdate(P, h);
	X = XUpdate(X, K, y);
	P = PUpdate(P, K, h);
	previous_XUpdate = X;
	// K = KUpdate(P, H);
	// X = XUpdate(X, K, y);
	// P = PUpdate(P, K, H);
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
