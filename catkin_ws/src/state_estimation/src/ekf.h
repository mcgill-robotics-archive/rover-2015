/*
  Version 0.1
 */

#ifndef EKF_HEADER
#define EKF_HEADER

#include <Eigen/Dense>

namespace ekf{

using namespace Eigen;

const int SENSOR_DIMS = 5;
const int STATE_DIMS = 4;
const int INPUT_DIMS = 2;

typedef Matrix<double, SENSOR_DIMS, SENSOR_DIMS> SquareSensorMatrix;
typedef Matrix<double, STATE_DIMS, STATE_DIMS> SquareStateMatrix;
typedef Matrix<double, SENSOR_DIMS, 1> SensorVector;
typedef Matrix<double, STATE_DIMS, 1> StateVector;
typedef Matrix<double, SENSOR_DIMS, STATE_DIMS> StateToSensorMatrix;
typedef Matrix<double, STATE_DIMS, SENSOR_DIMS> SensorToStateMatrix;
typedef Matrix<double, INPUT_DIMS, 1> InputVector;
typedef Matrix<double, STATE_DIMS, INPUT_DIMS> InputToStateMatrix;

class EKF{
public:

	EKF(SensorVector *sensorInput, const SquareStateMatrix P, const SquareStateMatrix Q, const SquareSensorMatrix R, double t_current){
		std::cout << "ekf initialized with 5-arg constructor" << std::endl;
		this->sensorInput = sensorInput;
		this->P = P;
		this->Q = Q;
		this->R = R;
		X = StateVector::Zero();
		F = SquareStateMatrix::Identity();
		f = SquareStateMatrix::Identity();
		H = StateToSensorMatrix::Identity();
		y = SensorVector::Ones();
		K = SensorToStateMatrix::Zero();
		h = StateToSensorMatrix::Identity();
		B = InputToStateMatrix::Zero(); B(2,0) = 1;
		t_previous = t_current;
		dt = 0;
	}

	EKF(){
		P = SquareStateMatrix::Identity();
		Q = SquareStateMatrix::Identity();
		R = SquareSensorMatrix::Identity();
		t_previous = 0;
		dt = 0;
	}
	
	/*
	  Prediction step, X_k|k-1 and P_k|k-1 are predicted based on previous updated values.
	 */	  
	void predict(double t_current);

	/*
	  Update step, X_k|k, y_k, K_k, P_k|k are all updated based on obsevation.
	 */
	void update();
	
	SquareStateMatrix getF(){
		return F;
	}

	StateVector getX(){
		return X;
	}

	SquareStateMatrix getP(){
		return P;
	}

	StateToSensorMatrix getH(){
		return H;
	}

	SensorVector getY(){
		return y;
	}

	SensorToStateMatrix getK(){
		return K;
	}
	
	void setu(double linearSpd, double angularSpd){
		u << linearSpd, angularSpd;
	}
private:

	/*
	  Prediction-step functions
	 */
	SquareStateMatrix fUpdate();
	SquareStateMatrix FCalc(StateVector previous_X, SquareStateMatrix previous_f); // Calculate the updated Covariance Transition Matrix (Jacobian)
	StateVector XPredict(StateVector previous_X);	// Predict the next State based on f (transition) and previous X
	SquareStateMatrix PPredict(SquareStateMatrix previous_P, SquareStateMatrix F); // Predict the next Covariance Matrix based on F and previous P


	/*
	  Update-step functions
	 */
	StateToSensorMatrix hCalc();
	StateToSensorMatrix HCalc(StateVector X, StateVector previous_X, StateToSensorMatrix h, StateToSensorMatrix previous_h); // Calculate the Measurement Transformation Matrix (Jacobian)
	SensorVector yUpdate(StateVector X);	// Update y based on current z (measurement) and X (state)
	SensorToStateMatrix KUpdate(SquareStateMatrix P, StateToSensorMatrix H); // Update the Kalman Gain, based on the estimated P, H and R
	StateVector XUpdate(StateVector X, SensorToStateMatrix K, SensorVector y); // Update the State estimate based on K and y
	SquareStateMatrix PUpdate(SquareStateMatrix P, SensorToStateMatrix K, StateToSensorMatrix H); // Update the Error Covariance Estimate given K and H
	

	/*
	  Filter Variables
	*/
	StateVector X;
	StateVector previous_XUpdate;
	StateVector previous_XPredict;
	
	InputVector u;
	InputToStateMatrix B;
	SquareStateMatrix f;
	SquareStateMatrix previous_f;
	
	SquareStateMatrix P;
	SensorToStateMatrix K;	

	/*
	  Observation variables
	*/
	SensorVector y;
	SensorVector *sensorInput;
	StateToSensorMatrix h;
	StateToSensorMatrix previous_h;
	/*
	  Jacobian variables (require calculation of Jacobian)
	*/
	StateToSensorMatrix H;	
	SquareStateMatrix F;

	/*
	  Noise/error variables (to be tuned)
	*/
	SquareStateMatrix Q;
	SquareSensorMatrix R;

	// Time since last update, in milliseconds
	double dt;
	double t_previous;
};


	/*
 	  Generate a matrix P holding the intial estimation covariance estimate.
	*/
SquareStateMatrix generateP(double covariance);
SquareStateMatrix generateP(double covariance[STATE_DIMS]);
	
	/*
	  Generate a matrix Q holding the estimated process-noise covariance.
	*/
SquareStateMatrix generateQ(double covariance[STATE_DIMS]);
	
	/*
	  Generate a matrix R holding the estimated measurement-noise covariance.
	*/
SquareSensorMatrix generateR(double covariance[SENSOR_DIMS]);
};
#endif
