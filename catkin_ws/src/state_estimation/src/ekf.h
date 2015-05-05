/*
  Version 0.1
 */

#ifndef EKF_HEADER
#define EKF_HEADER

#include <Eigen/Dense>

namespace ekf{

using namespace Eigen;

const int SENSOR_DIMS = 6;
const int STATE_DIMS = 6;

typedef Matrix<double, SENSOR_DIMS, SENSOR_DIMS> SquareSensorMatrix;
typedef Matrix<double, STATE_DIMS, STATE_DIMS> SquareStateMatrix;
typedef Matrix<double, SENSOR_DIMS, 1> SensorVector;
typedef Matrix<double, STATE_DIMS, 1> StateVector;
typedef Matrix<double, SENSOR_DIMS, STATE_DIMS> StateToSensorMatrix;
typedef Matrix<double, STATE_DIMS, SENSOR_DIMS> SensorToStateMatrix;

class EKF{
public:

	EKF(SensorVector *sensorInput, const SquareStateMatrix P, const SquareStateMatrix Q, const SquareStateMatrix R){
		this->sensorInput = sensorInput;
		this->P = P;
		this->Q = Q;
		this->R = R;
		t_previous = 0;
		dt = 0;
	}

	EKF(){
		std::cout << "ekf initialized with 0-arg constructor" << std::endl;
		P = SquareStateMatrix::Zero();
		Q = SquareStateMatrix::Zero();
		R = SquareSensorMatrix::Zero();
		t_previous = 0;
		dt = 0;
	}
	
	/*
	  Prediction step, X_k|k-1 and P_k|k-1 are predicted based on previous updated values.
	 */	  
	void predict(){
		dt = ros::Time::now().toSec() - t_previous;
		t_previous = ros::Time::now().toSec();
		f = fUpdate(dt);
		F = FCalc(X);
		X = XPredict(X);
		P = PPredict(P, F);
	}

	/*
	  Update step, X_k|k, y_k, K_k, P_k|k are all updated based on obsevation.
	 */
	void update(){
		H = HCalc(X);
		y = yUpdate(X);
		K = KUpdate(P, H);
		X = XUpdate(X, K, y);
		P = PUpdate(P, K, H);
	}

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

private:

	/*
	  Prediction-step functions
	 */
	SquareStateMatrix fUpdate(double dt);
	SquareStateMatrix FCalc(StateVector previous_X); // Calculate the updated Covariance Transition Matrix (Jacobian)
	StateVector XPredict(StateVector previous_X);	// Predict the next State based on f (transition) and previous X
	SquareStateMatrix PPredict(SquareStateMatrix previous_P, SquareStateMatrix F); // Predict the next Covariance Matrix based on F and previous P


	/*
	  Update-step functions
	 */
	StateToSensorMatrix HCalc(StateVector X); // Calculate the Measurement Transformation Matrix (Jacobian)
	SensorVector yUpdate(StateVector X);	// Update y based on current z (measurement) and X (state)
	SensorToStateMatrix KUpdate(SquareStateMatrix P, StateToSensorMatrix H); // Update the Kalman Gain, based on the estimated P, H and R
	StateVector XUpdate(StateVector X, SensorToStateMatrix K, SensorVector y); // Update the State estimate based on K and y
	SquareStateMatrix PUpdate(SquareStateMatrix P, SensorToStateMatrix K, StateToSensorMatrix H); // Update the Error Covariance Estimate given K and H
	

	/*
	  Filter Variables
	*/
	StateVector X;
	SquareStateMatrix f;

	SquareStateMatrix P;
	SensorToStateMatrix K;	


	/*
	  Observation variables
	*/
	SensorVector y;
	SensorVector *sensorInput;
	StateToSensorMatrix h;

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

};
#endif
