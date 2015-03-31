/*
  Version 0.1
 */

#ifndef EKF
#define EKF

#include "include/Eigen/Dense"
#include <Eigen/Dense>
using namespace Eigen;

namespace ekf{

const int SENSOR_DIMS = 3;
const int STATE_DIMS = 6;

typedef Matrix<double, SENSOR_DIMS, SENSOR_DIMS> SquareSensorMatrix;
typedef Matrix<double, STATE_DIMS, STATE_DIMS> SquareStateMatrix;
typedef Matrix<double, SENSOR_DIMS, 1> SensorVector;
typedef Matrix<double, STATE_DIMS, 1> StateVector;
typedef Matrix<double, SENSOR_DIMS, STATE_DIMS> StateToSensorMatrix;
typedef Matrix<double, STATE_DIMS, SENSOR_DIMS> SensorToStateMatrix;

class EKF
{
public:
	EKF(SensorVector *z, const SquareStateMatrix &P, const SquareStateMatrix &Q, const SquareStateMatrix &R){
		this->z = z; // Copy-by-value the input initial P, Q, and R matrices into the EKF
		this.P = P;
		this.Q = Q;
		this.R = R;
	};

	/*
	  Prediction step, X_k|k-1 and P_k|k-1 are predicted based on previous updated values.
	 */	  
	void predict(){
		F = FCalc();
		X = XPredict(X);
		P = PPredict(P, F):
	};

	/*
	  Update step, X_k|k, y_k, K_k, P_k|k are all updated based on obsevation.
	 */
	void update(){
		H = HCalc();
		y = yUpdate(X);
		K = KUpdate();
		X = XUpdate();
		P = PUpdate();
	};

private:

	/*
	  Prediction-step functions
	 */
	SquareStateMatrix FCalc(); // Calculate the updated Covariance Transition Matrix (Jacobian)
	StateVector XPredict(StateVector &previous_X);	// Predict the next State based on f (transition) and previous X
	SquareStateMatrix PPredict(SquareStateMatrix &previous_P, SquareStateMatrix &F); // Predict the next Covariance Matrix based on F and previous P


	/*
	  Update-step functions
	 */
	StateToSensorMatrix HCalc(); // Calculate the Measurement Transformation Matrix (Jacobian)
	SensorVector yUpdate(StateVector &X);	// Update y based on current z (measurement) and X (state)
	SensorToStateMatrix KUpdate(); // Update the Kalman Gain, based on the estimated P, H and R
	StateVector XUpdate(); // Update the State estimate based on K and y
	SquareStateMatrix PUpdate(); // Update the Error Covariance Estimate given K and H
	

	/*
	  Filter Variables
	*/
	StateVector X;
	const SquareStateMatrix f;

	SquareStateMatrix P;
	SensorToStateMatrix K;	


	/*
	  Observation variables
	*/
	SensorVector y;
	SensorVector *z;
	const StateToSensorMatrix h;

	/*
	  Jacobian variables (require calculation of Jacobian)
	*/
	StateToSensorMatrix H;	
	SquareStateMatrix F;

	/*
	  Noise/error variables (to be tuned)
	*/
	const SquareStateMatrix Q;
	const SquareSensorMatrix R;	
};

}
#endif
