#ifndef EKF
#define EKF

#include <Eigen/Dense>
using namespace Eigen;

const int SENSOR_DIMS = 6;
const int STATE_DIMS = 6;

typedef Matrix<double, SENSOR_DIMS, SENSOR_DIMS> SquareSensorMatrix;
typedef Matrix<double, STATE_DIMS, STATE_DIMS> SquareStateMatrix;
typedef Matrix<double, SENSOR_DIMS, 1> SensorVector;
typedef Matrix<double, STATE_DIMS, 1> StateVector;
typedef Matrix<double, SENSOR_DIMS, STATE_DIMS> StateToSensorMatrix;
typedef Matrix<double, STATE_DIMS, SENSOR_DIMS> SensorToStateMatrix;

class ekf
{
public:
	inline ekf(SensorVector z, const SquareStateMatrix P, const SquareStateMatrix Q, const SquareStateMatrix R){
		this.z = z;												   // Point the z measurement matrix to a supplied measurement matrix
		this.P = new SquareStateMatrix(P);						   // Copy-by-value the input initial P, Q, and R matrices into the EKF
		this.Q = new SquareStateMatrix(Q);
		this.R = new SquareSensorMatrix(R);
	};

	/*
	  Prediction step, X_k|k-1 and P_k|k-1 are predicted based on previous updated values.
	 */	  
	inline void predict(){
		F = FCalc();
		X = XPredict();
		P = PPredict():
	};

	/*
	  Update step, X_k|k, y_k, K_k, P_k|k are all updated based on obsevation.
	 */
	inline void update(){
		H = HCalc();
		y = yUpdate();
		K = KUpdate();
		X = XUpdate();
		P = PUpdate();
	};

private:

	/*
	  Prediction-step functions
	 */
	SquareStateMatrix FCalc();									   // Calculate the updated Covariance Transition Matrix (Jacobian)
	StateVector XPredict();										   // Predict the next State based on f (transition) and previous X
	SquareStateMatrix PPredict();								   // Predict the next Covariance Matrix based on F and previous P


	/*
	  Update-step functions
	 */
	StateToSensorMatrix HCalc();								   // Calculate the Measurement Transformation Matrix (Jacobian)
	SensorVector yUpdate();										   // Update y based on current z (measurement) and X (state)
	SensorToStateMatrix KUpdate();								   // Update the Kalman Gain, based on the estimated P, H and R
	StateVector XUpdate();										   // Update the State estimate based on K and y
	SquareStateMatrix PUpdate();								   // Update the Error Covariance Estimate given K and H
	

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
	SensorVector z;
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
}


#endif
