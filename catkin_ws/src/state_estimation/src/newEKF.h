#ifndef EKF_HEADER
#define EKF_HEADER

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

SensorVector *sensorInput;

class EKF
{
public:
	

	EKF(SensorVector *sensorInput, const SquareStateMatrix P, const SquareStateMatrix Q, const SquareStateMatrix R){
		this->sensorInput = sensorInput; // Copy-by-value the input initial P, Q, and R matrices into the EKF
		this->P = P;
		this->Q = Q;
		this->R = R;
	};
	SensorVector *sensorInput;
	SquareStateMatrix P;
	SquareStateMatrix Q;
	SquareSensorMatrix R;

	void predict(){
		F = FCalc();
		X = XPredict(X);
		P = PPredict(P, F);
	};

	void update(){
		H = HCalc();
		//y = yUpdate(sensorInput,X);
		K = KUpdate();
		X = XUpdate();
		P = PUpdate();
	};



	const SquareStateMatrix f;
	SquareStateMatrix F;
	StateVector X;
	SensorToStateMatrix K;	

	SensorVector y;
	const StateToSensorMatrix h;
	StateToSensorMatrix H;	

	SensorVector zSetter(double x, double y, double z, double v1, double v2, double v3);

public:
//private:

	SquareStateMatrix FCalc(); // Calculate the updated Covariance Transition Matrix (Jacobian)
	StateVector XPredict(StateVector &previous_X);	// Predict the next State based on f (transition) and previous X
	SquareStateMatrix PPredict(SquareStateMatrix &previous_P, SquareStateMatrix &F); // Predict the next Covariance Matrix based on F and previous P

	StateToSensorMatrix HCalc(); // Calculate the Measurement Transformation Matrix (Jacobian)
	
	SensorVector yUpdate(SensorVector *sensorInput, StateVector &X);	// Update y based on current z (measurement) and X (state)
	SensorToStateMatrix KUpdate(); // Update the Kalman Gain, based on the estimated P, H and R
	StateVector XUpdate(); // Update the State estimate based on K and y
	SquareStateMatrix PUpdate(); // Update the Error Covariance Estimate given K and H



}; // end of the class




#endif