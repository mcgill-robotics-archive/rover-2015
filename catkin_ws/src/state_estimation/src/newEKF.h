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
	inline ekf(SensorVector z, const SquareStateMatrix P, const SquareStateMatrix Q, const SquareSensorMatrix R){
		this->z = z;												   // Point the z measurement matrix to a supplied measurement matrix
	 	this->P = P;						   // Copy-by-value the input initial P, Q, and R matrices into the EKF
		this->Q = Q;
		this->R = R;
	};

		SensorVector z;
		SquareStateMatrix P;
		SquareStateMatrix Q;
		SquareSensorMatrix R;
};





#endif