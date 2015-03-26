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

class ekf
{
	ekf();
	void predict();
	void update();

	StateVector x;
	SquareSensorMatrix P;
	SquareStateMatrix k;
	SquareStateMatrix f;
	SquareSensorMatrix F;
	SensorVector y;
	
	

}


#endif
