#ifndef EKF
#define EKF

using namespace Eigen;

const int SENSOR_DIMS = 6;
const int STATE_DIMS = 6;

typedef Matrix<double, SENSOR_DIMS, SENSOR_DIMS> SquareSensorMatrix;
typedef Matrix<double, STATE_DIMS, STATE_DIMS> SquareStateMatrix;


class ekf
{
	ekf();
	void predict();
	void update();

	Matrix<double, STATE_DIMS, 1> x;
	Matrix6x6 P;
	Matrix<double, SENSOR_DIMS, 1> k;
	Matrix<double, STATE_DIMS, STATE_DIMS> f;
	Matrix<double, SENSOR_DIMS, SENSOR_DIMS> F;
	Matrix<double, SENSOR_DIMS, 1> y;
	

}


#endif
