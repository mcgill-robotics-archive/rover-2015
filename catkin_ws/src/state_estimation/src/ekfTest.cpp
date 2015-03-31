#include <iostream>
#include <Eigen/Dense>
#include "newEKF.h"
//#include "ekf.h"

using Eigen::MatrixXd;

//typedef Matrix<double, SENSOR_DIMS, SENSOR_DIMS> SquareSensorMatrix;

int main()
{
  	int i, j;
  	SquareSensorMatrix matrix1;
  	SquareStateMatrix matrix2;
	//SensorVector z;
	StateVector matrix4;
	StateToSensorMatrix matrix5;
	SensorToStateMatrix matrix6;
  	//Matrix<double, 100,100> matrix2;

	for (i = 0; i < 6; i++) {
		for (j = 0; j < 6; j++) {
			matrix1(i,j) = 1;
		}
	}
	


	MatrixXd m(3,3);
	m(0,0) = 3;
	m(1,0) = 2.5;
	m(0,1) = -1;
	m(1,1) = m(1,0) + m(0,1);
	std::cout << matrix1 << std::endl;

	return 0;
}