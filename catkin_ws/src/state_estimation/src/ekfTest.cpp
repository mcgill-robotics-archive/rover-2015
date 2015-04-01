#include <iostream>
//#include <Eigen/Dense>
#include "newEKF.h"
//#include "ekf.h"
using namespace std;
using Eigen::MatrixXd;

//typedef Matrix<double, SENSOR_DIMS, SENSOR_DIMS> SquareSensorMatrix;

int main()
{
  	int i, j;
  	SquareSensorMatrix matrix1;
  	SquareStateMatrix matrix2;
	SensorVector *z;
  	//Matrix<double, 100,100> matrix2;

	matrix1(0, 0) = 123;

	matrix2(0, 0) = 12312412;

	SensorVector y; 

	y(0, 0) = 45678910;
	
	z = &y;

	EKF ekf(&y, matrix1, matrix2, matrix2);






	MatrixXd m(3,3);
	m(0,0) = 3;
	m(1,0) = 2.5;
	m(0,1) = -1;
	m(1,1) = m(1,0) + m(0,1);
	cout << *z << std::endl;
	//cout << *ekf.z << endl;
	y(1, 0) = 222;
	//z = &y;
	cout << *ekf.z << endl;

	return 0;
}