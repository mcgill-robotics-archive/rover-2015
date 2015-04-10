#include <iostream>
//#include <Eigen/Dense>
#include "newEKF.h"
//#include "ekf.h"
using namespace std;
//using namespace ekf;
using Eigen::MatrixXd;

//typedef Matrix<double, SENSOR_DIMS, SENSOR_DIMS> SquareSensorMatrix;


SquareStateMatrix EKF::FCalc(){
	int row, column;
	//set up the F = 
	SquareStateMatrix F;
	for (row = 0; row< SENSOR_DIMS; row++) {
		for (column = 0; column < SENSOR_DIMS; column++) {
			if (row == 0 && column == 0) {
				F(row, column) = 2; 
			}
			else if (row == 1 && column == 1) {
				F(row, column) = 2;
			}
			else if (row == 2 && column == 2) {
				F(row, column) = 2;
			}
			else {
				F(row, column) = 0;
			}
		}
	}
	cout << F << endl;
	return F;
}


int main()
{
  	int i, j;
  	SquareSensorMatrix matrix1;
  	SquareStateMatrix matrix2;
	SensorVector *z;
  	//Matrix<double, 100,100> matrix2;

	matrix1(0, 0) = 123;

	matrix2(0, 0) = 12312412;

	//cout << matrix1 << endl;

	SensorVector y; 

	y(0, 0) = 45678910;
	
	z = &y;

	EKF e(&y, matrix1, matrix2, matrix2);

	y(1, 0) = 222;


	//cout << *z << std::endl;
	//cout << *ekf.z << endl;
	//y(1, 0) = 222;
	z = &y;
	//cout << *ekf.z << endl;

	e.FCalc();
	//FCalc();
	return 0;
}