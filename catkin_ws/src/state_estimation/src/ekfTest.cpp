#include <iostream>
//#include <Eigen/Dense>
#include "newEKF.h"
//#include "ekf.h"
#include <ros/ros.h>

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
	//cout << F << endl;
	return F;
}

StateVector EKF::XPredict(StateVector &previous_X){
	//set up the small f with 1 and delta t 
	SquareStateMatrix small_f;





	return f * previous_X;
}

SquareStateMatrix EKF::PPredict(SquareStateMatrix &previous_P, SquareStateMatrix &F){
	
	//set up Q --> we can adjust the values accordingly 
	cout << "HELLO WORLD\n";
	SquareStateMatrix Q;
	int row; 
	int column;
	for (row = 0; row < STATE_DIMS; row++) {
		for (column = 0; column < STATE_DIMS; column++) {
			if (row == column) {
				Q(row, column) = 0.0000001;
			}
			else {
				Q(row, column) = 0;
			}
		}
	}
	cout << F * previous_P * F.transpose() + Q << endl;
	return F * previous_P * F.transpose() + Q;
}

void zSetter(double x, double y, double z, double v1, double v2, double v3) {


	SensorVector sensor; 

	sensor(0, 0) = x;
	sensor(1, 0) = y;
	sensor(2, 0) = z;
	sensor(3, 0) = v1;
	sensor(4, 0) = v2;
	sensor(5, 0) = v3;

	sensorInput = &sensor;

	//cout << *sensorInput << endl;
	
}



SensorVector EKF::yUpdate(SensorVector *sensorInput, StateVector &X) {
	
	//cout << "Hello world\n";
	SensorVector y;
	SensorVector z; 
	z = *sensorInput;

	int row;
	int column;
	//set up a sample matrix for h
	SquareSensorMatrix h;
	for (row = 0; row < SENSOR_DIMS; row++) {
		for (column = 0; column < STATE_DIMS; column++) {
			if (row == 0 && column == 0) {
				h(row, column) = 1;
			}
			else if (row == 1 && column == 1) {
				h(row, column) = 1;
			}
			else if (row == 2 && column == 2) {
				h(row, column) = 1;
			}
			else {
				h(row, column) = 0;
			}
		}
	}

	cout << h << endl;

	y =  z- h * X;


	return y;

}





int main()
{
  	int row, column;
  	SquareSensorMatrix p;
  	SquareStateMatrix matrix2;
	SensorVector *z;
	SquareStateMatrix bigF;
	SquareStateMatrix *Fptr;
	SquareStateMatrix *ptr;
	StateVector x;
	SensorVector y; 

	//set up a simple P
	for (row = 0; row < STATE_DIMS; row++) {
	//	cout << "ENTER HERE\n";
		x (row, 0) = 10;
		y (row, 0) = 112;
		for (column = 0; column < STATE_DIMS; column++) {
			if (row == column) {
				p(row, column) = 1000;
			}
			else {
				p(row, column) = 0;
			}
		}
	}

	//cout << p << endl;

	
	
	z = &y;


	EKF e(&y, p, matrix2, matrix2);

	zSetter(100000, 2, 3, 4, 5, 6);


	e.yUpdate(z, x);


	return 0;
}