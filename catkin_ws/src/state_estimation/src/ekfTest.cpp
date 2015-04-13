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



int main()
{
  	int row, column;
  	SquareSensorMatrix p;
  	SquareStateMatrix matrix2;
	SensorVector *z;
	SquareStateMatrix bigF;
	SquareStateMatrix *Fptr;
	SquareStateMatrix *ptr;
	
	//set up a simple P
	for (row = 0; row < STATE_DIMS; row++) {
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

	SensorVector y; 
	y(0, 0) = 45678910;
	z = &y;


	EKF e(&y, p, matrix2, matrix2);


	e.predict();
	//bigF = e.FCalc();

	//Fptr = &bigF;
	//ptr = &p;
	//SquareStateMatrix Pfinal;
	//Pfinal = e.PPredict(p, bigF);
	
	//cout<< bigF.transpose() << endl;




	return 0;
}