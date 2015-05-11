#include <stdio.h>
#include <iostream>
#include <time.h>

using namespace std;

int main()
{
	//The two parameters for the PI loop
	float kp = 1;
	float ki = 1;
	//PI loop is v(t) = kp*error(t) + ki*integral(error(x),(x,0,t))
	//optimization will occur by changing kp, ki.
	//The sum of all the errors:
	float intError = 0;
	//The voltage to output at each loop
	float outputVoltage = 0;
	//The current position of the motor		
	float realPos;
	//The desired position of the motor
	float pos;
	//The current error of the motor
	float error;

	while (TRUE)
	{

	}
	return 0;
}