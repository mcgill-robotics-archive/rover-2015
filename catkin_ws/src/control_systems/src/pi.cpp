#include <stdio.h>
#include <iostream>
#include <time.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <ros/console.h>


int main(int argc,char *argv[])
{
	//The two parameters for the PI loop
	float kp = 1;
	float ki = 1;
	//Refresh rate of the PI loop
	int frequency = 100;
	//PI loop is v(t) = kp*error(t) + ki*integral(error(x),(x,0,t))
	//optimization will occur by changing kp, ki.
	//The sum of all the errors:
	float intError = 0;
	//The voltage to output at each loop
	//float outputVoltage = 0;
	//The current position of the motor		
	//This value would be obtained from the encoders
	float realPos;
	//The desired position of the motor
	//This value would be obtained from a ros topic directed from the user
	float pos;
	//The current error of the motor
	float error;
	//Period is time interval between refreshes
	float period = 1/( (float)frequency );

	//initialize the node
	ros::init(argc, argv, "pi_controller");
	ros::NodeHandle n;
	//number is how many messages in the buffer
	ros:: Publisher arm_voltage = n.advertise<std_msgs::Float32>("arm_voltage",100);

	//current operating frequency
	ros::Rate loop_rate(frequency);


	while (ros::ok())
	{
		std_msgs::Float32 outVolt;
		error = pos - realPos;
		intError += error*period;
		outVolt.data = kp*error + ki*intError;
		//output this value to another topic
		arm_voltage.publish(outVolt);

		//done iteration, so sleep
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}