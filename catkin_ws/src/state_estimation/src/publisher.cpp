#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <rover_msgs/GPS.h>
#include "ekf.h"

using namespace ekf;

ros::Publisher pub;
ros::Subscriber sub;
EKF* filter = new EKF();
SensorVector *sensorInput;

void initialize(){
	sensorInput = new SensorVector(SensorVector::Zero());
	std::cout << "test2" << std::endl;
	SquareStateMatrix P = SquareSensorMatrix::Zero();
	SquareStateMatrix Q = SquareStateMatrix::Zero();
	SquareSensorMatrix R = SquareSensorMatrix::Zero();
	filter = new EKF(sensorInput, P, Q, R);
}

void gpsCallback(const rover_msgs::GPS::ConstPtr& GPS) {
	filter->predict();
	std::cout << filter->getP();	
}

int main (int argc, char **argv){
	ros::init(argc, argv, "ekf");
	initialize();
	ros::NodeHandle node;
    pub = node.advertise<geometry_msgs::Twist>("ekf", 100);
    sub = node.subscribe("raw_gps", 100, gpsCallback);
    ros::spin();
	return 0;
}
