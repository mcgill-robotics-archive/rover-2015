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

/*
  Re-initialize the filter with initial inputs
 */
void initialize(double start_time){
	sensorInput = new SensorVector(SensorVector::Ones());
	SquareStateMatrix P = generateP(1000);
	double qVals[STATE_DIMS] = {.01,.01,.05,.05,.05,.1};
	double rVals[SENSOR_DIMS] = {.9,.9,.9,.9,.9,.9};
	SquareStateMatrix Q = generateQ(qVals);
	SquareSensorMatrix R = generateR(rVals);
	
	filter = new EKF(sensorInput, P, Q, R, start_time);
	std::cout << "EKF Initialized" << std::endl;
	std::cout << filter->getX() << std::endl;
}

void gpsCallback(const rover_msgs::GPS::ConstPtr& GPS) {
	filter->predict(ros::Time::now().toSec());
	std::cout << "Predicted X: " << std::endl << filter->getX() << std::endl;
	filter->update();
	std::cout << "Updated X: " << std::endl << filter->getX() << std::endl;
}

int main (int argc, char **argv){
	ros::init(argc, argv, "ekf");
	ros::NodeHandle node;
    pub = node.advertise<geometry_msgs::Twist>("ekf", 100);
    sub = node.subscribe("raw_gps", 100, gpsCallback);
	double start_time = ros::Time::now().toSec();
	
	initialize(start_time);
	
    ros::spin();
	
	return 0;
}
