#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Pose2D.h>
#include <rover_msgs/GPS.h>
#include <rover_msgs/Imu.h>
#include <rover_msgs/RoverSpeed.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include "ekf.h"

# define PI 3.14159265358979


using namespace ekf;

double const IMU_RAW_TO_DEGS = 250.0 / 16384;
double const LAT_TO_METERS  = 110574;
double const LONG_TO_METERS = 111320;

ros::Publisher pub;
ros::Subscriber subGPS;
ros::Subscriber subIMU;
ros::Subscriber subINPUT;

geometry_msgs::Pose2D pose = geometry_msgs::Pose2D();
bool firstGPSUpdate = true;
double initialLatitude;
double initialLongitude;

EKF* filter = new EKF();
SensorVector *sensorInput;

/*
  Re-initialize the filter with initial inputs
 */
void initialize(double start_time){
	sensorInput = new SensorVector(SensorVector::Zero());
	SquareStateMatrix P = generateP(1000);
	double qVals[STATE_DIMS] = {1,1,5,1};
	double rVals[SENSOR_DIMS] = {.05,.05,.01,.01,.01};
	SquareStateMatrix Q = generateQ(qVals);
	SquareSensorMatrix R = generateR(rVals);

	filter = new EKF(sensorInput, P, Q, R, start_time);
	filter->setu(0, 0);
	std::cout << "EKF Initialized" << std::endl;
}

/*
  Called each time a new GPS message is received, update the sensorInput
  delta-latitude/longitude values.
*/
void gpsCallback(const rover_msgs::GPS::ConstPtr& GPS){
	ros::Duration(1).sleep();
	if(firstGPSUpdate){
		initialLatitude = GPS->latitude;
		initialLongitude = GPS->longitude;
		firstGPSUpdate = false;
	}

	
	(*sensorInput)(0) = LONG_TO_METERS * cos(GPS->latitude) * (GPS->longitude - initialLongitude); // X in meters
	(*sensorInput)(1) = LAT_TO_METERS * (GPS->latitude - initialLatitude); // y in meters

	filter->predict(ros::Time::now().toSec());
	filter->update();

	std::cout << "SensorInput: " << std::endl << *sensorInput << std::endl;

	pose.x = (filter->getX())(0);
	pose.y = (filter->getX())(1);
	pose.theta = (filter->getX())(3);

	pub.publish(pose);	
}

/*
  Called each time a new IMU message is received, update the sensorInput
  X/Y acceleration values.
*/
void imuCallback(const rover_msgs::Imu::ConstPtr& IMU){
	ros::Duration(1).sleep();
	(*sensorInput)(2) = IMU->acelX;								   // x-acceleration, m/s^2
	(*sensorInput)(3) = IMU->acelY;								   // y-acceleration, m/s^2
	(*sensorInput)(4) = IMU->gyroZ * IMU_RAW_TO_DEGS;			   // angular velocity, deg/s

	filter->predict(ros::Time::now().toSec());
	filter->update();

	std::cout << "X: " << std::endl << filter->getX() << std::endl;

	pose.x = (filter->getX())(0);
	pose.y = (filter->getX())(1);
	pose.theta = (filter->getX())(3);

	pub.publish(pose);
}


/*
  Set the input vector u (linear and angular velocity), but don't predict/update
  the filter.
*/
void inputCallback(const rover_msgs::RoverSpeed::ConstPtr& INPUT){
	filter->setu(INPUT->linear, INPUT->angular);
}

int main (int argc, char **argv){
	ros::init(argc, argv, "ekf");
	ros::NodeHandle node;

    pub = node.advertise<geometry_msgs::Pose2D>("ekf", 100);
    subGPS = node.subscribe("raw_gps", 100, gpsCallback);
	subIMU = node.subscribe("imu", 10, imuCallback);
	subINPUT = node.subscribe("odo_speeds", 10, inputCallback);

	double start_time = ros::Time::now().toSec();	
	initialize(start_time);	

	ros::Duration(2).sleep();
    ros::spin();
	
	return 0;
}
