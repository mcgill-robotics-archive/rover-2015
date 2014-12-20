#include "ros/ros.h"
#include "odometry.h"
#include "nav_msgs/Odometry.h"
#include "odometry/Tachometers.h"
#include "boost/date_time/posix_time/posix_time.hpp"

void callback(odometry::Tachometers tachoCounts) {
	double diffMiddleLeft;
  double diffMiddleRight;
  double diffMiddleCenter;
  double diffTheta;

  double pi = 3.1415926535897931;

  namespace pt = boost::posix_time;
	pt::ptime current = pt::microsec_clock::local_time();

	pt::time_duration duration = current - lastOut;
	lastOut = current;


  diffMiddleLeft = radiusLeft * pi *  (tachoCounts.tachoML - previousTachoMiddleLeft);
  diffMiddleRight = radiusRight * pi * (tachoCounts.tachoMR - previousTachoMiddleRight);
  diffMiddleCenter = (diffMiddleLeft + diffMiddleRight) / 2.0;

  diffTheta = (diffMiddleRight - diffMiddleLeft) / wheelBase;
  
  heading = fmod(heading + diffTheta, 2 * pi);
  
  x += diffMiddleCenter * cos(heading);
  y += diffMiddleCenter * sin(heading);
  
  nav_msgs::Odometry odo_msg;

  odo_msg.pose.pose.position.x = x;
  odo_msg.pose.pose.position.y = y;
  odo_msg.pose.pose.orientation.z = heading;

  double delay = duration.total_milliseconds();

  odo_msg.twist.twist.linear.x = diffMiddleCenter/(delay*1000000);
  odo_msg.twist.twist.angular.z = diffTheta/(delay*1000000);
}

int main(int argc, char **argv)
{

	ros::init(argc,argv, "odometry_publisher");

	ros::NodeHandle n;
	
	n.param<double>("wheel_radius/left/middle", radiusLeft, 0.20);
 	n.param<double>("wheel_radius/right/middle", radiusRight, 0.20);
  n.param<double>("controls/wh_base", wheelBase, 0.40);

  tachoSub = n.subscribe("/ODO_TOPIC", 1000, callback);

  wheelBase *= 2;
  heading = previousTachoMiddleLeft = previousTachoMiddleRight = 0;

	lastOut = pt::microsec_clock::local_time();

  ros::spin();

 	// subscribe to tachometer topics
 	// on reception of tacho message

	return 0;
}