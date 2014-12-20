#ifndef __ODOMETRYH__

#include "boost/date_time/posix_time/posix_time.hpp"
#include "ros/ros.h"

namespace pt = boost::posix_time;
pt::ptime lastOut;

double x, y, heading;
double wheelBase, radiusLeft, radiusRight;

int previousTachoMiddleLeft, previousTachoMiddleRight;

long int dt;

ros::Subscriber tachoSub;

#define __ODOMETRYH__ 
#endif