/*
 * File: MW_odo.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 18-Dec-2014 12:35:00
 */

#ifndef __MW_ODO_H__
#define __MW_ODO_H__
#include "odometry/DisplacementMiddleWheels.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
/* Function Declarations */

double rLeft, rRight, whBase;

bool MW_odo(odometry::DisplacementMiddleWheels::Request  &req,
            odometry::DisplacementMiddleWheels::Response &res);
#endif

/*
 * File trailer for MW_odo.h
 *
 * [EOF]
 */
