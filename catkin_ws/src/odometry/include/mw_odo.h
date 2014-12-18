#ifndef __MW_ODO_H__
#define __MW_ODO_H__

#include "odometry/DisplacementMiddleWheels.h"

double rLeft, rRight, whBase;

bool MW_odo(odometry::DisplacementMiddleWheels::Request  &req,
            odometry::DisplacementMiddleWheels::Response &res);
#endif

