//
// Created by david on 9/4/15.
//

#ifndef ARDUINO_TEST_LIDARTABLESERVO_H
#define ARDUINO_TEST_LIDARTABLESERVO_H


#include <ros.h>
#include "std_msgs/Int16.h"

static int currentAngle = 90;
void initROS();
void sendTransform();
#endif //ARDUINO_TEST_LIDARTABLESERVO_H
