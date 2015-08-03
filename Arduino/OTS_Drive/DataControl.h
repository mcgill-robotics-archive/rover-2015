//
// Created by David Lavoie-Boutin on 15-07-01.
//

#ifndef ARDUINO_TEST_DATACONTROL_H
#define ARDUINO_TEST_DATACONTROL_H

#include "rover_msgs/MotorStatus.h"

namespace data
{
    void setModeOpenLoop();
    void setModeLowSpeed();
    void setModeMediumSpeed();
    void setModeHighSpeed();

    void sendMotorStatus(ros::Publisher &publisher);
}



#endif //ARDUINO_TEST_DATACONTROL_H
