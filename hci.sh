#!/bin/bash

export ROS_MASTER_URI=http://air:11311

rosparam load $ROS_WORKSPACE/src/hci/config/camera_topics.yaml
rosparam load $ROS_WORKSPACE/src/hci/config/joystick_profile.yaml

rosrun hci hci.py

