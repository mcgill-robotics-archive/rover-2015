set(CMAKE_TOOLCHAIN_FILE cmake/ArduinoToolchain.cmake)

cmake_minimum_required(VERSION 2.8.3)
project(arduino-test C CXX)
if(linux)
    set(ROS_LIB_DIR /home/david/rover/catkin_ws/build/arduino_test/ros_lib)
    #set(ROS_LIB_DIR /home/david/sketchbook/libraries/ros_lib)
elseif(osx)
    set(ROS_LIB_DIR /Users/David/VMs/SharedFolder/rover/catkin_ws/build/arduino_test/ros_lib)
    message(WARNING "Mac OS X Plateform")
endif()

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

add_subdirectory(OTS_Drive)
add_subdirectory(calibrateMegaServo)
add_subdirectory(LidarTable)