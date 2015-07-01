set(CMAKE_TOOLCHAIN_FILE cmake/ArduinoToolchain.cmake)

cmake_minimum_required(VERSION 2.8.3)
project(arduino-test C CXX)

include_directories(${ROS_LIB_DIR})
# Remove this if using an Arduino without native USB (eg, other than Leonardo)
#add_definitions(-DUSB_CON)
message(${ROS_LIB_DIR})
message(${CMAKE_TOOLCHAIN_FILE})
print_board_list()
generate_arduino_firmware(hello
      SRCS chatter.cpp ${ROS_LIB_DIR}/time.cpp 
        BOARD mega2560
          PORT /dev/ttyACM0
          )
