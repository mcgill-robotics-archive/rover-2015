generate_arduino_firmware(lidar_servo
    SRCS ${ROS_LIB_DIR}/time.cpp main.cpp
#    HDRS LidarTableServo.h
    BOARD mega2560
    PORT /dev/ttyACM1
)