generate_arduino_firmware(lidar
    SRCS LidarTable.cpp ${ROS_LIB_DIR}/time.cpp main.cpp
    HDRS LidarTable.h
    BOARD mega2560
    PORT /dev/ttyACM0
)
