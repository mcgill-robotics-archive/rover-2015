generate_arduino_firmware(drive
    SRCS OTS_Drive.cpp ${ROS_LIB_DIR}/time.cpp
    HDRS pins.h
    BOARD mega2560
    PORT /dev/ttyACM0
)