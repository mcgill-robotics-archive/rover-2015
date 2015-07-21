generate_arduino_firmware(drive
    SRCS OTS_Drive.cpp ${ROS_LIB_DIR}/time.cpp SteeringControl.cpp DriveControl.cpp DataControl.cpp CameraControl.cpp
    HDRS pins.h SteeringControl.h DriveControl.h DataControl.h CameraControl.h
    BOARD mega2560
    PORT /dev/ttyACM0
)