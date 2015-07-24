generate_arduino_firmware(drive
    SRCS OTS_Drive.cpp ${ROS_LIB_DIR}/time.cpp SteeringControl.cpp DriveControl.cpp DataControl.cpp Camera.cpp
    HDRS pins.h SteeringControl.h DriveControl.h DataControl.h Camera.h
    BOARD mega2560
    PORT /dev/mega_drive
)
