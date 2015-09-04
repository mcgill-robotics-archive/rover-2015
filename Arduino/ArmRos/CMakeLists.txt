generate_arduino_firmware(arm
    SRCS ArmController.cpp ${ROS_LIB_DIR}/time.cpp main.cpp
    HDRS ArmController.h
    LIBS pid
    BOARD mega2560
    PORT /dev/mega_arm
)
