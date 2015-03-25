import rospy

param_names = ["/joystick/drive_mode",
               "/joystick/arm_base_mode",
               "/joystick/end_effector_mode",
               "/joystick/point_steer",
               "/joystick/coord_system",
               "/joystick/camera/arm",
               "/joystick/camera/pantilt",
               "/joystick/camera/haz_front",
               "/joystick/camera/haz_back",
               "/joystick/camera/haz_right",
               "/joystick/camera/haz_left"]


class ProfileParser():
    def __init__(self, controller):
        self.controller = controller
        self.param_value = {}
        self.mapping = {}
        for param in param_names:
            self.mapping[param] = rospy.get_param(param)

    def update_values(self):
        for param in param_names:
            value = self.mapping[param]
            if value == "1":
                self.param_value[param] = self.controller.b1
            elif value == "2":
                self.param_value[param] = self.controller.b2 
            elif value == "3":
                self.param_value[param] = self.controller.b3 
            elif value == "4":
                self.param_value[param] = self.controller.b4 
            elif value == "5":
                self.param_value[param] = self.controller.b5 
            elif value == "6":
                self.param_value[param] = self.controller.b6 
            elif value == "7":
                self.param_value[param] = self.controller.b7 
            elif value == "8":
                self.param_value[param] = self.controller.b8 
            elif value == "9":
                self.param_value[param] = self.controller.b9 
            elif value == "10":
                self.param_value[param] = self.controller.b10 
            elif value == "11":
                self.param_value[param] = self.controller.b11 
            elif value == "12":
                self.param_value[param] = self.controller.b12
            elif value == "hat_top":
                self.param_value[param] = self.controller.hat_top
            elif value == "hat_left":
                self.param_value[param] = self.controller.hat_left
            elif value == "hat_right":
                self.param_value[param] = self.controller.hat_right
            elif value == "hat_down":
                self.param_value[param] = self.controller.hat_down
