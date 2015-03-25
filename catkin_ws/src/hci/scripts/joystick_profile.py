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
        for param in param_names:
            value = rospy.get_param(param)
            if value is 1:
                self.param_value[param] = self.controller.b1
            elif value is 2:
                self.param_value[param] = self.controller.b2 
            elif value is 3:
                self.param_value[param] = self.controller.b3 
            elif value is 4:
                self.param_value[param] = self.controller.b4 
            elif value is 5:
                self.param_value[param] = self.controller.b5 
            elif value is 6:
                self.param_value[param] = self.controller.b6 
            elif value is 7:
                self.param_value[param] = self.controller.b7 
            elif value is 8:
                self.param_value[param] = self.controller.b8 
            elif value is 9:
                self.param_value[param] = self.controller.b9 
            elif value is 10:
                self.param_value[param] = self.controller.b10 
            elif value is 11:
                self.param_value[param] = self.controller.b11 
            elif value is 12:
                self.param_value[param] = self.controller.b12
            elif value is "hat_top":
                self.param_value[param] = self.controller.hat_top
            elif value is "hat_left":
                self.param_value[param] = self.controller.hat_left
            elif value is "hat_right":
                self.param_value[param] = self.controller.hat_right
            elif value is "hat_down":
                self.param_value[param] = self.controller.hat_down
