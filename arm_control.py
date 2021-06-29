#
#
#

import math
from controllers import *
from profile_position_control import *

class ArmControl:

    def __init__(self, arm, use_profile):
        self.arm = arm
        self.use_profile = use_profile
        if self.arm.L < 0.03:
            self.speed_controller = PI_SAT_Controller(0.3, 10, 5)
        else:
            self.speed_controller = PI_SAT_Controller(10, 5, 5)
        if self.use_profile:
            self.position_controller = ProfilePositionController(6.0, 2.0, 2.0)
        else:
            self.position_controller = PSAT_Controller(15, 8)
        self.target = 0
        self.w_target = 0

    def set_target(self, target):
        self.target = math.radians(target)

    def evaluate(self, delta_t):
        if self.use_profile:
            self.w_target = self.position_controller.evaluate(self.target, self.arm.theta, self.arm.w, delta_t)
        else:
            self.w_target = self.position_controller.evaluate(self.target - self.arm.theta, delta_t)
        torque = self.speed_controller.evaluate(self.w_target - self.arm.w, delta_t)
        #print(">>", self, self.w_target, self.target)
        self.arm.evaluate(torque, delta_t)

