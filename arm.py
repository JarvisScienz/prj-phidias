#
# arm.py
#

import math
from arm_model import *
from arm_control import *
from trajectory import *
from geometry import *
from pose import *
from block import *

class ThreeJointsArm:

    def __init__(self, trajectory_controller, use_profile):
        self.element_1_model = ArmElement(0.1, 0.5 + 0.5 + 0.2, 7e-5, "arm 1")
        self.element_1_control = ArmControl(self.element_1_model, use_profile)
        self.element_2_model = ArmElement(0.1, 0.5 + 0.2, 7e-5, "arm 2")
        self.element_2_control = ArmControl(self.element_2_model, use_profile)
        self.element_3_model = ArmElement(0.02, 0.2, 7e-5, "end effector")
        self.element_3_control = ArmControl(self.element_3_model, use_profile)
        self.trajectory = trajectory_controller
        self.trajectory.arm = self
        self.pose = Pose()
        self.alpha_vect = []
        self.captured_block = None
        self.DISTANCE_FROM_BLOCK = self.element_3_model.L - Block.HEIGHT/2 - 0.001
    def set_captured_block(self, block):
        self.captured_block = block
        
    def release_captured_block(self):
        b = self.captured_block
        self.captured_block = None
        return b

    def set_target(self, theta1, theta2, theta3):
        self.element_1_control.set_target(theta1)
        self.element_2_control.set_target(theta2)
        self.element_3_control.set_target(theta3)
        
    def set_block_pose(self, alpha, x, y):
        if self.captured_block != None:
            self.captured_block.set_pose(x + self.DISTANCE_FROM_BLOCK*math.cos(alpha) - Block.WIDTH/2, \
                                         y + self.DISTANCE_FROM_BLOCK*math.sin(alpha), \
                                         alpha + math.radians(90))

    def evaluate(self, delta_t):
        self.element_1_control.evaluate(delta_t)
        self.element_2_control.evaluate(delta_t)
        self.element_3_control.evaluate(delta_t)

    def get_pose_degrees(self):
        return ( math.degrees(self.element_1_model.theta),
                 math.degrees(self.element_2_model.theta),
                 math.degrees(self.element_3_model.theta) )

    def get_pose(self):
        (x1, y1) = self.element_1_model.get_pose()

        (_x2, _y2) = self.element_2_model.get_pose()
        (x2, y2) = local_to_global(x1, y1, self.element_1_model.theta, _x2, _y2)

        alpha = self.element_1_model.theta + self.element_2_model.theta + self.element_3_model.theta
        (x3, y3) = local_to_global(x2, y2, alpha, self.element_3_model.L, 0)
        self.set_block_pose(alpha, x3, y3)
        
        return [ (x1, y1), (x2, y2), (x3, y3) ]

    def get_pose_xy_a(self, add_alpha):
        (x1, y1) = self.element_1_model.get_pose()

        (_x2, _y2) = self.element_2_model.get_pose()
        (x2, y2) = local_to_global(x1, y1, self.element_1_model.theta, _x2, _y2)

        alpha = self.element_1_model.theta + self.element_2_model.theta + self.element_3_model.theta
        if add_alpha == True:
            self.alpha_vect.append(alpha)
        self.pose.set_pose(x2, y2, alpha)

        return self.pose

    def inverse_kinematics(self, xt, yt, alpha):
        atan_den = (xt**2 + yt ** 2 - self.element_1_model.L ** 2 - self.element_2_model.L ** 2) / (2 * self.element_1_model.L * self.element_2_model.L )
        arg = 1 - atan_den**2
        if arg < 0:
            return (None, None, None)
        theta2 = math.atan2( - math.sqrt( arg ), atan_den )
        theta1 = math.atan2(yt, xt) - math.atan2(self.element_2_model.L * math.sin(theta2),
                                                 self.element_1_model.L + self.element_2_model.L * math.cos(theta2) )
        theta3 = math.radians(alpha) - theta1 - theta2
        return (math.degrees(theta1), math.degrees(theta2), math.degrees(theta3))

    def set_target_xy_a(self, x, y, a):
        p = self.get_pose()
        alpha = self.element_1_model.theta + self.element_2_model.theta + self.element_3_model.theta
        self.trajectory.set_target(p[1][0], p[1][1], alpha, x, y, math.radians(a))

    def evaluate_trajectory(self, delta_t):
        (self.trajectory_x, self.trajectory_y, self.trajectory_a) = self.trajectory.evaluate(delta_t)
        (th1, th2, th3) = self.inverse_kinematics(self.trajectory_x, self.trajectory_y, math.degrees(self.trajectory_a))
        if th1 is not None:
            self.set_target(th1, th2, th3)
        self.evaluate(delta_t)

    def plot_alpha(self, i, t_vect):
        if len(t_vect) != len(self.alpha_vect):
            print("end effector: alpha and time vectors have different lengths, plot unavailable")
            print("alpha: " + str(len(self.alpha_vect)) + " elementi")
            print("time: " + str(len(t_vect)) + " elements")
            return
        
        pylab.figure(i)
        pylab.plot(t_vect, self.alpha_vect, 'g-+', label='position, alpha, end effector')
        pylab.xlabel('time')
        pylab.legend()
        pylab.show()

    def plot(self, t_vect):
        self.element_1_model.plot(0, t_vect)
        self.element_2_model.plot(2, t_vect)
        self.element_3_model.plot(4, t_vect)
        self.plot_alpha(6, t_vect)
        
    def reset_vectors(self):
        alpha_vect = []
        self.element_1_model.reset_vectors()
        self.element_2_model.reset_vectors()
        self.element_3_model.reset_vectors()