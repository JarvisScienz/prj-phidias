#
#
#

import math
import pylab

GRAVITY = 9.81

class ArmElement:

    def __init__(self, _L, _M, _b, _arm_name):
        self.w = 0
        self.theta = 0
        self.L = _L
        self.M = _M
        self.b = _b
        self.arm_name = _arm_name
        self.w_vect = []
        self.theta_vect = []

    def evaluate(self, _input_torque, delta_t):
        self.w = self.w - GRAVITY * delta_t * math.cos(self.theta) - \
            (self.b * delta_t * self.w * self. L) / self.M + \
            delta_t * _input_torque / (self.M * self.L)
        self.w_vect.append(self.w)
        self.theta = self.theta + delta_t * self.w
        self.theta_vect.append(math.degrees(self.theta))
        
    def get_pose(self):
        return (self.L * math.cos(self.theta), self.L * math.sin(self.theta) )

    def reset_vectors(self):
        self.w_vect = []
        self.theta_vect = []
        
    def plot_w(self, i, t_vect):
        if len(t_vect) != len(self.w_vect):
            print(self.arm_name + ": speed and time vectors have different lengths, no plot available")
            return
        
        pylab.figure(i)
        pylab.plot(t_vect, self.w_vect, 'r-+', label='vel, w(t), ' + self.arm_name)
        pylab.xlabel('time')
        pylab.legend()
        pylab.show()
        
    def plot_theta(self, i, t_vect):
        if len(t_vect) != len(self.theta_vect):
            print(self.arm_name + ": theta and time vectors have different lengths, no plot available")
            return
        
        pylab.figure(i)
        pylab.plot(t_vect, self.theta_vect, 'b-+', label='position, theta(t),' + self.arm_name)
        pylab.xlabel('time')
        pylab.legend()
        pylab.show()
        
    def plot(self, i, t_vect):
        self.plot_w(i, t_vect)
        self.plot_theta(i+1, t_vect)