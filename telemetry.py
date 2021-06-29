#
# telemetry.py
#

import pylab

class Telemetry:

    def __init__(self):
        self.reset()

    def reset(self):
        self.times = [ ]
        self.target_omega_array = [ ]
        self.target_theta_array = [ ]
        self.current_omega_array = [ ]
        self.current_theta_array = [ ]
        self.t = 0

    def gather(self,delta_t, current_omega, target_omega):
        self.times.append(self.t)
        self.target_omega_array.append(target_omega)
        self.current_omega_array.append(current_omega)
        self.t = self.t + delta_t

    def show(self):
        pylab.figure(1)
        pylab.plot(self.times, self.target_omega_array, 'b-+', label="Omega Target")
        pylab.plot(self.times, self.current_omega_array, 'r-+', label="Omega Current")
        pylab.legend()

        #pylab.figure(2)
        #pylab.plot(self.times, self.target_vr_array, 'b-+', label="V Right Target")
        #pylab.plot(self.times, self.current_vr_array, 'r-+', label="V Right Current")
        #pylab.legend()

        #pylab.figure(3)
        #pylab.plot(self.times, self.current_w_array, 'r-+', label="Omega Current")
        #pylab.legend()

        pylab.show()



