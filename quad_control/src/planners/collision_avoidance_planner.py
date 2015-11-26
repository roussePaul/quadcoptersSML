"""This file implements a planner for collision avoidance.
"""


import numpy 
import scipy.integrate as spi

class CollisionAvoidancePlanner:


    def __init__(self, gain, threshold):
    
        self.gain = gain
        self.threshold = threshold
        
        
    def _ode_f(self, y, t, u):
        p = y[0:4]
        v = y[4:8]
        u = numpy.array(u)
        dp = numpy.array(v)
        dv = numpy.array(u)
        dy = numpy.concatenate([dp, dv])
        return dy
        
        
    def get_collision_avoidance_drive(self, pos, other_pos, time_step):
        """This function returns the contributions to position, velocity and
        acceleration given by the collision avoidance drive.
        """
        
        k = self.gain
        ths = self.threshold
        
        displacement = other_pos[0:3] - pos[0:3]
        distance = numpy.linalg.norm(displacement)
        print(distance)
        if distance > ths:
            acc3 = numpy.zeros(3)
        elif distance < 0.01*ths:
            acc3 = numpy.zeros(3)
            print("GUYS, SOMETHING IS VEEEEERY WRONG HERE!!!!")
        else:
            acc3 = -(1.0/distance**2-1.0/ths**2)*displacement/distance
        
        acc = numpy.concatenate([acc3, numpy.zeros(1)])
        y = spi.odeint(self._ode_f, numpy.zeros(8), (0.0, time_step), args=tuple([acc]))
        new_y = y[1]
        delta_pos = new_y[0:4]
        delta_vel = new_y[4:8]
        
        return delta_pos, delta_vel, acc
