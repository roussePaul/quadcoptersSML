"""This file defines the abstract trajectory class.
This class gives the interface of a general trajectory class,
and different trajectories can be defined as subclasses.
A 'Trajectory' class needs to have an 'output' method that takes the current
time and returns the current value of the trajectory along with the first five
derivatives.
All trajectories are in 3D, and the class is not flexible as for that.
Also note that Trajectory objects have no memory. The returned position and the
derivatives are computed analitically.
"""

import numpy
from numpy import sin as s
from numpy import cos as c

class Trajectory:
    """Abstract class for a generic trajectory"""

    def __init__(self, offset, rotation):
        self.offset = numpy.array(offset)
        self.rotation = numpy.array(rotation)
        
    def output(self, time):
        position     = None
        velocity     = None
        acceleration = None
        jerk         = None
        snap         = None
        crackle      = None
        return position, velocity, acceleration, jerk, snap, crackle
        

class StillTrajectory(Trajectory):
    """A still trajectory (stays forever in the initial point)""" 

    def __init__(self, offset, rotation):
        super().__init__(offset, position)
        
    def output(self, time):
        p = numpy.array(self.offset)
        v = numpy.zeros(3)
        a = numpy.zeros(3)
        j = numpy.zeros(3)
        s = numpy.zeros(3)
        c = numpy.zeros(3)
        return p, v, a, j, s, c


class CircleTrajectory(Trajectory):
    """Circle trajectory"""

    def __init__(self, offset, rotation, radius, angular_velocity):
        super().__init__(offset, rotation)
        self.radius = radius
        self.angular_velocity = angular_velocity
        
    def output(self, time):
        
        r = self.radius
        w = self.angular_velocity
        rot = self.rotation
        off = self.offset
        
        p = r*w**0*numpy.array([ c(w*t), -s(w*t), 0.0])
        v = r*w**1*numpy.array([-s(w*t), -c(w*t), 0.0])
        a = r*w**2*numpy.array([-c(w*t),  s(w*t), 0.0])
        j = r*w**3*numpy.array([ s(w*t),  c(w*t), 0.0])
        s = r*w**4*numpy.array([ c(w*t), -s(w*t), 0.0])
        c = r*w**5*numpy.array([-s(w*t), -c(w*t), 0.0])
        
        p = rot.dot(p) + off
        v = rot.dot(v)
        a = rot.dot(a)
        j = rot.dot(j)
        s = rot.dot(s)
        c = rot.dot(c)
        
        return p, v, a, j, s, c
        
        
circle = CircleTrajectory(0.0, numpy.eye(3), 1.0, 0.02)
print(circle.output(3.5))
