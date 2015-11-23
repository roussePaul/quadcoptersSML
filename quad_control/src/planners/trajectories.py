"""This file defines the abstract trajectory class.
This class gives the interface of a general trajectory class,
and different trajectories can be defined as subclasses.
A 'Trajectory' class needs to have an 'output' method that takes the current
time and returns the current value of the trajectory along with the first five
derivatives.
All trajectories are in 4D, that is x, y, z, and yaw.
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
        position = None
        velocity = None
        acceleration = None
        jerk = None
        snap = None
        crackle = None
        return position, velocity, acceleration, jerk, snap, crackle



class StillTrajectory(Trajectory):
    """A still trajectory (stays forever in the initial point)."""

    def __init__(self, offset, rotation):
        Trajectory.__init__(offset, position)

    def output(self, time):
        p = numpy.array(self.offset)
        v = numpy.zeros(4)
        a = numpy.zeros(4)
        j = numpy.zeros(4)
        s = numpy.zeros(4)
        c = numpy.zeros(4)
        return p, v, a, j, s, c



class CircleTrajectory(Trajectory):
    """Circle trajectory."""

    def __init__(self, offset, rotation, radius, angular_velocity):
        Trajectory.__init__(self, offset, rotation)
        self.radius = radius
        self.angular_velocity = angular_velocity

    def output(self, time):

        t = time
        r = self.radius
        w = self.angular_velocity
        rot = self.rotation
        off = self.offset

        p = numpy.zeros(4)
        v = numpy.zeros(4)
        a = numpy.zeros(4)
        j = numpy.zeros(4)
        sn = numpy.zeros(4)
        cr = numpy.zeros(4)

        p[0:3] = r * w**0 * numpy.array([c(w * t), -s(w * t), 0.0])
        v[0:3] = r * w**1 * numpy.array([-s(w * t), -c(w * t), 0.0])
        a[0:3] = r * w**2 * numpy.array([-c(w * t), s(w * t), 0.0])
        j[0:3] = r * w**3 * numpy.array([s(w * t), c(w * t), 0.0])
        sn[0:3] = r * w**4 * numpy.array([c(w * t), -s(w * t), 0.0])
        cr[0:3] = r * w**5 * numpy.array([-s(w * t), -c(w * t), 0.0])

        p[3] = -numpy.arctan2(s(w*t),c(w*t))
        v[3] = -w
        a[3] = 0.0
        j[3] = 0.0
        sn[3] = 0.0
        cr[3] = 0.0

        p[0:3] = rot.dot(p[0:3]) + off[0:3]
        v[0:3] = rot.dot(v[0:3])
        a[0:3] = rot.dot(a[0:3])
        j[0:3] = rot.dot(j[0:3])
        sn[0:3] = rot.dot(sn[0:3])
        cr[0:3] = rot.dot(cr[0:3])

        p[3] += off[3]

        return p, v, a, j, sn, cr




# test
#circle = CircleTrajectory(numpy.zeros(4), numpy.eye(4), 1.0, 0.02)
# print(circle.output(3.5))
