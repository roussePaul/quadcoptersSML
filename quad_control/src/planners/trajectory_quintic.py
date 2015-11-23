import numpy
from numpy import cos as c
from numpy import sin as s

import planners.trajectory as tj



class TrajectoryCubic(tj.Trajectory):
    """Cubic trajectory.
    Implements a trajectory that starts in q0 = numpy.zeros(4) and ends in a
    given point qf.
    The trajectory has a given duration tf.
    Initial and final veocity are zero.
    """


    def __init__(self, offset, rotation, final_point, duration):
        """Arguments:
        - offset (numpy array, 4)
        - rotation (numpy array, 3-by-3)
        - final point (numpy array, 4)
        - duration (float)
        """
        
        tj.Trajectory.__init__(self, offset, rotation)
        
        t0 = 0.0
        q0 = numpy.zeros(4)
        dq0 = numpy.zeros(4)
        qf = final_point
        tf = duration        
        dqf = numpy.zeros(4)


        # compute polynomial coefficients
        
        # known term: [q0, dq0, qf, dqf]
        known_term = numpy.concatenate(q0, dq0, qf, dqf)
        
        # matrix of the times
        matrix_q0 = numpy.kron(numpy.ones(4,1), numpy.array([[1.0, t0, t0**2, t0**3]]))
        print(matrix_q0)
        matrix_dq0 = numpy.kron(numpy.ones(4,1), numpy.array([[0.0, 1.0, 2*t0, 3*t0**2]]))
        matrix_qf = numpy.kron(numpy.ones(4,1), numpy.array([[1.0, tf, tf**2, tf**3]]))
        matrix_dqf = numpy.kron(numpy.ones(4,1), numpy.array([[0.0, 1.0, 2*tf, 3*tf**2]]))
        
        matrix = numpy.concatenate([matrix_q0, matrix_dq0, matrix_qf, matrix_dqf], axis=0)


    def _get_untransformed_point(self, time):

        t = time
        
        pass
        #return p, v, a, j, sn, cr
        
        
# test
cubic = TrajectoryCubic(numpy.zeros(4), numpy.eye(3), numpy.ones(4), 10.0)
