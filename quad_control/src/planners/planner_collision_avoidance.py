import numpy


class PlannerCollisionAvoidance:
    """A planner to avoid collisions.
    It offers a function to compute a 3D acceleration to drive the body away
    from the obstacle. The acceleration is returned as a 4D numpy array, but the
    yaw component is always set to zero.
    """


    def __init__(self, gain, ths):
        """The constructor takes a gain and a threshold.
        The threshold is the distance where the collision avoidance starts to
        take effect.
        """
        
        self._gain = gain
        self._ths = ths


    def _rotate_2d_array_by_pi_halves(self, array):
        aux = numpy.zeros(2)
        aux[0] = -array[1]
        aux[1] = array[0]
        return aux
        
    
    def _saturate(self, acc, ths):
        """Saturates the acceleration if needed."""
        
        exp = 10.0
        acc_norm = numpy.linalg.norm(acc)
        acc_sat = acc/(1+(acc_norm/ths)**exp)**(1.0/exp)
        
        return acc_sat
        

    def get_acceleration(self, quad_pos, obs_pos):

        k = self._gain
        s = self._ths
        
        po = obs_pos
        p = quad_pos
        d = numpy.linalg.norm(po-p)
        
        if d < s:
            acc = -0.5*k*(po-p)/d*(1.0/d-1.0/s) 
            aux = self._rotate_2d_array_by_pi_halves(acc[0:2])
            acc[0:2] += 0.5*k*aux
            acc[2] = 0.0
            #acc = self._saturate(acc, 5.0)
        else:
            acc = numpy.zeros(4)

        h = numpy.linalg.norm(p[2])
        hs = 0.5
        kh = 0.3
        if h < hs:
            acc[2] += kh*(1.0/h - 1.0/hs)

        return acc
