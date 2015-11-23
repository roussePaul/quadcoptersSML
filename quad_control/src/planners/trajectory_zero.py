import trajectory as tj

class TrajectoryZero(tj.Trajectory):
    """A still trajectory (stays forever in the initial point)."""

    def __init__(self, offset, rotation):
        tj.Trajectory.__init__(offset, position)

    def _get_untransformed_point(self, time):
        p = numpy.array(self.offset)
        v = numpy.zeros(4)
        a = numpy.zeros(4)
        j = numpy.zeros(4)
        s = numpy.zeros(4)
        c = numpy.zeros(4)
        
        return p, v, a, j, s, c
