from simulator_skeleton import QuadSimulator


class QuadSimulatorZero(QuadSimulator):
    """Dumb simulator that leaves the quad where it is."""

    def __init__(self, t, p, ea_deg, u):
        QuadSimulator.__init__(self, t, p, ea_deg, u)
        
    def update_pose(self, ti, tf, p, ea_deg, u):
        new_p = numpy.array(p)
        new_ea_deg = numpy.array(ea_deg)
        return new_p, new_ea_deg
