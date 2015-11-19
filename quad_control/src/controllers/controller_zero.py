from controller_skeleton import QuadController

class QuadControllerZero(QuadController):
    """Dummy Controller.
    It always returns zero.
    """

    def __init__(self):
        QuadController.__init__(self)
        
    def control_law(self, t, p, ea_deg, v, a, rp, rv, ra, rj, rs, rc):
        cmd = [0.0, 0.0, 0.0, 0.0]
        return cmd
