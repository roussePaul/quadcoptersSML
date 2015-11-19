class QuadSimulator:
    """Skeleton class for the quad simulator.
    You can think about it as an abstract class.
    The real simulators are obtained by inheritance.
    Each inherited simulator must implement the 'update_pose' function, which
    defines the dynamics of the simulator.
    """
    
    def __init__(self, t, p, ea_deg, u):
        """Constructor of the simulator.
        The initialization parameters are:
        - initial time t
        - initial position p
        - initial attitude in the form of euler angles in degrees ea_deg
        - initial control input u
        """
        pass
        
    def update_pose(self, ti, tf, p, ea_deg, u):
        """Template of the 'update_pose' function.
        The arguments are:
        - current time
        - next time tf
        - current position p
        - current attitude in the form of euler angles in degrees ea_deg
        - current control input u
        The function returns:
        - new position new_p
        - new attitude new_ea_deg
        Each simulator must implement this function, but it does not matter how.
        Most likely, there will be an ODE solver that will integrate the
        dynamics of the quad between the times ti and tf.
        """
        new_p = None
        new_ea_deg = None
        return new_p, new_ea_deg
