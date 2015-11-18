#!/usr/bin/env python
"""This file implements the quad simulator.
There are different dynamics that you can choose from.
The dynamics can be changed by setting the function 'derivatives' to one of the
given dynamics function.
Of course, more dynamics functions can be defined and added to the simulator.
The dynamics can be changed dynamically using a ROS service.
"""


import numpy
import rospy

# for solving differential equations
from scipy.integrate import ode

# import the ROS msgs used by the simulator
import quad_control.msg as qcm

# import the simulation parameters
import utils.simulator_parameters as sp

# utility functions
import utils.utility_functions as uf



# self-explanatory
def vectorize_quad_state(p, v, rot):
    rot_vec = numpy.reshape(rot,9)
    return numpy.concatenate([p, v, rot_vec])

    
# self-explanatory
def unvectorize_quad_state(x):
    p = x[0:3]
    v = x[3:6]
    rot = numpy.reshape(x[6:], (3,3))
    return p, v, rot


def zero_dynamics(p, v, rot, control):
    """This function is one of the possible quad dynamics.
    This functions correponds to the trivial dynamics, i.e. to a quad that
    never changes position or attitude, regardless of the input.
    """
    dp = numpy.zeros(3)
    dv = numpy.zeros(3)
    drot = numpy.eye(3)
    return dp, dv, drot



class QuadSimulator:
    """This class implements the quad simulator.
    The dynamics of the simulator depend on the 'derivatives' function.
    """

    def __init__(self):
        
        # frequency of the ROS node corresponding to the simulator (Hz)
        self.frequency = 200
        
        # time delay with which the simulator is started
        self.delay = 2.0
        
        # current control input
        # it is initialized at zero
        # it is updated by subscribing to the controller node
        # it corresponds to the positions of the four sticks of the remote
        self.cmd = [0.0, 0.0, 0.0, 0.0]
        
        # current state
        self.p = numpy.zeros(3)
        self.v = numpy.zeros(3)
        self.rot = numpy.eye(3)
        
        # solver for the simulator
        self.solver = ode(self.ode_f).set_integrator('dopri5')
        x0 = vectorize_quad_state(self.p, self.v, self.rot)
        self.solver.set_initial_value(x0)
        
        # current dynamics (python callable)
        self.dynamics = zero_dynamics
        
        

        
    def ode_f(self, t, y, u):
        """This is the function that needs to be passed to the ODE solver.
        It needs to have the following format.
        It takes the parameters t, y, u in that order, where dy = f(t,y,u).
        It returns dy.
        Here y is a vectorized version of the quad state.
        """

        p, v, rot = unvectorize_quad_state(y)
        dp, dv, drot = zero_dynamics(p, v, rot, u)
        dy = vectorize_quad_state(dp, dv, drot)
        
        
    def write_quad_state_msg(self):
        
        # quad state message
        qsm = qcm.QuadStateMsg()
        
        qsm.x, qsm.y, qsm.z = self.p
        qsm.vx, qsm.vy, qsm.vz = self.v
        qsm.roll, qsm.pitch, qsm.yaw = uf.rot_to_ea_deg(self.rot)
        
        return qsm
        
        
    def get_cmd_msg(self, msg):
        """This is the callback function that is called when a new value of the
        control input is received from the controller.
        This function copies the new value that has been received in the local
        variable 'self.control'.
        """
        self.cmd = [msg.cmd_1, msg.cmd_2, msg.cmd_3, msg.cmd_4]
    
    
    def work(self):
        """This function is the worker of the ROS node corresponding to this object.
        """
        
        # initialize a ROS node corresponding to the simulator
        rospy.init_node('quad_simulator')
        
        # the node subscribes to the topic 'quad_cmd'
        # 'qcm.quad_cmd' is the class corresponding to a 'quad_cdm' message
        # 'self.get_cmd_msg' is the callback called when a cmd message is received
        quad_cmd_sub = rospy.Subscriber('quad_cmd', qcm.QuadCmdMsg, self.get_cmd_msg)
        
        # the node publishes to the topic 'quad_state'
        # 'qcm.quad_state' is the class corresponding to a 'quad_state' message
        quad_state_pub = rospy.Publisher('quad_state', qcm.QuadStateMsg, queue_size=10)
        
        rate = rospy.Rate(self.frequency)
        
        while not rospy.is_shutdown():
        
            # give the new control input to the ODE solver
            self.solver.set_f_params(self.cmd)
            
            # run the ODE solver
            self.solver.integrate(self.solver.t+1.0/self.frequency)
            
            # prepare the ODE solver for the next round
            self.solver.set_initial_value(self.solver.y, self.solver.t)
            
            # publish the quad state
            quad_state_msg = self.write_quad_state_msg()
            quad_state_pub.publish(quad_state_msg)
            
            # update the quad state
            self.p, self.v, self.rot = unvectorize_quad_state(self.solver.y)
            
            # let the node sleep
            rate.sleep()
            
        rospy.spin()
        
        
if __name__ == '__main__':
    sim = QuadSimulator()
    sim.work()

