#!/usr/bin/env python
"""This file implements the quad simulator.
There are different dynamics that you can choose from.
The dynamics can be changed by setting the object 'quad_simulator' to one of the
simulators at disposal.
Of course, more dynamics functions can be defined and added to the simulator.
The type of simulator can be changed dynamically using a ROS service.
"""

"""The weird line on top
'#!/usr/bin/env python'
should be left there.
It is needed by ROS to understand that this file corresponds to a ROS node.
"""

import numpy
from numpy import pi

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


class QuadSimulator:
    """Skeleton class for the quad simulator."""
    
    def __init__(self, t, p, ea_deg, u):
        pass
        
    def update_pose(self, tf, p, ea_deg, u):
        new_p = None
        new_ea_deg = None
        return new_p, new_ea_deg
        

class QuadSimulatorZero(QuadSimulator):
    """Dumb simulator that leaves the quad where it is."""

    def __init__(self, t, p, ea_deg, u):
        QuadSimulator.__init__(self, t, p, ea_deg, u)
        
    def update_pose(self, tf, p, ea_deg, u):
        new_p = numpy.array(p)
        new_ea_deg = numpy.array(ea_deg)
        return new_p, new_ea_deg
    

class QuadSimulatorStabilizeMode(QuadSimulator):
    """Simulator corresponding to the stabilize mode of the IRIS."""

    def __init__(self, t, p, ea_deg, u):
        
        QuadSimulator.__init__(self, t, p, ea_deg, u)
        
        self.t = t
        self.p = numpy.array(p)
        self.v = numpy.zeros(3)
        self.rot = uf.ea_deg_to_rot(*ea_deg)
        
        self.cmd = list(u)
        
        self.solver = ode(self.ode_f).set_integrator('dopri5')
        y0 = vectorize_quad_state(self.p, self.v, self.rot)
        self.solver.set_initial_value(y0, self.t)
    
    
    def dynamics(self, p, v, rot, cmd):
        
        # unpacking the control input
        roll_cmd, pitch_cmd, thrust_cmd, yaw_rate_cmd = cmd
    
        ea = uf.rot_to_ea(rot)
    
        # current yaw
        yaw = ea[2]
    
        # maximum yaw rate
        max_yaw_rate_deg = sp.max_yaw_rate_deg
        max_yaw_rate = max_yaw_rate_deg*pi/180
    
        # maximum tilt
        max_tilt_deg = sp.max_tilt_deg
        max_tilt = max_tilt_deg*pi/180
    
        # desired roll
        roll_des = (roll_cmd-1500)*max_tilt/500
    
        # desired pitch
        pitch_des = -(pitch_cmd-1500)*max_tilt/500
    
        # gain of the inner loop for attitude control
        ktt = sp.inner_loop_gain
    
        # desired thrust versor
        e3 = numpy.array([0.0, 0.0, 1.0])
        dtv = uf.ea_to_rot(roll_des, pitch_des, yaw).dot(e3)
    
        # actual thrust versor
        atv = rot.dot(e3)
    
        # angular velocity
        aux = ktt*uf.skew(atv).dot(dtv)
        rot_t = numpy.transpose(rot)
        omega = rot_t.dot(aux)
    
        # yaw rate
        omega[2] = -(yaw_rate_cmd-1500)*max_yaw_rate/500
    
        # neutral thrust
        nt = sp.neutral_thrust
    
        # thrust gain
        kt = sp.quad_mass*sp.g/nt
    
        # thrust command adjustment
        # The thrust sent to the motors is automatically adjusted according to
        # the tilt angle of the vehicle (bigger tilt leads to bigger thrust)
        # to reduce the thrust compensation that the pilot must give.
        thrust_cmd = thrust_cmd/numpy.dot(atv,e3)
    
        # dynamics
        dp = v
        dv = kt*thrust_cmd*atv/sp.quad_mass - sp.g*e3
        drot = rot.dot(uf.skew(omega))
        
        return dp, dv, drot
    
    
    def ode_f(self, t, y, u):
        
        p, v, rot = unvectorize_quad_state(y)
        dp, dv, drot = self.dynamics(p, v, rot, u)
        dy = vectorize_quad_state(dp, dv, drot)
        return dy
    
    
    def update_pose(self, tf, p, ea_deg, cmd):
        
        self.solver.set_f_params(self.cmd)
        new_y = self.solver.integrate(tf)
        self.solver.set_initial_value(self.solver.y, tf)
    
        self.p, self.v, self.rot = unvectorize_quad_state(new_y)
        new_ea_deg = list(uf.rot_to_ea_deg(self.rot))
        
        return self.p, new_ea_deg



class QuadSimulatorNode:
    """This class implements the quad simulator node.
    The dynamics of the simulator depend on the object 'quad_simulator'
    that this class contains as a member.
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
        self.cmd = [1500.0, 1500.0, sp.neutral_thrust, 1500.0]
        #self.cmd = [0.0, 0.0, 0.0, 0.0]
        
        # current time
        self.time = None
        
        # current pose
        self.p = numpy.zeros(3)
        #self.v = numpy.zeros(3)
        self.ea_deg = numpy.zeros(3)
        
        # solver for the simulator
        #self.solver = ode(self.ode_f).set_integrator('dopri5')
        
        # current dynamics (python callable)
        #self.dynamics = stabilize_mode
        
        # current simulator
        self.quad_simulator = None
    
        
    def write_quad_pose_msg(self):
    
        qpm = qcm.QuadPoseMsg()
        
        qpm.time = self.time
        qpm.x, qpm.y, qpm.z = self.p
        qpm.roll, qpm.pitch, qpm.yaw = self.ea_deg
        
        return qpm
        
        
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
        
        # initialize the time
        initial_time = rospy.get_time()
        self.time = initial_time
        
        # initialize the simulator
        self.quad_simulator = QuadSimulatorStabilizeMode(initial_time, self.p, self.ea_deg, self.cmd)
        
        # the node subscribes to the topic 'quad_cmd'
        # 'qcm.QuadCmdMsg' is the class corresponding to a 'QuadCmdMsg' message
        # 'self.get_cmd_msg' is the callback called when a 'QuadCmdMsg' message is received
        quad_cmd_sub = rospy.Subscriber('quad_cmd', qcm.QuadCmdMsg, self.get_cmd_msg)
        
        # the node publishes the quad pose to the topic 'quad_pose'
        # 'qcm.quad_pose' is the class corresponding to a 'QuadPoseMsg' message
        quad_pose_pub = rospy.Publisher('quad_pose', qcm.QuadPoseMsg, queue_size=10)
        
        rate = rospy.Rate(self.frequency)
        
        while not rospy.is_shutdown():
        
            # give the new control input to the ODE solver
            #self.solver.set_f_params(self.cmd)
            
            # get the current time
            current_time = rospy.get_time()
            
            # publish the quad pose
            quad_pose_msg = self.write_quad_pose_msg()
            quad_pose_pub.publish(quad_pose_msg)
            
            # update the quad pose
            #self.p, self.v, self.rot = unvectorize_quad_state(self.solver.y)
            new_p, new_ea_deg = self.quad_simulator.update_pose(current_time, self.p, self.ea_deg, self.cmd)
            self.p = numpy.array(new_p)
            self.ea_deg = numpy.array(new_ea_deg)
            
            # update the time
            self.time = current_time - initial_time
            
            # let the node sleep
            rate.sleep()
            
        rospy.spin()
        

# executing the node
if __name__ == '__main__':
    sim_node = QuadSimulatorNode()
    sim_node.work()

