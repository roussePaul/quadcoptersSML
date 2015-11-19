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
import rospy

# for solving differential equations
from scipy.integrate import ode

# import the ROS msgs used by the simulator
import quad_control.msg as qcm

import simulators.simulator_stabilize_mode as ssm


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
        # self.cmd = None
        self.cmd = [0.0, 0.0, 0.0, 0.0]
        
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
        self.time = 0.0
        
        # initialize the simulator
        self.quad_simulator = ssm.QuadSimulatorStabilizeMode(0.0, self.p, self.ea_deg, self.cmd)
        
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
            current_time = rospy.get_time()-initial_time
            
            # publish the quad pose
            quad_pose_msg = self.write_quad_pose_msg()
            quad_pose_pub.publish(quad_pose_msg)
            
            # update the quad pose
            #self.p, self.v, self.rot = unvectorize_quad_state(self.solver.y)
            new_p, new_ea_deg = self.quad_simulator.update_pose(self.time, current_time, self.p, self.ea_deg, self.cmd)
            self.p = numpy.array(new_p)
            self.ea_deg = numpy.array(new_ea_deg)
            
            # update the time
            self.time = current_time
            
            # let the node sleep
            rate.sleep()
            
        rospy.spin()
        

# executing the node
if __name__ == '__main__':
    sim_node = QuadSimulatorNode()
    sim_node.work()

