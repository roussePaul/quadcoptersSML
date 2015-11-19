#!/usr/bin/env python
"""This file implements the quad controller node.
There are different control laws that you can choose from.
The control law can be changed by setting the object 'controller' to one of
the given controllers.
Of course, more controllers can be defined and added to the module.
The controllers can be changed dynamically using a ROS service.
"""

"""The weird line on top
'#!/usr/bin/env python'
should be left there.
It is needed by ROS to understand that this file corresponds to a ROS node.
"""


import rospy

import quad_control.msg as qcm

import controllers

import numpy as np

class QuadControllerNode:
    """The QuadControllerNode class.
    It corresponds to a ROS node.
    The type of controller depends on the object 'controller' that this node
    contains.
    """

    def __init__(self):
        
        # frequency of the control action
        self.frequency = 90.0
        
        # local time
        # can be updated with 'rospy.get_time()'
        # after 'rospy.init_node()' is called.
        self.time = None
        
        # timestamp of the last quad measurement
        self.quad_time = None
        
        # last measured position of the quad
        self.p = np.zeros(3)
        
        # last measured velocity of the quad
        self.v = np.zeros(3)
        
        # last measured acceleration of the quad
        self.a = np.zeros(3)
        
        # last measured attitude of the quad (in degrees)
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        
        # last received reference with timestamp and derivatives
        self.ref_time = 0.0
        self.rp = np.zeros(3)
        self.rv = np.zeros(3)
        self.ra = np.zeros(3)
        self.rj = np.zeros(3)
        self.rs = np.zeros(3)
        self.rc = np.zeros(3)
        
        # current controller
        self.controller = None
        
        
    def get_quad_pose(self, qpm):
        """This function is the callback that is called when the controller
        receives a position measurement from the quad.
        The argument 'qpm' (quad pose message) is a ROS message of type
        'QuadPoseMsg', which contains position and attitude of the quad.
        """
        
        # time when the message was sent
        timestamp = qpm.time
        self.quad_time = timestamp
        
        # position of the quad
        self.p = np.array([qpm.x, qpm.y, qpm.z])
        
        # velocity of the quad
        # estimated with the velocity filter
        self.v = self.velocity_filter.update_and_output(self.p, timestamp)
        
        # pose of the quad
        self.roll_deg = qpm.roll
        self.pitch_deg = qpm.pitch
        self.yaw_deg = qpm.yaw
        
        
    def get_reference_trajectory(self, rtm):
        """This function is the callback that is called when the controller
        receives a time instance of the reference trajectory and its derivatives.
        The argument 'rtm' (reference trajectory message) is a ROS message of type
        'ReferenceTrajectoryMsg', which contains a timestamp and the
        corresponding values of the reference trajectory and its derivatives. 
        """
        
        # time when the message was sent
        self.ref_time = qpm.time
        
        # reference trajectory and derivatives
        self.rp = np.array(qpm.p)
        self.rv = np.array(qpm.v)
        self.ra = np.array(qpm.a)
        self.rj = np.array(qpm.j)
        self.rs = np.array(qpm.s)
        self.rc = np.array(qpm.c)
        
        
    def write_quad_cmd_msg(self):
        
        message = qcm.QuadCmdMsg()
        
        message.time = self.time
        ea_deg = (self.roll_deg, self.pitch_deg, self.yaw_deg)
        message.cmd_1, message.cmd_2, message.cmd_3, message.cmd_4 = self.controller.control_law(self.time, self.p, ea_deg, self.v, self.a, self.rp, self.rv, self.ra, self.rj, self.rs, self.rc)
        
        return message
        
    
    def work(self):
        """This function is the worker of the ROS node corresponding to this object.
        """
    
        # initialize the ROS node
        rospy.init_node('controller')
        
        # initialize the time
        initial_time = rospy.get_time()
        self.time = 0.0
        
        # initialize the controller
        self.controller = controllers.ControllerZero()
        
        # publisher of the control input signal
        cmd_pub = rospy.Publisher('quad_cmd', qcm.QuadCmdMsg, queue_size=10)
        
        # subscriber to the quad pose
        pose_sub = rospy.Subscriber('quad_pose', qcm.QuadPoseMsg, self.get_quad_pose)
        
        # subscriber to the reference trajectory
        ref_sub = rospy.Subscriber('reference_trajectory', qcm.ReferenceTrajectoryMsg, self.get_reference_trajectory)
        
        # setting the frequency of the execution
        rate = rospy.Rate(self.frequency)
        
        # do work
        while not rospy.is_shutdown():
        
            cmd_msg = self.write_quad_cmd_msg()
            cmd_pub.publish(cmd_msg)
            self.time = rospy.get_time() - initial_time
    
            rate.sleep()
    
        rospy.spin()
        
        
# run the node
if __name__ == '__main__':
    controller_node = QuadControllerNode()
    controller_node.work()
