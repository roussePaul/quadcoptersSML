#!/usr/bin/env python
"""In this file, a ROS node that publishes a trajectory is started.
The trajectory is planned by copying the trajectory of a leader quad and adding
a constant offset.
The trajectory is published in the form of a ROS message of the type
'MultiDOFJointTrajectory', and a description can be found at this link.

http://docs.ros.org/indigo/api/trajectory_msgs/html/msg/MultiDOFJointTrajectory.html 

Such ROS message can be taken as a reference by the quad models in the RotorS
simulator.
"""

"""The weird line on top
'#!/usr/bin/env python'
should be left there.
It is needed by ROS to understand that this file corresponds to a ROS node.
"""


import rospy

import numpy

import trajectory_msgs.msg as tm
import geometry_msgs.msg as gm
import nav_msgs.msg as nm

import planners.trajectory_circle as ct
import planners.trajectory_cubic as cbt
import planners.trajectory_quintic as qt

import tf.transformations as tft




class RotorSFollowerPlannerNode():

    def __init__(self):
        pass


    def _trajectory_to_multi_dof_joint_trajectory(self, p, v, a, j, s, c):
        """Function to transform a 'Trajectory' object into a
        'MultiDOFJointTrajectoryPoint object'.
        """

        msg = tm.MultiDOFJointTrajectory()
        point = tm.MultiDOFJointTrajectoryPoint()
        msg.points.append(point)

        #print(p)

        transform = gm.Transform()
        transform.translation.x = p[0]
        transform.translation.y = p[1]
        transform.translation.z = p[2]
        quaternion = tft.quaternion_from_euler(0.0, 0.0, p[3])
        transform.rotation.x = quaternion[0]
        transform.rotation.y = quaternion[1]
        transform.rotation.z = quaternion[2]
        transform.rotation.w = quaternion[3]
        point.transforms.append(transform)

        velocity = gm.Twist()
        velocity.linear.x = v[0]
        velocity.linear.y = v[1]
        velocity.linear.z = v[2]
        velocity.angular.z = v[3]
        point.velocities.append(velocity)

        acceleration = gm.Twist()
        acceleration.linear.x = a[0]
        acceleration.linear.y = a[1]
        acceleration.linear.z = a[2]
        point.accelerations.append(acceleration)

        return msg

    
    def _pose_stamped_to_reference_position(self, msg):
        """This function converts a message of type geometry_msgs.PoseStamped
        into a numpy array containing the position (including yaw) of the quad.
        """
        
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        aux = msg.pose.orientation
        quaternion = numpy.array([aux.x, aux.y, aux.z, aux.w])
        euler_angles = tft.euler_from_quaternion(quaternion)
        yaw = euler_angles[2]
        
        return numpy.array([x, y, z, yaw])
        
       
    def _odometry_to_reference_point(self, msg):
        """This function converts a message of type geomtry_msgs.Odometry into a
        reference point. The reference point inclues pos, vel, acc, jerk, snap
        and crackle, all in 4D (that is, including x, y z, and yaw) in the form
        of numpy arrays.
        Derivstives further than vel are set to zero.
        """
    
        aux1 = msg.pose.pose.position
        aux2 = msg.pose.pose.orientation
        quaternion = numpy.array([aux2.x, aux2.y, aux2.z, aux2.w])
        ea = tft.euler_from_quaternion(quaternion)
        yaw = ea[2]
        pos = numpy.array([aux1.x, aux1.y, aux1.z, yaw])
        aux3 = msg.twist.twist.linear
        aux4 = msg.twist.twist.angular
        vel = numpy.array([aux3.x, aux3.y, aux3.z, aux4.z])
        acc = numpy.zeros(4)
        jerk = numpy.zeros(4)
        snap = numpy.zeros(4)
        crackle = numpy.zeros(4)
        return pos, vel, acc, jerk, snap, crackle
        
     
    def _get_leader_state(self, msg):
        self.lead_pos, self.lead_vel, self.lead_acc, self.lead_jrk, self.lead_snp, self.lead_crk = self._odometry_to_reference_point(msg)
        self.got_leader_state_flag = True
        
    
    def _get_quad_initial_pose(self, msg):
        """This is called only once to get the initial position of the quad, in
        case one wants to use it as the starting point for the reference
        trajectory.
        This callback kills the subcriber object herself at the end of the call.
        """
        
        self.quad_initial_pose = self._pose_stamped_to_reference_position(msg)
        self.follower_pose_subscriber.unregister()
        self.got_quad_initial_pose_flag = True
    

    def work(self):

        # initialize node
        rospy.init_node('rotors_follower_planner_node')
        
        # instantiate the publisher
        topic = rospy.get_param('follower_reference_trajectory_topic', default='command/trajectory')
        pub = rospy.Publisher(topic, tm.MultiDOFJointTrajectory, queue_size=10)

        # to get the initial position of the quad
        self.quad_initial_pose = None
        self.got_quad_initial_pose_flag = False
        topic = rospy.get_param('follower_pose_topic', default='ground_truth/pose')
        self.follower_pose_subscriber = rospy.Subscriber(topic, gm.PoseStamped, self._get_quad_initial_pose)

        # subscriber to the position of the leader
        topic = rospy.get_param('leader_state_topic', default='/hummingbird_leader/ground_truth/odometry')
        self.got_leader_state_flag = False
        self.leader_state_subscriber = rospy.Subscriber(topic, nm.Odometry, self._get_leader_state)

        # setting the frequency of execution
        rate = rospy.Rate(1e1)

        # get initial quad position
        while not self.got_leader_state_flag:
            rate.sleep()

        # desired offset with respect to the leader
        self.offset = numpy.array([1.0, 0.0, 0.0, 0.0])
        #self.trajectory = ct.TrajectoryCircle(self.quad_initial_pose, numpy.eye(3), delay, delay+duration, radius, ang_vel)

        # do work
        while not rospy.is_shutdown():

            p = self.lead_pos + self.offset
            v = self.lead_vel
            a = numpy.zeros(4)
            j = numpy.zeros(4)
            sn = numpy.zeros(4)
            cr = numpy.zeros(4)
            #print(p)
            msg = self._trajectory_to_multi_dof_joint_trajectory(p, v, a, j, sn, cr)
            pub.publish(msg)
            rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    node = RotorSFollowerPlannerNode()
    node.work()
    
