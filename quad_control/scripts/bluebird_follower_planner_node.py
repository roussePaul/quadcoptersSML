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




class BluebirdFollowerPlannerNode():

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

    
    def _odometry_to_pos_vel(self, msg):
        """This function converts a message of type geomtry_msgs.Odometry into
        position and velocity of the quad as 4D numpy arrays.
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

        return pos, vel
        
     
    def _get_leader_state(self, msg):
        """Callback to get the position and velocity of the leader.
        """
    
        self._lead_pos, self._lead_vel = self._odometry_to_pos_vel(msg)
        self.got_leader_state_flag = True
        
    
    def _get_quad_initial_pos(self, msg):
        """
        Callback to get the position and velocity of the quad.
        This is called only once to get the initial position of the quad, in
        case one wants to use it as the starting point for the reference
        trajectory.
        This callback kills the subcriber object herself at the end of the call.
        """
        
        self._initial_pos, dummy = self._odometry_to_pos_vel(msg)
        self._follower_pose_subscriber.unregister()
        self._got_quad_initial_pos_flag = True
    

    def work(self):

        # initialize node
        rospy.init_node('rotors_follower_planner_node')
        
        # instantiate the publisher
        topic = rospy.get_param('follower_reference_trajectory_topic', default='command/trajectory')
        pub = rospy.Publisher(topic, tm.MultiDOFJointTrajectory, queue_size=10)

        # to get the initial position of the quad
        self._quad_initial_pos = None
        self._got_quad_initial_pos_flag = False
        topic = rospy.get_param('follower_pos_vel_topic', default='msf_core/odometry')
        self.follower_pose_subscriber = rospy.Subscriber(topic, nm.Odometry, self._get_quad_initial_pos)

        # subscriber to the position of the leader
        topic = rospy.get_param('leader_pos_vel_topic', default='msf_core/odometry')
        self._got_leader_state_flag = False
        self._leader_state_subscriber = rospy.Subscriber(topic, nm.Odometry, self._get_leader_state)

        # setting the frequency of execution
        rate = rospy.Rate(1e1)

        # get initial quad position
        while not self._got_leader_state_flag and not rospy.is_shutdown():
            print("The follower planner is waiting for the leader position.")
            rate.sleep()

        # desired offset with respect to the leader
        self.offset = numpy.array([1.0, 0.0, 0.0, 0.0])
        #self.trajectory = ct.TrajectoryCircle(self.quad_initial_pose, numpy.eye(3), delay, delay+duration, radius, ang_vel)

        # do work
        while not rospy.is_shutdown():

            p = self._lead_pos + self._offset
            v = self._lead_vel
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
    node = BluebirdFollowerPlannerNode()
    node.work()
    
