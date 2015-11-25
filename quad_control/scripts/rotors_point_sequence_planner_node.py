#!/usr/bin/env python
"""In this file, a ROS node that publishes a trajectory is started.
The trajectory is published in the form of a ROS message of the type
'MultiDOFJointTrajectory', and a description can be found at this link.

http://docs.ros.org/indigo/api/trajectory_msgs/html/msg/MultiDOFJointTrajectory.html 

Such ROS message can be taken as a reference by the quad models in the RotorS
simulator.

The actual trajectory is generated as a sequence of waypoint.
Each navigation segment is generated with one of the functions in the module
'trajectories' of this package.
"""

"""The weird line on top
'#!/usr/bin/env python'
should be left there.
It is needed by ROS to understand that this file corresponds to a ROS node.
"""


import rospy

import numpy
import random

import trajectory_msgs.msg as tm
import geometry_msgs.msg as gm

import planners.trajectory_circle as ct
import planners.trajectory_cubic as cbt
import planners.trajectory_quintic as qt

import tf.transformations as tft




class RotorSPointSequencePlannerNode():

    def __init__(self):
        pass


    def trajectory_to_multi_dof_joint_trajectory(self, p, v, a, j, s, c):
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

    
    def pose_stamped_to_reference_position(self, msg):
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
    
    
    def get_quad_pose(self, msg):
        """This is the callback used to get the position of the quad."""
        
        self.quad_pose = self.pose_stamped_to_reference_position(msg)
        self.got_quad_initial_pose_flag = True
    
    
    def generate_initial_waypoint(self):
        if self.quad_pose[2] < 0.5:
            self.waypoint = numpy.array(self.quad_pose)
            self.waypoint[2] = 0.5
            duration = 2.5*numpy.linalg.norm(self.waypoint-self.quad_pose)
            time = rospy.get_time() - self.initial_time
            self.trajectory = qt.TrajectoryQuintic(self.quad_pose, numpy.eye(3), time, time+duration, self.waypoint)
        else:
            self.generate_waypoint()


    def generate_waypoint(self):
        self.waypoint = numpy.array(self.quad_pose)
        distance = numpy.linalg.norm(self.waypoint-self.quad_pose)
        while distance < 0.1:
            x = random.uniform(-1.5, 1.5)
            y = random.uniform(-1.5, 1.5)
            z = random.uniform(1.0, 2.0)
            yaw = 0.0
            self.waypoint = numpy.array([x, y, z, yaw])
            distance = numpy.linalg.norm(self.waypoint-self.quad_pose)
        duration = 2.5*numpy.linalg.norm(self.waypoint-self.quad_pose)
        time = rospy.get_time() - self.initial_time
        self.trajectory = qt.TrajectoryQuintic(self.quad_pose, numpy.eye(3), time, time+duration, self.waypoint)


    def work(self):

        # initialize node
        rospy.init_node('rotors_planner_node')

        # local time
        initial_time = rospy.get_time()
        self.time = rospy.get_time() - initial_time
        
        # instantiate the publisher
        topic = rospy.get_param('reference_trajectory_topic', default='command/trajectory')
        pub = rospy.Publisher(topic, tm.MultiDOFJointTrajectory, queue_size=10)

        # to get the pose of the quad
        self.quad_pose = None
        self.got_quad_initial_pose_flag = False
        topic = rospy.get_param('quad_pose_topic', default='ground_truth/pose')
        self.sub = rospy.Subscriber(topic, gm.PoseStamped, self.get_quad_pose)

        # current waypoint
        self.waypoint = numpy.array([0.0, 0.0, 1.0, 0.0])

        # setting the frequency of execution
        rate = rospy.Rate(1e1)

        # get initial time
        aux = rospy.get_time()
        while rospy.get_time() == aux:
            pass
        self.initial_time = rospy.get_time()
        self.time = rospy.get_time() - self.initial_time
        #print(self.time)
        #print(type(self.initial_time))

        # get initial quad position
        while not self.got_quad_initial_pose_flag:
            rate.sleep()

        # generate initial waypoint
        self.generate_initial_waypoint()

        # do work
        while not rospy.is_shutdown():

            # get current time
            self.time = rospy.get_time() - self.initial_time
            print(self.time)
            
            # see if the current waypoint is reached
            print self.quad_pose
            print self.waypoint
            print numpy.linalg.norm(self.quad_pose-self.waypoint)
            
            if numpy.linalg.norm(self.quad_pose-self.waypoint) < 0.1:
                self.generate_waypoint()
                
            p, v, a, j, s, c = self.trajectory.get_point(self.time)
            msg = self.trajectory_to_multi_dof_joint_trajectory(p, v, a, j, s, c)
            
            pub.publish(msg)
            rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    node = RotorSPointSequencePlannerNode()
    node.work()
    
