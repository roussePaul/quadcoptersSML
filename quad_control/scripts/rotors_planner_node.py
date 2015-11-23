#!/usr/bin/env python
"""In this file, a ROS node that publishes a trajectory is started.
The trajectory is published in the form of a ROS message of the type
'MultiDOFJointTrajectory', and a description can be found at this link.

http://docs.ros.org/indigo/api/trajectory_msgs/html/msg/MultiDOFJointTrajectory.html 

Such ROS message can be taken as a reference by the quad models in the RotorS
simulator.

The actual trajectory is generated with one of the functions in the module
'trajectories' of this package.
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

import planners.trajectory_circle as ct
import planners.trajectory_cubic as cbt

import tf.transformations as tft




class RotorSPlannerNode():

    def __init__(self):
        pass


    def trajectory_to_multidofjointtrajectory(self, p, v, a, j, s, c):
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
        point.velocities.append(velocity)

        acceleration = gm.Twist()
        acceleration.linear.x = a[0]
        acceleration.linear.y = a[1]
        acceleration.linear.z = a[2]
        point.accelerations.append(acceleration)

        return msg

    

    def work(self):

        # initialize node
        rospy.init_node('rotors_planner_node')

        # topic to publish on
        #topic = 'rotors_reference_trajectory'
        topic = 'hummingbird/command/trajectory'
        
        # instantiate the publisher
        pub = rospy.Publisher(topic, tm.MultiDOFJointTrajectory, queue_size=10)

        # setting the frequency of execution
        rate = rospy.Rate(1e1)

        # trajectory to be published
        #trajectory = ct.TrajectoryCircle([0.0, 0.0, 1.0, numpy.pi], numpy.eye(3), 2.0, 0.3)
        self.trajectory = cbt.TrajectoryCubic([0.0, 0.0, 1.0, 0.0], numpy.eye(3), [2.0, -2.0, 1.0, 0.0], 10.0, 15.0)

        # get initial time
        aux = rospy.get_time()
        while rospy.get_time() == aux:
            pass 
        self.initial_time = rospy.get_time()
        self.time = rospy.get_time() - self.initial_time
        #print(self.time)
        #print(type(self.initial_time))

        # do work
        while not rospy.is_shutdown():

            self.time = rospy.get_time() - self.initial_time
            #print(self.time)
            #print(self.initial_time)
            p, v, a, j, s, c = self.trajectory.get_point(self.time)
            #print(p)
            msg = self.trajectory_to_multidofjointtrajectory(p, v, a, j, s, c)
            pub.publish(msg)
            rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    node = RotorSPlannerNode()
    node.work()
    
