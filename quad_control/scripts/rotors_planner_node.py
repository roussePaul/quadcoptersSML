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
import planners.trajectory_quintic as qt

import tf.transformations as tft




class RotorSPlannerNode():

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
        """This is called only once to get the initial position of the quad, in
        case one wants to use it as the starting point for the reference
        trajectory.
        This callback kills the subcriber object herself at the end of the call.
        """
        
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        aux = msg.pose.orientation
        quaternion = numpy.array([aux.x, aux.y, aux.z, aux.w])
        euler_angles = tft.euler_from_quaternion(quaternion)
        yaw = euler_angles[2]
        
        self.quad_initial_pose = numpy.array([x, y, z, yaw])
        self.sub.unregister()
        self.got_quad_initial_pose_flag = True
    

    def work(self):

        # initialize node
        rospy.init_node('rotors_planner_node')
        
        # instantiate the publisher
        topic = 'hummingbird/command/trajectory'
        pub = rospy.Publisher(topic, tm.MultiDOFJointTrajectory, queue_size=10)

        # to get the initial position of the quad
        self.quad_initial_pose = None
        self.got_quad_initial_pose_flag = False
        topic = 'hummingbird/ground_truth/pose'
        self.sub = rospy.Subscriber(topic, gm.PoseStamped, self.pose_stamped_to_reference_position)

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

        # trajectory to be published
        #trajectory = ct.TrajectoryCircle([0.0, 0.0, 1.0, numpy.pi], numpy.eye(3), 2.0, 0.3)
        #self.trajectory = cbt.TrajectoryCubic([0.0, 0.0, 1.0, 0.0], numpy.eye(3), [2.0, -2.0, 1.0, 0.0], 10.0, 15.0)
        #self.trajectory = qt.TrajectoryQuintic([0.0, 0.0, 1.0, 0.0], numpy.eye(3), [2.0, -2.0, 1.0, 0.0], 5.0, 10.0)
        displacement = rospy.get_param('displacement', default=[0.0, 0.0, 1.0, 0.0])
        delay = rospy.get_param('delay', default=0.0)
        duration = rospy.get_param('duration', default=numpy.linalg.norm(displacement))
        self.trajectory = qt.TrajectoryQuintic(self.quad_initial_pose, numpy.eye(3), displacement, delay, delay+duration)

        # do work
        while not rospy.is_shutdown():

            self.time = rospy.get_time() - self.initial_time
            #print(self.time)
            #print(self.initial_time)
            p, v, a, j, s, c = self.trajectory.get_point(self.time)
            #print(p)
            msg = self.trajectory_to_multi_dof_joint_trajectory(p, v, a, j, s, c)
            pub.publish(msg)
            rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    node = RotorSPlannerNode()
    node.work()
    
