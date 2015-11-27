#!/usr/bin/env python
"""In this file, a ROS node that publishes a trajectory is started.
The trajectory is planned by an object of type PlannerToGoal from this package.
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

import planners.planner_to_goal as ptg

import tf.transformations as tft




class RotorSToGoalPlannerNode():

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
        acceleration.angular.z = a[3]
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
        
     
    def _get_quad_pos_vel(self, msg):
        """Callback to save the position and velocity of the quad into
        object properties.
        """
        
        self._pos, self._vel = self._odometry_to_pos_vel(msg)
        self._got_initial_pos_vel_flag = True
    

    def work(self):

        # initialize node
        rospy.init_node('rotors_collision_avoidance_follower_planner_node')
        
         # setting the frequency of execution
        freq = 1e2
        rate = rospy.Rate(freq)
        
        # planner
        gain_pos = rospy.get_param('gain_pos', default=1.0)
        gain_vel = rospy.get_param('gain_vel', default=1.0)
        goal_point = rospy.get_param('goal_point', default=[-2.0, 0.0, 1.5, 2.0])
        self._planner = ptg.PlannerToGoal(goal_point, gain_pos, gain_vel)
        
        # instantiate the publisher
        topic = rospy.get_param('ref_traj_topic', default='hummingbird/command/trajectory')
        pub = rospy.Publisher(topic, tm.MultiDOFJointTrajectory, queue_size=10)

        # subscriber to position and velocity
        topic = rospy.get_param('pos_vel_topic', default='hummingbird/ground_truth/odometry')
        self._got_initial_pos_vel_flag = False
        rospy.Subscriber(topic, nm.Odometry, self._get_quad_pos_vel)

        # get initial quad position
        while not self._got_initial_pos_vel_flag:
            rate.sleep()

        # do work
        while not rospy.is_shutdown():
            
            vel = numpy.array(self._vel)
            pos = numpy.array(self._pos)
            
            acc = self._planner.get_acceleration(pos, vel)
            
            aux = self._planner.get_velocity(pos)
            vel[2] = aux[2]
            
            j = numpy.zeros(4)
            sn = numpy.zeros(4)
            cr = numpy.zeros(4)

            print(numpy.linalg.norm(pos-self._planner._goal_point))

            msg = self._trajectory_to_multi_dof_joint_trajectory(pos, vel, acc, j, sn, cr)
            pub.publish(msg)
            rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    node = RotorSToGoalPlannerNode()
    node.work()
    
