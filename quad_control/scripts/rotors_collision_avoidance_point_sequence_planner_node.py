#!/usr/bin/env python
"""In this file, a ROS node that publishes a trajectory is started.
The trajectory is planned as a sequence of randomly selected waypoints, plus a
conrtibution on acceleration for collision avoidance.
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
import random

import trajectory_msgs.msg as tm
import geometry_msgs.msg as gm
import nav_msgs.msg as nm

import planners.trajectory_circle as ct
import planners.trajectory_cubic as cbt
import planners.trajectory_quintic as qt
import planners.planner_collision_avoidance as pca
import planners.planner_to_goal as ptg

import tf.transformations as tft




class RotorSCollisionAvoidancePointSequencePlannerNode():

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
        
     
    def _get_quad_pos_vel(self, msg):
        """Callback to save the position and velocity of the quad into
        object properties.
        """
        
        self._pos, self._vel = self._odometry_to_pos_vel(msg)
        self._got_initial_pos_vel_flag = True    
    

    def _get_other_pos(self, msg):
    
        self._other_pos, dummy = self._odometry_to_pos_vel(msg)
        self._got_other_initial_pos_flag = True
        

    def _generate_initial_waypoint(self):
        if self._pos[2] < 1.0:
            self._waypoint = numpy.array(self._pos)
            self._waypoint[2] = 1.0
            #duration = 2.5*numpy.linalg.norm(self._waypoint-self._pos)
            #time = rospy.get_time() - self.initial_time
            #delay = rospy.get_param('delay', default=1.0)
            #self._roaming_planner = cbt.TrajectoryCubic(self._pos, numpy.eye(3), delay+time, delay+time+duration, self._waypoint)
            self._roaming_planner = ptg.PlannerToGoal(self._waypoint, 1.0, 5.0)
        else:
            self._generate_waypoint()


    def _generate_waypoint(self):
        self._waypoint = numpy.array(self._pos)
        distance = numpy.linalg.norm(self._waypoint-self._pos)
        while distance < 0.1:
            #x = random.uniform(-1.5, 1.5)
            #y = random.uniform(-1.5, 1.5)
            #z = random.uniform(1.0, 2.5)
            #yaw = random.uniform(-numpy.pi, numpy.pi)
            #self._waypoint = numpy.array([x, y, z, yaw])
            self._waypoint = numpy.array(self._other_pos)
            distance = numpy.linalg.norm(self._waypoint-self._pos)
        #duration = 2.0*numpy.linalg.norm(self._waypoint-self._pos)
        #time = rospy.get_time() - self.initial_time
        #delay = rospy.get_param('delay', default=1.0)
        #self._roaming_planner = cbt.TrajectoryCubic(self._pos, numpy.eye(3), delay+time, delay+time+duration, self._waypoint)
        self._roaming_planner = ptg.PlannerToGoal(self._waypoint, 1.0, 5.0)



    def work(self):

        # initialize node
        rospy.init_node('rotors_collision_avoidance_point_sequence_planner_node')
        
        # planner to compute the collision avoidance contributions
        gain = 4.0
        ths = 6.0
        self._collision_avoidance_planner = pca.PlannerCollisionAvoidance(gain, ths)
        
        # planner to roam around
        # it will be set up in the callback
        self._roaming_planner = None
        
        # instantiate the publisher
        topic = rospy.get_param('ref_traj_topic', default='command/trajectory')
        pub = rospy.Publisher(topic, tm.MultiDOFJointTrajectory, queue_size=10)

        # to get the initial position of the quad
        self._pos = None
        self._vel = None
        self._got_initial_pos_vel_flag = False
        topic = rospy.get_param('quad_pos_vel_topic', default='ground_truth/odometry')
        self._pos_subscriber = rospy.Subscriber(topic, nm.Odometry, self._get_quad_pos_vel)

        # subscriber to the position of the leader
        self._other_pos = None
        self._got_other_initial_pos_flag = False
        topic = rospy.get_param('other_pos_topic', default='/hummingbird_1/ground_truth/odometry')
        self._other_pos_subscriber = rospy.Subscriber(topic, nm.Odometry, self._get_other_pos)

        # setting the frequency of execution
        freq = 1e1
        rate = rospy.Rate(freq)

        # get initial time
        #aux = rospy.get_time()
        #while rospy.get_time() == aux:
        #    pass
        #self.initial_time = rospy.get_time()
        #self.time = rospy.get_time() - self.initial_time
        #print(self.time)
        #print(type(self.initial_time))

        # get initial quad position
        while not self._got_initial_pos_vel_flag:
            rate.sleep()

        # get initial position of the other
        while not self._got_other_initial_pos_flag:
            rate.sleep()

        # generate initial waypoint
        self._generate_initial_waypoint()
        waypoint_counter = 0

        # do work
        while not rospy.is_shutdown():

            # compute collision avoidance contribution
            ca_acc = self._collision_avoidance_planner.get_acceleration(self._pos, self._vel, self._other_pos)

            # get current time
            #self.time = rospy.get_time() - self.initial_time
            #print(self.time)
            
            # see if the current waypoint is reached
            #print self._pos
            #print self._waypoint
            #print numpy.linalg.norm(self._pos-self._waypoint)
            
            if numpy.linalg.norm(self._pos-self._waypoint) < 0.15:
                #print("\nNEW WAYPOINT!!!: " + str(waypoint_counter) + "\n")
                #print("\nTIME: " + str(rospy.get_time()) + "\n")
                waypoint_counter += 1
                self._generate_waypoint()
                
            #print("TO GOAL: " + str(numpy.linalg.norm(self._pos-self._waypoint)))
            print("GOAL: " + str(self._waypoint))
            print("CURRENT: " + str(self._pos))
            print("TIME: " + str(rospy.get_time()))
            print("WAYPOINTS REACHED: " + str(waypoint_counter))
                
            r_acc = self._roaming_planner.get_acceleration(self._pos, self._vel)

            acc = ca_acc + r_acc
            
            aux = self._roaming_planner.get_velocity(self._pos)
            vel = numpy.array(self._vel)
            vel[3] += aux[3]
            
            pos = numpy.array(self._pos)
            
            j = numpy.zeros(4)
            sn = numpy.zeros(4)
            cr = numpy.zeros(4)
            
            msg = self._trajectory_to_multi_dof_joint_trajectory(pos, vel, acc, j, sn, cr)
            pub.publish(msg)
            
            rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    node = RotorSCollisionAvoidancePointSequencePlannerNode()
    node.work()
    
