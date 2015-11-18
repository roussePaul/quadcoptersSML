"""This file collects some parameters for the simulator of the quad.
The parameters are not defined directly here, but they are defined in a launch
file.
This file takes out the parameters from the launch file, but also gives a
default value for each parameter.
In this way, ROS is able to change the values of the parameters online, if
desired.
For example, we do such changes in the GUI.
"""

import rospy
import numpy

# acceleration due to gravity (m/s^2)
g = rospy.get_param("g_sim",9.81)

# quad mass (kg)
quad_mass = rospy.get_param("quad_mass_sim",1.442)

# neuthral throttle (throttle that cancels the gravity)
neuthral_throttle = rospy.get_param("throttle_neutral_sim",1484.0)

# angular velocity sensitivity in the acro mode
acro_rpp = rospy.get_param("acro_rpp_sim",4.5)

# maximum roll and pitch angle
max_roll_pitch_deg = rospy.get_param("max_roll_pitch_deg_sim",45.0)

# inner loop gain
ktt_inner_loop = rospy.get_param("ktt_inner_loop_sim",10.0)

# maximum yaw speed
max_yaw_speed_deg = rospy.get_param("max_yaw_speed_deg_sim",200.0)

# test
# print(g)
