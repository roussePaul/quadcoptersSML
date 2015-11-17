"""This file implements some utility functions
"""

import numpy
from numpy import cos as c
from numpy import sin as s
from numpy import pi


def rot_mat_x(tt):
    """This function returns the rotation matrix corresponding to a rotation
	of tt radians about the x-axis.
	"""
    return numpy.array(
        [[1.0, 0.0, 0.0], [0.0, c(tt), -s(tt)], [0.0, s(tt), c(tt)]])

# print Rx(60*3.14/180)


def rot_mat_y(tt):
    """This function returns the rotation matrix corresponding to a rotation
	of tt radians about the y-axis.
	"""
    return numpy.array(
        [[c(tt), 0.0, s(tt)], [0.0, 1, 0.0], [-s(tt), 0.0, c(tt)]])

# print Ry(60*3.14/180)


def rot_mat_z(tt):
    """This function returns the rotation matrix corresponding to a rotation
	of tt radians about the z-axis.
	"""
    return numpy.array(
        [[c(tt), -s(tt), 0.0], [s(tt), c(tt), 0.0], [0.0, 0.0, 1]])

# print Rz(60*3.14/180)


def skew(v):
	"""This function returns the skew matrix corresponding to the vector xx.
	The vector xx should be in R^3 and the returned skew matrix is 3-by-3.
	"""
	x = v[0]
	y = v[1]
	z = v[2]
	return numpy.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])

# print skew([1,2,3])


def orthogonal_projector(v):
	"""This function returns the orthognal projector operator of the vector v.
	The vector v should be in R^3 and the returned orthogonal projector is a
	3-by-3 matrix.
	"""
	return -skew(v).dot(skew(v))

# print OP([1,2,3])
# print OP([1,0,0])


def unit_vec(psi, theta):
	"""This function returns the unit vector corresponding to the euler angles
	psi (about the z-axis) and theta (about the y-axis).
	"""
	e1 = numpy.array([1.0, 0.0, 0.0])
	aux = rot_mat_z(psi).dot(e1)
	aux = rot_mat_y(theta).dot(aux)
	return aux

# print unit_vec(45*3.14/180,0)
# print unit_vec(45*3.14/180,45*3.14/180)
# print unit_vec(0*3.14/180,-90*3.14/180)


def saturate(x, max_val, min_val):
	"""This function returns the saturated version of a scalar x,
	with saturation limits max_val from above and min_val from below.
	"""
	return max(min_val, min(max_val, x))


def rot_max_to_euler_angles(rot_max):
	"""This function computes the euler angles corresponding to the rotation
	matrix rot_max.
	"""
    # phi   = atan2(R(3,2),R(3,3));
    # theta = asin(-R(3,1));
    # psi   = atan2(R(2,1),R(1,
	phi = numpy.arctan2(
	    saturate(rot_max[2, 1], 1, -1), saturate(rot_max[2, 2], 1, -1));
	theta = numpy.arcsin(-saturate(rot_max[2, 0], 1, -1));
	psi = numpy.arctan2(
	    saturate(rot_max[1, 0], 1, -1), saturate(rot_max[0, 0], 1, -1));
	return phi, theta, psi


def rot_max_to_euler_angles_deg(rot_max):
	"""This function returns the euler angles corresponding to the rotation
	matrix rot_max in degrees.
	"""
	return rot_max_to_euler_angles(rot_max) * 180.0 / pi


def euler_angles_to_rot_max(ee_rad):
	"""This function returns the rotation matrix corresponding to the euler
	angles ee_rad.
	"""
	aux = rot_max_z(ee_rad[2])
	aux = aux.dot(rot_max_y(ee_rad[1]))
	aux = aux.dot(rot_max_x(ee_rad[0]))
	return aux


def euler_angles_deg_to_rot_max(ee_deg):

    return euler_angles_to_rot_max(ee_deg * pi / 180.0)
