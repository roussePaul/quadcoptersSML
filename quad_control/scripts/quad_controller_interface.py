## Quad Api
# implement all the function that communicate with the quad and with the controller
import rospy
from std_srvs.srv import Empty

from quad_control.msg import quad_state_and_cmd
from quad_control.srv import *
from mavros_msgs.msg import OverrideRCIn

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

import numpy

## TODO List:
# Create a quad API that implement all the functions with errors taken in account
# 

class Quad:
	def __init__(self,ns):
		self.ns = ns

		self.qualisys_check = False
		self.qualisys_error = "NOT STARTED"
		self.state = quad_state_and_cmd()

		self.rc_check = False
		self.rc_error = "NOT STARTED"
		self.rc_msg = OverrideRCIn()

		self.arm_check = False
		self.arm_error = False

		self.mode_check = False
		self.mode_error = False

		self.traj_check = False
		self.traj_error = False
		self.traj_point = None

		self.namespace  = "/" + self.ns + "/"

		self.connect_to_ROS()


	# Connection to ROS topics
	def connect_to_ROS(self):
		rospy.Subscriber(self.namespace+"quad_state_and_cmd", quad_state_and_cmd, self.data_from_qualisys)
		rospy.Subscriber(self.namespace+"mavros/rc/override", OverrideRCIn, self.data_to_rc)


	def call_service(self,srv_name,mtype,*arg):
		success = False
		error = ""
		reply = None
		try: 
			# time out of one second for waiting for service
			rospy.wait_for_service(srv_name,1.0)
			try:
				service = rospy.ServiceProxy(srv_name, mtype)
				reply = service(*arg)
				success = True
			except Exception, e:
				error = "did not answered : " + str(e)
		except Exception, e: 
			error = "unavailable : " + str(e)

		return (reply,success,error)

	# Connection to Qualisys
	def connect_to_qualysis(self):
		reply,success,error = self.call_service(self.namespace+'Mocap_Set_Name',Mocap_Name, True,self.ns,True)
		success = success and reply.success
		error = error+reply.error
		self.qualisys_error = error
		return (success,error)

	def data_from_qualisys(self,state):
		self.qualisys_check = (float(self.state.x) != float(state.x))
		self.state = state

	def data_to_rc(self,rc_msg):
		self.rc_check = True
		self.rc_msg = rc_msg
		self.rc_error = "OK"

	def arm(self):
		#This function is used to arm the quad

		self.arm_error = 'Arming Quad ...'
		reply,success,error = self.call_service(self.namespace+'mavros/cmd/arming',CommandBool, True)

		if success:
			if reply.success:
				error = error + 'Armed'
				success = True
			else:
				error = error + 'Cannot arm the quad'
				success = False

		self.arm_error = error
		self.arm_check = success
		return (success,error)
	
	def unarm(self):
		#This function is used to arm the quad

		self.arm_error = 'UnArming Quad ...'
		reply,success,error = self.call_service(self.namespace+'mavros/cmd/arming',CommandBool, False)

		if success:
			if reply.success:
				error = error + 'UnArmed'
				success = False
			else:
				error = error + 'Cannot arm the quad'
				success = False

		self.arm_error = error
		self.arm_check = success
		return (success,error)

	def set_flight_mode(self,mode):
		#Change the flight mode on the Pixhawk flight controller
		reply,success,error = self.call_service(self.namespace+'mavros/set_mode',SetMode, 0,mode)

		if success:
			if reply.success:
				error = error + mode
				success = True
			else:
				error = error + 'Could not change Flight mode : ' + mode
				success = False
		self.mode_error = error
		self.mode_check = success
		return (success,error)

	def set_PID_reference_point(self,x,y,z):
		traj,offset,rotation,parameters = self.fixed_point(x,y,z)
		reply,success,error = self.call_service(self.namespace+'TrajDes_GUI',TrajDes_Srv, traj,offset,rotation,parameters)
		if success:
			if reply.received == True:
				error = error + "OK"
				success = True
				self.traj_point = [x,y,z]
			else:
				error = error + "Service not available"
				success = False

		return (success,error)

	def fixed_point(self,x,y,z):

		traj = 0
		offset = numpy.array([x,y,z])
		
		rotation = numpy.array([0.0,0.0,0.0])

		parameters = None

		return traj,offset,rotation,parameters


	#@Slot(bool)
	def set_offboard_controller(self):
		P = 2.9
		D = 2.4
		I = 0.1
		P_z = 1.0
		D_z = 1.4
		I_z = 0.1
		TN = 1400

		parameters = numpy.array([P,D,I,P_z,D_z,I_z,TN,rospy.get_time()])

		controller = 1
		reply,success,error = self.call_service(self.namespace+'Controller_GUI', Controller_Srv, controller,parameters)

		if success:
			if reply.received:
				success = True
			else:
				success = False
				error = error+"Impossible to change the controller"
		return (success,error)

	def goto_z(self,z):
		self.set_PID_reference_point(self.state.x,self.state.y,z)
	def goto_xy(self,x,y):
		self.set_PID_reference_point(x,y,self.state.z)
