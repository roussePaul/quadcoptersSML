#!/usr/bin/env python
import rospy
from mavros_msgs.msg import OverrideRCIn

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

import time
import serial
import serial.tools
import array
import sys

# Add some mora advanced safety features:
class serialRC:
	def __init__(self,port):
		ser = serial.Serial(port=port)

		if not ser.isOpen():
		    print "Port is already opened"
		    ser.close()
		    ser.open()

		#ser.setDTR(False)
		time.sleep(1)
		# toss any data already received, see
		# http://pyserial.sourceforge.net/pyserial_api.html#serial.Serial.flushInput
		ser.flushInput()
		ser.flushOutput()
		ser.setDTR(True)

		ser = serial.Serial(port=port,
		                    baudrate=115200,
		                    bytesize=serial.EIGHTBITS,
		                    parity=serial.PARITY_NONE,
		                    stopbits=serial.STOPBITS_ONE,
		                    timeout=1.0,
		                    xonxoff=0,
		                    rtscts=0
		                    )
		self.ser = ser
		self.chan = [1500]*8
		self.mode = 1147
		self.cmd=False
		self.cmd_chan = self.chan

	# chan: array of 8 intergers (between 1000 and 2000)
	def getChan(self,data):
		if self.cmd==True:
			self.chan = self.cmd_chan
		else:
			self.chan = list(data.channels)
		self.chan[4] = self.mode
		self.sendRC()

		
	def sendRC(self):
		c =  [ch-23 for ch in self.chan]
		data = array.array('h',c+[0]).tostring()
		self.ser.write(data)
		#print self.chan

	def stopRC(self):
		self.ser.close()

	def set_mode(self,data):
		if data.custom_mode == 'STABILIZE':
			self.mode = 1100
		elif data.custom_mode == 'ACRO':
			self.mode = 1300
		else:
			self.mode = 1400
		return True

	def arm(self):
		self.cmd=True
		self.cmd_chan[2] = 1000
		self.cmd_chan[3] = 2000
		print self.cmd_chan
		self.sendRC()
		rospy.sleep(3.0)
		self.cmd_chan[2] = 1000
		self.cmd_chan[3] = 1500
		self.sendRC()
		self.cmd=False

	def disarm(self):
		self.cmd=True
		self.cmd_chan[2] = 1000
		self.cmd_chan[3] = 1000
		self.sendRC()
		rospy.sleep(3.0)
		self.cmd_chan[2] = 1000
		self.cmd_chan[3] = 1500
		self.sendRC()
		self.cmd=False

	def cmd_arm(self,data):
		if data.value:
			self.arm()
		else:
			self.disarm()
		return {'success':True,'result':0}

def rosRC():
	rospy.init_node('chan2serial', anonymous=True)
	
	port = sys.argv[1]
	print "RC connect to " + port
	ser = serialRC(port)

	rospy.Service('mavros/set_mode', SetMode, ser.set_mode)
	rospy.Service('mavros/cmd/arming', CommandBool, ser.cmd_arm)
	rospy.Subscriber("mavros/rc/override", OverrideRCIn, ser.getChan)

	rospy.on_shutdown(ser.stopRC)
	rate = rospy.Rate(100)

	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == '__main__':
	try:
		rosRC()
	except rospy.ROSInterruptException:
		pass