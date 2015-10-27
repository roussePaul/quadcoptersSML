import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4.QtCore import QObject, pyqtSignal


from quad_control.srv import Controller_Srv

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

#from mocap.msg import Record

#import analysis
#import utils
import subprocess


import argparse


# import services defined in quad_control
# TWO services being imported
from quad_control.srv import *


class saver_mavrosPlugin(Plugin):


    
    def __init__(self, context,namespace = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())


        super(saver_mavrosPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('saver_mavrosPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns
        
        
        
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'saver_mavros.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('saver_mavrosUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # ---------------------------------------------- #
        # ---------------------------------------------- #

        # BUTTON TO START EXPERIMENT
        self._widget.ButtonSTART_SIM.stateChanged.connect(self.StartSimulatorService)

        # Button to Reset Simulator
        self._widget.ResetSimulator.clicked.connect(self.ResetSimulator)

        # BUTTON TO CONNECT TO QUALISYS
        self._widget.Qualisys.stateChanged.connect(self.QualisysConnect)

        # BUTTON TO CHANGE ID
        self._widget.changeID.clicked.connect(self.changeID)

        # Default Value for Id of Quad (as in QUALISYS computer)
        self._widget.ID.setValue(13)


        # SERVICES REALATED TO MAVROS
        # Button to have controller sending minimum throttle and neutral values for other inputs
        self._widget.MinThrottle.clicked.connect(self.MinThrottle)    

        # Button to Arm Quad
        self._widget.ArmQuad.clicked.connect(self.Arming_Quad)    
        # Button to UnArm Quad
        self._widget.UnArmQuad.clicked.connect(self.UnArming_Quad) 

        self._widget.MODE_LAND.stateChanged.connect(self.ChangeMode)
        self._widget.MODE_STABILIZE.stateChanged.connect(self.ChangeMode)
        self._widget.MODE_ACRO.stateChanged.connect(self.ChangeMode)

    def MinThrottle(self):
        #Change the flight mode on the Pixhawk flight controller
        try:
            # it waits for service for 2 seconds
            rospy.wait_for_service(self.namespace+'Controller_GUI',1.0)

            try:
                AskForMinThrotlle = rospy.ServiceProxy(self.namespace+'Controller_GUI',Controller_Srv)
                
                answer = AskForMinThrotlle(4,None)
                if answer.received:
                    rospy.logwarn('Min Throttle')
                    # return True
                else:
                    rospy.logwarn('Could not provide Service')
                    # return False
            except:
                rospy.logwarn('Could not provide Service')
                # return False

        except:
            rospy.logwarn('Could not provide Service')
            # return False

        
    def ChangeMode(self):
        if self._widget.MODE_LAND.isChecked():
            self.Set_Flight_Mode('LAND')

        if self._widget.MODE_STABILIZE.isChecked():
            self.Set_Flight_Mode('STABILIZE')

        if self._widget.MODE_ACRO.isChecked():
            self.Set_Flight_Mode('ACRO')


    def Set_Flight_Mode(self,MODE):
        
        #Change the flight mode on the Pixhawk flight controller
        try:
            # it waits for service for 2 seconds
            rospy.wait_for_service(self.namespace+'mavros/set_mode',2.0)

            try:
                change_param = rospy.ServiceProxy(self.namespace+'mavros/set_mode',SetMode)
                param=change_param(0,MODE)

                if param.success:
                    rospy.logwarn('Flight mode changed to '+MODE)
                    # return True
                else:
                    rospy.logwarn('Could not change Flight mode')
                    # return False
            except:
                rospy.logwarn('Mavros is not available')
                # return False

        except:
            rospy.logwarn('Mavros is not available')
            # return False

    def Arming_Quad(self,base_name=""):
        #This function is used to arm the quad

        #Arming the Quad
        srv_path = self.namespace+'mavros/cmd/arming'
        # if base_name!="":
            # srv_path = "/%s/%s"%(base_name,srv_path)

        try:

            rospy.logwarn('Arming Quad ...')
            rospy.wait_for_service(srv_path,2.0)

            try:
                arming = rospy.ServiceProxy(srv_path,CommandBool)
                arming_result=arming(True)
                if arming_result.success:
                    rospy.logwarn('Quad is Armed!!!!')
                    # return True
                else:
                    rospy.logwarn('Cannot arm quad')
                    # return False

            except:
                rospy.logwarn('Cannot arm quad')
                # return False

        except:
            rospy.logwarn('No connection to Mavros')
            # return False    


    def UnArming_Quad(self,base_name=""):
        #This function is used to arm the quad."""

        #Un-Arming the Quad
        srv_path = self.namespace+'mavros/cmd/arming'
        # if base_name!="":
            # srv_path = "/%s/%s"%(base_name,srv_path)

        try:
            rospy.logwarn('Un-Arming Quad ...')
            rospy.wait_for_service(srv_path,2.0)

            try:
                arming = rospy.ServiceProxy(srv_path,CommandBool)
                arming_result=arming(False)
                if arming_result.success:
                    rospy.logwarn('Quad is Un-Armed!!!!')
                    # return True
                else:
                    rospy.logwarn('Cannot Un-arm quad')
                    # return False

            except:
                rospy.logwarn('Cannot Un-arm quad')
                # return False

        except:
            rospy.logwarn('No connection to Mavros')
            # return False   


    #@Slot(bool)
    def StartSimulatorService(self):
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'StartSimulator',1.0)
            
            try:
                AskForStart = rospy.ServiceProxy(self.namespace+'StartSimulator', StartSim)

                # if button is pressed save data
                if self._widget.ButtonSTART_SIM.isChecked():
                    # request controller to save data
                    reply = AskForStart(True)
                    if reply.Started == True:
                        # if controller receives message, we know it
                        # print('Started')
                        self._widget.SimulatorSuccess.setChecked(True) 
                        self._widget.SimulatorFailure.setChecked(False) 
                else:
                    # request simulator to freeze and restart
                    reply = AskForStart(False)
                    if  reply.Started == True:
                        # if controller receives message, we know it
                        # print('Stopped')
                        self._widget.SimulatorSuccess.setChecked(True) 
                        self._widget.SimulatorFailure.setChecked(False) 

            except rospy.ServiceException:
                # print "Service call failed: %s"%e   
                self._widget.SimulatorSuccess.setChecked(False) 
                self._widget.SimulatorFailure.setChecked(True) 
            
        except: 
            # print "Service not available ..."        
            self._widget.SimulatorSuccess.setChecked(False) 
            self._widget.SimulatorFailure.setChecked(True)
            pass  


    #@Slot(bool)
    def ResetSimulator(self):
        
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'ResetSimulator',1.0)
            
            try:
                AskForStart = rospy.ServiceProxy(self.namespace+'ResetSimulator', StartSim)

                # if button is pressed save data
                if self._widget.ButtonSTART_SIM.isChecked():
                    # request controller to save data
                    reply = AskForStart(True)
                    if reply.Started == True:
                        # if controller receives message, we know it
                        # print('Reseted')
                        self._widget.SimulatorSuccess.setChecked(True) 
                        self._widget.SimulatorFailure.setChecked(False) 
                    else:
                        self._widget.SimulatorSuccess.setChecked(False) 
                        self._widget.SimulatorFailure.setChecked(True)                         
                # else: do nothing

            except rospy.ServiceException:
                # print "Service call failed: %s"%e   
                self._widget.SimulatorSuccess.setChecked(False) 
                self._widget.SimulatorFailure.setChecked(True) 
            
        except: 
            # print "Service not available ..."        
            self._widget.SimulatorSuccess.setChecked(False) 
            self._widget.SimulatorFailure.setChecked(True)
            pass  


    #@Slot(bool)
    def QualisysConnect(self):
        
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'Mocap_Set_Id',1.0)
            
            try:
                AskMocap = rospy.ServiceProxy(self.namespace+'Mocap_Set_Id', Mocap_Id)

                if self._widget.Qualisys.isChecked() == True:
                    reply = AskMocap(True,self._widget.ID.value(),True)

                    if reply.success == True:
                        # if controller receives message, we know it
                        self._widget.QualisysSuccess.setChecked(True) 
                        self._widget.QualisysFailure.setChecked(False) 
                    if reply.exists == True:
                        self._widget.Exists.setChecked(True)
                        self._widget.ExistsNot.setChecked(False)
                    else:
                        self._widget.Exists.setChecked(False)
                        self._widget.ExistsNot.setChecked(True)
                else:
                    reply = AskMocap(False,self._widget.ID.value(),True)
                    
                    if reply.success == True:
                        # if controller receives message, we know it
                        self._widget.QualisysSuccess.setChecked(True) 
                        self._widget.QualisysFailure.setChecked(False) 

                    self._widget.Exists.setChecked(False)
                    self._widget.ExistsNot.setChecked(False)


            except rospy.ServiceException:
                # print "Service call failed: %s"%e   
                self._widget.QualisysSuccess.setChecked(False) 
                self._widget.QualisysFailure.setChecked(True) 
                self._widget.Exists.setChecked(False)
                self._widget.ExistsNot.setChecked(False)
            
        except: 
            # print "Service not available ..."        
            self._widget.QualisysSuccess.setChecked(False) 
            self._widget.QualisysFailure.setChecked(True)
            self._widget.Exists.setChecked(False)
            self._widget.ExistsNot.setChecked(False)            
            pass  


    #@Slot(bool)
    def changeID(self):
        
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'Mocap_Set_Id',1.0)
            
            try:
                AskMocap = rospy.ServiceProxy(self.namespace+'Mocap_Set_Id', Mocap_Id)

                if self._widget.Qualisys.isChecked() == True:
                    reply = AskMocap(True,self._widget.ID.value(),True)
                    print(reply)
                    if reply.success == True:
                        # if controller receives message, we know it
                        self._widget.QualisysSuccess.setChecked(True) 
                        self._widget.QualisysFailure.setChecked(False)

                    if reply.exists == True:
                        self._widget.Exists.setChecked(True)
                        self._widget.ExistsNot.setChecked(False)
                    else:
                        self._widget.Exists.setChecked(False)
                        self._widget.ExistsNot.setChecked(True)

            except rospy.ServiceException:
                # print "Service call failed: %s"%e   
                self._widget.QualisysSuccess.setChecked(False) 
                self._widget.QualisysFailure.setChecked(True) 
                self._widget.Exists.setChecked(False)
                self._widget.ExistsNot.setChecked(False)
            
        except: 
            # print "Service not available ..."        
            self._widget.QualisysSuccess.setChecked(False) 
            self._widget.QualisysFailure.setChecked(True)
            self._widget.Exists.setChecked(False)
            self._widget.ExistsNot.setChecked(False)            
            pass


    def _parse_args(self, argv):

        parser = argparse.ArgumentParser(prog='saver', add_help=False)

        # args = parser.parse_args(argv)

        if argv:
            namespace = argv[0]
            return namespace            
        else:
            # is argv is empty return empty string
            return ""




    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    
