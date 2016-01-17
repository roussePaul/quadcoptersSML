import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from std_srvs.srv import Empty
from PyQt4.QtCore import *
from PyQt4.QtGui import * 


from quad_control.msg import quad_state_and_cmd
from quad_control.srv import *
from mavros_msgs.msg import OverrideRCIn

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

import subprocess

import re
import argparse
import threading

import numpy

import sys
sys.path += ['/home/smladmin/catkin_ws/src/quadcoptersSML/quad_control/scripts']
from quad_controller_interface import Quad

## TODO List:
# Create a quad API that implement all the functions with errors taken in account
# Fix the sys path problem to have a working one

def call_command(quad,cmd,arg):
    if cmd == multiQuadGuiPlugin.CMD_LAND:
        quad.set_flight_mode('LAND')

    if cmd == multiQuadGuiPlugin.CMD_QUALYSIS_CONNECT:
        quad.connect_to_qualysis()

    if cmd == multiQuadGuiPlugin.CMD_STOP:
        pass

    if cmd == multiQuadGuiPlugin.CMD_GO_UP:
        quad.goto_z(2.0)

    if cmd == multiQuadGuiPlugin.CMD_GO_MIDDLE:
        quad.goto_z(1.5)

    if cmd == multiQuadGuiPlugin.CMD_GO_DOWN:
        quad.goto_z(0.7)

    if cmd == multiQuadGuiPlugin.CMD_GOTO:
        quad.goto_xy(arg[0],arg[1])

    if cmd == multiQuadGuiPlugin.CMD_ARM:
        quad.arm()

    if cmd == multiQuadGuiPlugin.CMD_UNARM:
        quad.unarm()

    if cmd == multiQuadGuiPlugin.CMD_SET_MODE:
        quad.set_flight_mode('STABILIZE')

    if cmd == multiQuadGuiPlugin.CMD_SET_CONTROLLER:
        quad.set_offboard_controller()

class Arena(QtGui.QWidget):
    def __init__(self,parent):
        super(Arena, self).__init__()
        self.w = 400 
        self.h = 400 
        self.setFixedSize(self.w,self.h)
        self.parent = parent

    def trans(self,x,y):
        px = x/2.0*200+200
        py = y/2.0*200+200
        return (px,py)

    def itrans(self,px,py):
        x = (px-200.)/200.*2.0
        y = (py-200.)/200.*2.0
        return (x,y)

    def draw_quad(self,quad,qp,selected):
        if selected:
            qp.setBrush(Qt.blue)
        else:
            qp.setBrush(Qt.white)
        (px,py) = self.trans(quad.state.x,quad.state.y)
        qp.drawEllipse(QPoint(px,py), 7, 7)
        qp.drawText(QPoint(px,py-10), quad.ns)        


    def draw_waypoint(self,quad,qp):
        if quad.traj_point==None:
            return
        qp.setBrush(Qt.green)
        (px,py) = self.trans(quad.state.x,quad.state.y)
        (wx,wy) = self.trans(quad.traj_point[0],quad.traj_point[1])
        qp.drawEllipse(QPoint(wx,wy), 3, 3)
        qp.drawLine(px,py,wx,wy)

    def draw(self,quad,qp,selected):
        self.draw_quad(quad,qp,selected)
        self.draw_waypoint(quad,qp)

    def getTable(self,quad):
        return [(quad.ns,None),
                (str(quad.qualisys_error),quad.qualisys_check),
                (str(quad.rc_error),quad.rc_check),
                (str(quad.mode_error),quad.mode_check),
                (str(quad.arm_error),quad.arm_check),
                ("",None)]

    def paintEvent(self, event):
        qp = QPainter()
        qp.begin(self)
        qp.setBrush(Qt.yellow)
        qp.drawRect(0,0,self.w-1,self.h-1)
        for (qid,q) in self.parent.quads.iteritems():
            self.draw(q,qp,qid in self.parent.quad_select)
        qp.end()

    def mousePressEvent(self, event):
        wp = self.itrans(event.x(),event.y())
        self.parent.call_command(multiQuadGuiPlugin.CMD_GOTO,wp)

class multiQuadGuiPlugin(Plugin):

    CMD = []
    CMD_LAND                = ("LAND",              Qt.Key_L)
    CMD_QUALYSIS_CONNECT    = ("QUALISYS CONNECT",  Qt.Key_C)
    CMD_STOP                = ("STOP",              Qt.Key_S)
    CMD_GO_UP               = ("GO UP",             Qt.Key_Q)
    CMD_GO_MIDDLE           = ("GO MIDDLE",         Qt.Key_A)
    CMD_GO_DOWN             = ("GO DOWN",           Qt.Key_Z)
    CMD_GOTO                = ("GOTO",              Qt.Key_G)
    CMD_ARM                 = ("ARM",               Qt.Key_T)
    CMD_UNARM               = ("UNARM",             Qt.Key_Y)
    CMD_SET_MODE            = ("SET_MODE",          Qt.Key_U)
    CMD_SET_CONTROLLER      = ("SET_CONTROLLER",    Qt.Key_I)
    
    CMD = [CMD_LAND,CMD_QUALYSIS_CONNECT,CMD_STOP,CMD_GO_UP,CMD_GO_MIDDLE,CMD_GO_DOWN,CMD_GOTO,CMD_ARM,CMD_UNARM,CMD_SET_MODE,CMD_SET_CONTROLLER]
    
    def __init__(self, context,namespace = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())


        super(multiQuadGuiPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('multiQuadGuiPlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'multiQuadGui.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('multiQuadGuiUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface

        self.arena = Arena(self)
        self._widget.layout().addWidget(self.arena,0,1)

        self._widget.keyPressEvent = self.keyPressEvent
        self._widget.keyReleaseEvent = self.keyReleaseEvent
        self._widget.table.keyPressEvent = self.keyPressEvent
        self._widget.table.keyReleaseEvent = self.keyReleaseEvent
        
        context.add_widget(self._widget)
        
        # # Adding all the tabs

        self.quad_ns_list = []
        self.quads = {}

        self.quad_select = set()
        self.command = None

        self.update_timer = QTimer(self)
        self.update_timer.setInterval(1000)
        self.update_timer.timeout.connect(self.update_quad_list)
        self.update_timer.start()

        self.update_draw = QTimer(self)
        self.update_draw.setInterval(1000/30)
        self.update_draw.timeout.connect(self.arena.repaint)
        self.update_draw.timeout.connect(self.update_table)
        self.update_draw.start()


        table_sizes = [25,100,150,60,100,120,70,70,100]
        for i in range(len(table_sizes)):
            self._widget.table.setColumnWidth(i,table_sizes[i])

    def _parse_args(self, argv):
        parser = argparse.ArgumentParser(prog='saver', add_help=False)
        # args = parser.parse_args(argv)
        if argv:
            namespace = argv[0]
            return namespace            
        else:
            # is argv is empty return empty string
            return ""

    def get_id(self,ns):
        i = re.findall('\d+', ns)[0]
        return int(i)

    #@Slot(bool)
    def update_quad_list(self):
        self.quad_ns_list = rospy.get_param("/quad_ns_list")

        for ns in self.quad_ns_list:
            qid = self.get_id(ns)
            if qid not in self.quads.keys():
                self.add_quad(ns,qid)

    def add_quad(self,ns,qid):
            print "Add new quad: " + str(ns)
            self.quads[qid] = Quad(ns)

            idR = self.getRowId(qid)
            self._widget.table.insertRow(idR)


    def getRowId(self,idq):
        ids = sorted(self.quads.keys())
        return ids.index(idq)

    #@Slot(bool)
    def update_table(self):
        for (idq,q) in self.quads.iteritems():
            t = self.arena.getTable(q)
            idT = self.getRowId(idq)
            t = [(idq,None)] + t
            for (idx,value) in enumerate(t):
                item = self._widget.table.item(idT, idx)
                if item==None:
                    item = QTableWidgetItem()
                    self._widget.table.setItem(idT,idx,item)

                if idx==0:
                    if idq in self.quad_select:
                        item.setBackground(QColor(25,0,255,100))
                    else:
                        item.setBackground(Qt.white)
                else:
                    if value[1] == False:
                        item.setBackground(QColor(250,0,25,150))
                    elif value[1] == True:
                        item.setBackground(QColor(0,250,25,150))
                    else:
                        item.setBackground(Qt.white)
                item.setText(str(value[0]))

    def print_console(self,s):
        self._widget.console.append(s)

    def call_command(self,cmd,arg=None):
        for i in self.quad_select:
            func = lambda : call_command(self.quads[i],cmd,arg)
            threading.Thread(target=func).start()

        self.print_console(' '.join([str(l) for l in self.quad_select]) + " " + cmd[0])

    def select(self,i):
        if i==0:
            for d in self.quads.keys():
                self.quad_select.add(d)
        if i in self.quads.keys():
            self.quad_select.add(i)
        self.update_table()

    def unselect(self,i):
        if i==0:
            self.quad_select.clear()
        if i in self.quads.keys() and i in self.quad_select:
            self.quad_select.remove(i)
        self.update_table()


    def keyPressEvent(self, event):
        for c in multiQuadGuiPlugin.CMD:
            if type(event) == QKeyEvent and event.key() == c[1]: 
                self.call_command(c)

        if type(event) == QKeyEvent and (event.key() >= Qt.Key_0 and event.key() <= Qt.Key_9): 
            self.select(event.key()-Qt.Key_0)

    def keyReleaseEvent(self,event):
        if type(event) == QKeyEvent and (event.key() >= Qt.Key_0 and event.key() <= Qt.Key_9): 
            self.unselect(event.key()-Qt.Key_0)


    # def execute(self,cmd):
    #     #subprocess.Popen(["bash","-c","cd "+self.pwd+"/src/kampala/gui/scripts; echo "+cmd+" > pipefile" + self.name]) 


    # def shutdown_plugin(self):
    #     # TODO unregister all publishers here
    #     pass

    # def save_settings(self, plugin_settings, instance_settings):
    #     # TODO save intrinsic configuration, usually using:
    #     # instance_settings.set_value(k, v)
    #     instance_settings.set_value("irisindex", self._widget.IrisInputBox.currentIndex())

    # def restore_settings(self, plugin_settings, instance_settings):
    #     # TODO restore intrinsic configuration, usually using:
    #     # v = instance_settings.value(k)
    #     index = instance_settings.value("irisindex",0)
    #     self._widget.IrisInputBox.setCurrentIndex(int(index))
        
    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    
