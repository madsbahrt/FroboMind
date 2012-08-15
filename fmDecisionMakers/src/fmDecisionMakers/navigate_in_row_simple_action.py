import rcommander.tool_utils as tu

from msg import navigate_in_row_simpleAction,navigate_in_row_simpleGoal
import math

class NavigateInRowSimpleState(tu.SimpleStateBase): 

    def __init__(self, name, action_name,desired_offset,proportional_gain,distance_scale,vel,timeout):
        """
        float64 desired_offset_from_row
        float64 P
        float64 distance_scale
        float64 forward_velcoity
        uint16  headland_timeout
        """
        tu.SimpleStateBase.__init__(self, name, \
                action_name, 
                navigate_in_row_simpleAction, 
                goal_cb_str = 'ros_goal')
        self.action_name = action_name
        self.desired_offset = desired_offset
        self.P = proportional_gain
        self.distance_scale = distance_scale
        self.vel = vel
        self.headland_timeout = timeout

    def ros_goal(self, userdata, default_goal):
        return navigate_in_row_simpleGoal(desired_offset_from_row=self.desired_offset,P=self.P,distance_scale=self.distance_scale,forward_velocity=self.vel,headland_timeout=self.headland_timeout)
    
    
from PyQt4.QtGui import *
from PyQt4.QtCore import *

class NavigateInRowSimpleTool(tu.ToolBase):

    def __init__(self, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'navigate_in_row_simple', 'Navigate in row', NavigateInRowSimpleState)

    def fill_property_box(self, pbox):
        formlayout = pbox.layout()
        
        self.action_name = QLineEdit(pbox)
        self.action_name.setText("/navigate_in_row_simple")
        
        self.offset_box = QDoubleSpinBox(pbox)
        self.offset_box.setMinimum(0)
        self.offset_box.setMaximum(5)
        self.offset_box.setSingleStep(.01)
        self.offset_box.setValue(1)
        
        self.gain_box = QDoubleSpinBox(pbox)
        self.gain_box.setMinimum(0.1)
        self.gain_box.setMaximum(5)
        self.gain_box.setSingleStep(0.01)
        self.gain_box.setValue(1.2)
        
        self.dist_scale_box = QDoubleSpinBox(pbox)
        self.dist_scale_box.setMinimum(-5)
        self.dist_scale_box.setMaximum(5)
        self.dist_scale_box.setSingleStep(0.01)
        self.dist_scale_box.setValue(-0.3)
        
        self.vel_box = QDoubleSpinBox(pbox)
        self.vel_box.setMinimum(0.1)
        self.vel_box.setMaximum(5)
        self.vel_box.setSingleStep(0.01)
        self.vel_box.setValue(1.2)
        
        self.timeout_box = QDoubleSpinBox(pbox)
        self.timeout_box.setMinimum(0)
        self.timeout_box.setMaximum(50)
        self.timeout_box.setSingleStep(1)
        self.timeout_box.setValue(1)
        
        formlayout.addRow("&ActionName",self.action_name)
        formlayout.addRow("&Desired offset", self.offset_box)
        formlayout.addRow("&Proportional gain",self.gain_box)
        formlayout.addRow("&Distance scale factor",self.dist_scale_box)
        formlayout.addRow("&Velocity",self.vel_box)
        formlayout.addRow("&Headland Timeout",self.timeout_box)
        
        

    def new_node(self, name=None):
        if name == None:
            nname = self.name + str(self.counter)
        else:
            nname = name
        return NavigateInRowSimpleState(nname,str(self.action_name.text()), \
                                 self.offset_box.value(), self.gain_box.value(),self.dist_scale_box.value(),self.vel_box.value(),self.timeout_box.value())

    def set_node_properties(self, my_node):
        self.action_name.setText(my_node.action_name)
        self.gain_box.setValue(my_node.P)
        self.offset_box.setValue(my_node.desired_offset)
        self.dist_scale_box.setValue(my_node.distance_scale)
        self.vel_box.setValue(my_node.vel)
        self.timeout_box.setValue(my_node.headland_timeout)
        
    def reset(self):
        self.action_name.setText("/navigate_in_row_simple")
        self.offset_box.setValue(1)
        self.gain_box.setValue(1.2)
        self.dist_scale_box.setValue(-0.3)
        self.vel_box.setValue(1.2)
        self.timeout_box.setValue(1)
        