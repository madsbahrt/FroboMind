#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmTools")
import rospy
import actionlib
from nav_msgs.msg import Odometry
from fmMsgs.msg import motor_status


avg_power = 0;
avg_current = 0;
avg_voltage = 0;

def on_msg(msg,publisher):
    
    


if __name__ == "__main__":
    rospy.init_node("motorviz")
    
    p = rospy.Publisher("/fmTools/motor_status_left", motor_status)
    rospy.Subscriber("/fmActuators/status_left",motor_status,callback=on_msg,callback_args=[p])
    