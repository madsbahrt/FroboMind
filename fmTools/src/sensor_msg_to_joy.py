#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmTools")
import rospy
import actionlib
from sensor_msgs.msg import Joy as newJoy
from joy.msg import Joy


def on_new_joy(msg,publisher):
    out = Joy()
    out.axes= msg.axes;
    out.buttons = msg.buttons;
    publisher[0].publish(out)
    
    
    

if __name__ == "__main__":
    rospy.init_node("joy_converter")
    publisher = rospy.Publisher("/fmHMI/joy", Joy)
    s = rospy.Subscriber("/fmHMI/joy_new", newJoy, on_new_joy,callback_args=[publisher])
    
    rospy.spin();
