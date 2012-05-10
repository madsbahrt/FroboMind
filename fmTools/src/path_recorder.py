#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmTools")
import rospy
from nav_msgs.msg import Odometry

import sys,select,string

points = []
record_point = False;
latest_odom =None

def waypoints_to_file(waypoints,filename):
    with open(filename,"w") as out:
        out.write("x\ty\n")
        for p in waypoints:
            out.write("%f\t%f\n"%(p.pose.pose.position.x,p.pose.pose.position.y))
        
        

def on_odometry_received(msg):
    latest_odom = msg
        
        
def on_timer(event):
    
    if select.select([sys.stdin], [], [], 0)[0] == []:
        pass
    else:
        cmd = sys.stdin.readline()
        if cmd[0] == "a":
            if len(points) > 0:
                if latest_odom.header.stamp > points[len(points)-1].header.stamp:
                    rospy.loginfo("Added %.4f %.4f to waypoints"%(latest_odom.pose.pose.position.x,latest_odom.pose.pose.position.y))
                    points.append(latest_odom)
                else:
                    rospy.loginfo("Did not record waypoint since it has not been updated")
            elif latest_odom is not None:
                rospy.loginfo("Added %.4f %.4f to waypoints"%(latest_odom.pose.pose.position.x,latest_odom.pose.pose.position.y))
                points.append(latest_odom)
            else:
                rospy.loginfo("No Odometry message received yet")
        if cmd[0] == "d":
            #delete latest point added if any
            points.pop()
        if cmd[0] == "s":
            # store waypoints
            filename = cmd[1:len(cmd)].replace(" ","")
            
            rospy.loginfo("Storing to: " + filename )
            waypoints_to_file(points,filename)
            
if __name__ == "__main__":
    
    rospy.init_node("path_recorder")
    
    rospy.Subscriber("/fmProcessors/odometry_combined",Odometry,on_odometry_received)
    rospy.timer.Timer(rospy.Duration.from_sec(0.1), on_timer)
    
    rospy.loginfo("Press \"a\" to record a waypoint \n Press \"d\" to delete the last recorded waypoint  \n Press \"s\" followed by a space and filename in order to store ")
    
    rospy.spin();
    