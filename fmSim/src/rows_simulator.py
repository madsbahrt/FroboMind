import roslib
roslib.load_manifest("fmSim")

import rospy
from tf import TransformListener
import numpy.random as rand

n_rows = 3
avg_distance = 1
sigma = 0.5
row_length = 20
offset_x = 10
offset_y = 15

lines = []

def distance_to_line(x0,y0,x1,y1,x2,y2):
    line_length_sq = pow(x1-x0,2) + pow(y2-y1,2)
    u = (x2-x0)*(x1-x0)+(y2-y0)*(y1-y0) / (line_length_sq)
    px = x0 + u*(x1-x0)
    py = y0 + u*(y1-y0)
    
    return sqrt(pow(px - x2,2) + (py - y2)) 
     
    
def calculate_distance_to_rows():
    tflisten = TransformListener()
    dist = []
    for i in range(0,n_rows):
        (veh_trans,veh_rot) = tflisten.lookupTransform("odom","base_footprint",rospy.Time(0))
        
        
    pass

def spin(event):
    pass
    


if __name__ == "__main__":
    
    rospy.init_node("row_simulator")
    
    rospy.get_param("~n_rows",n_rows)
    rospy.get_param("~row_length",row_length)
    rospy.get_param("~sigma",sigma)
    rospy.get_param("~avg_distance",avg_distance)
    rospy.get_param("~offset_x",offset_x)
    rospy.get_param("~offset_y",offset_y)
    
    for i in range(0,n_rows):
        x0 = rand.normal()*sigma + offset_x + i*avg_distance*rand.normal()
        y0 = rand.normal()*sigma + offset_y
        
        x1 = x0
        y1 = rand.normal() * sigma + offset_y + row_length
        
        
        line.append((x0,y0,x1,y1))