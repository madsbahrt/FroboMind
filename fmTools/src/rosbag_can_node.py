#!/usr/bin/env python

import roslib; 
roslib.load_manifest("fmTools")
import rospy
import subprocess

from fmMsgs.msg import can

logging_active = False


def start_bag(topics,dir,name):
    prog = None
    topics_as_arg =  " ".join(topics)
    command = "rosbag record " + topics_as_arg + " -o " + name
    try:
        prog = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=dir)
        rospy.loginfo("started bag with pid: %d" % prog.pid)
    except (subprocess.CalledProcessError,OSError),msg:
        print msg
    return prog
    
def stop_bag(prog):
    prog.send_signal(2)
    rospy.loginfo("bag stopped")
    
def on_new_can(msg):
    if msg.id == 7:
        global logging_active
        rospy.loginfo("canmsg received")   
        print ord(msg.data[6])
        if ord(msg.data[6]) == 1:
            rospy.loginfo("starting")
            logging_active = True
        else:
            rospy.loginfo("stopping")
            logging_active = False
        
    
if __name__ == "__main__":
    
    rospy.init_node("rosbag_recorder")
    cantopic = rospy.get_param("~can_rx","can0_rx")
    topics = rospy.get_param("~topics",None)
    name = rospy.get_param("~log_name","auto")
    retry = rospy.get_param("~retry",False)
    dir = rospy.get_param("~log_dir","/media/bagdrive/")
    rospy.loginfo("Running log with name: %s with topics: %s" % (topics,name))
    rospy.Subscriber(cantopic, can, on_new_can)
    
    started = False
    
    rate = rospy.Rate(5)
    
    p = None
    
    while not rospy.is_shutdown():
        
        if logging_active and not started:
            p = start_bag(topics, dir, name)
            if p:
                started = True
            else:
                if not retry:
                    exit(1)
                    
            
        if started and not logging_active:
            stop_bag(p)
            started = False
        
        if started:    
            if p.poll():
                rospy.logerror("rosbag has died with code %d" % prog.returncode)
                if retry:
                    started = False
                    rospy.logwarn("retrying")
                else:
                    exit(1)
        
        rate.sleep()
        
    if p:
        stop_bag(p)
        
        