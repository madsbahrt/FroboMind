#!/usr/bin/env python

import roslib; 
roslib.load_manifest("fmTools")
import rosbag
import argparse
import subprocess, yaml



outfiles = {}

class TopicExtractor():
    
    def __init__(self,fields,output="",prefix=""):
        self.fields = fields
        self.output = output
        self.prefix = prefix
        self.data = []
    
    def add_msg(self,msg):
        # find out how to dynamically access entry
        tmp = []
        for f in self.fields:
            r = msg
            for t in f:
                r = getattr(r,t) 
            tmp.append(r)
        self.data.append(tmp)

topics = {"/fmActuators/status_left":TopicExtractor([["header","stamp"],["motor_amps_in"], ["motor_voltage_in"]],prefix="left",output="motor_status"),
          "/fmActuators/status_right":TopicExtractor([["header","stamp"],["motor_amps_in"], ["motor_voltage_in"]],prefix="right",output="motor_status")}

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(prog='extract sensors',description=""" extracts and collates data from bags into csv files """)
    parser.add_argument('-i', '--inbag')
    
    args = parser.parse_args()
    
    in_bag = args.inbag
    

    info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', in_bag], stdout=subprocess.PIPE).communicate()[0])
    
    with rosbag.Bag(in_bag) as input:
        
        for topic,msg,time in input.read_messages(topics=[k for k in topics]):
            if topic in topics:
                topics[topic].add_msg(msg)            
    
    # write to file
    
    for outputname in topics:
        entries = topics[outputname]
        with open(outputname.replace('/','-'),"w") as out:
            fieldstr = str.join("",[entries.prefix + str.join("",s) + " " for s in entries.fields])
            out.write(fieldstr + "\n")
            for data in entries.data:
                out.write(str.join("",[str(e) + " " for e in data]) + "\n")
                
