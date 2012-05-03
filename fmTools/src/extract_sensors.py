#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmTools")
import rosbag
import re 


sensors = ["/fmSensors/encoder_left",
           "/fmSensors/encoder_right",
           "/fmSensors/encoder_angle",
           "/fmSensors/gpgga",
           "/fmSensors/IMU"];

with rosbag.Bag("/home/morl/work/ASuBot/testfilterbag.bag") as input:
    print "Input bag opened"
    with rosbag.Bag("/home/morl/work/ASuBot/testbag-filtered.bag","w") as output:
        print "outputbag opened"
        for topic,msg,t in input.read_messages():
            if topic in sensors:
                output.write(topic,msg,msg.header.stamp if msg._has_header else t);
            