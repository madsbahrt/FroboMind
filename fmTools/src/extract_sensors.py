#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmTools")
import rosbag
import argparse

description = """
    Python script for extracting the sensor data from the ASuBot
"""


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(prog='extract sensors',description=description)
    parser.add_argument('-i', '--inbag')
    parser.add_argument('-o', '--outbag')
    
    args = parser.parse_args()
    
    in_bag = args.inbag
    out_bag = args.outbag 
    
    
    
    sensors = ["/fmSensors/encoder_left",
               "/fmSensors/encoder_right",
               "/fmSensors/encoder_angle",
               "/fmSensors/gpgga",
               "/fmSensors/IMU"];
    
    with rosbag.Bag(in_bag) as input:
        print "Input bag opened"
        with rosbag.Bag(out_bag,"w") as output:
            print "outputbag opened"
            for topic,msg,t in input.read_messages():
                if topic in sensors:
                    output.write(topic,msg,msg.header.stamp if msg._has_header else t);
            