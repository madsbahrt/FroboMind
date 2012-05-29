#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmTools")
import rosbag
import argparse

import matplotlib.pyplot as plt

description = """
    Python script for analysing the motors and their power/voltage etc.
"""

cmd_vel_main = []
cmd_vel_left = []
cmd_vel_right = []

dist_left = []
t_dist_left = []
dist_right = []
t_dist_right = []

motor_cmd_sent_left = []
t_motor_cmd_left = []
motor_cmd_sent_right = []
t_motor_cmd_right = []
motor_amps_left = []

t_motor_amps_left = []
motor_amps_right = []
t_motor_amps_right = []

motor_voltage_left =[]
t_motor_voltage_left = []
motor_voltage_right = []
t_motor_voltage_right = []

battery_voltage_left = []
t_battery_voltage_left = []
battery_voltage_right = []
t_battery_voltage_right = []
    

def parse_bag(in_bag):
    with rosbag.Bag(in_bag) as input:
        print "Input bag opened"
        for topic,msg,t in input.read_messages():
            if topic == "/fmCSP/S0_tx" and msg.data.startswith("!G"):
                motor_cmd_sent_left.append( int(msg.data.replace("!G","")))
                t_motor_cmd_left.append(msg.header.stamp.to_sec())
                
            if topic == "/fmCSP/S1_tx" and msg.data.startswith("!G"):
                motor_cmd_sent_right.append( int(msg.data.replace("!G","")))
                t_motor_cmd_right.append(msg.header.stamp.to_sec())
                
            if topic == "/fmActuators/status_left":
                motor_amps_left.append(msg.motor_amps_in)
                motor_voltage_left.append(msg.motor_voltage_out)
                battery_voltage_left.append(msg.motor_voltage_in)
                
                t_motor_amps_left.append(msg.header.stamp.to_sec())
                t_motor_voltage_left.append(msg.header.stamp.to_sec())
                t_battery_voltage_left.append(msg.header.stamp.to_sec())
                
            
            if topic == "/fmActuators/status_right":
                motor_amps_right.append(msg.motor_amps_in)
                motor_voltage_right.append(msg.motor_voltage_out)
                battery_voltage_right.append(msg.motor_voltage_in)
                
                t_motor_amps_right.append(msg.header.stamp.to_sec())
                t_motor_voltage_right.append(msg.header.stamp.to_sec())
                t_battery_voltage_right.append(msg.header.stamp.to_sec()) 
                
            if topic == "/fmKinematics/cmd_vel_left":
                cmd_vel_left.append(msg.twist.linear.x);
                
            if topic == "/fmKinematics/cmd_vel_right":
                cmd_vel_right.append(msg.twist.linear.x);
                
            if topic == "/fmTools/cmd_vel":
                cmd_vel_main.append([msg.twist.linear.x, msg.twist.angular.z])
                
            if topic == "/fmSensors/encoder_left":
                dist_left.append(msg.encoderticks)
                t_dist_left.append(msg.header.stamp.to_sec())
            if topic == "/fmSensors/encoder_right":
                dist_right.append(msg.encoderticks)
                t_dist_right.append(msg.header.stamp.to_sec())


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(prog='extract sensors',description=description)
    parser.add_argument('-i', '--inbag')
    
    args = parser.parse_args()
    
    in_bag = args.inbag
    
    parse_bag(in_bag)
    
    
    with open("data-motor-cmd-left.txt","w") as outfile:
        outfile.write("t, motor_cmd_left")
        for t,cmd in zip(t_motor_cmd_left,motor_cmd_sent_left):
            outfile.write("%f, %f\n" % (t,cmd))
            
    with open("data-motor-cmd-right.txt","w") as outfile:
        outfile.write("t, motor_cmd_right")
        for t,cmd in zip(t_motor_cmd_right,motor_cmd_sent_right):
            outfile.write("%f, %f\n" % (t,cmd))

    with open("data-encoder_left.txt","w") as outfile:
        outfile.write("t, encoder_left")
        for t,cmd in zip(t_dist_left,dist_left):
            outfile.write("%f, %f\n" % (t,cmd))

    with open("data-encoder_right.txt","w") as outfile:
        outfile.write("t, encoder_right")
        for t,cmd in zip(t_dist_right,dist_right):
            outfile.write("%f, %f\n" % (t,cmd))

    with open("data-battery_left.txt","w") as outfile:
        outfile.write("t, battery_left")
        for t,cmd in zip(t_battery_voltage_left,battery_voltage_left):
            outfile.write("%f, %f\n" % (t,cmd))
            
    with open("data-battery_right.txt","w") as outfile:
        outfile.write("t, battery_right")
        for t,cmd in zip(t_battery_voltage_right,battery_voltage_right):
            outfile.write("%f, %f\n" % (t,cmd))
            
    with open("data-amp_left.txt","w") as outfile:
        outfile.write("t, amp_left")
        for t,cmd in zip(t_motor_amps_left,motor_amps_left):
            outfile.write("%f, %f\n" % (t,cmd))
    
    with open("data-amp_right.txt","w") as outfile:
        outfile.write("t, amp_right")
        for t,cmd in zip(t_motor_amps_right,motor_amps_right):
            outfile.write("%f, %f\n" % (t,cmd))