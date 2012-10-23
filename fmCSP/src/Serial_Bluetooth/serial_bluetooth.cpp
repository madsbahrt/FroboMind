#include <csignal>
#include <cstdio>
#include "ros/ros.h"
#include "Bluetooth.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "can_node");

  ros::NodeHandle n("~");
  ros::NodeHandle nh;
  int term;

  std::string device,publisher_topic,subscriber_topic,addr;

  n.param<std::string>("device", device, "can0");
  n.param<std::string>("publisher_topic", publisher_topic, "can_rx");
  n.param<std::string>("subscriber_topic", subscriber_topic, "can_tx");
  n.param<std::string>("bluetooth_address",addr,"00:06:66:04:9E:1E");
  n.param<int> ("termination_character", term,10);
  BluetoothSerial can;

  can.term_char = (char) term;



  if(can.initInterface(device)!=0)
  {
	  ROS_ERROR("Could not open device: %s",device.c_str());
	  return -1;
  }
  else
  {
	  can.serial_rx_publisher_ = nh.advertise<fmMsgs::serial>(publisher_topic.c_str(),1);
	  can.serial_tx_subscriber_ = nh.subscribe(subscriber_topic.c_str(),10,&BluetoothSerial::processSerialTxEvent,&can);
	  ros::spin();
  }

  return 0;
}

