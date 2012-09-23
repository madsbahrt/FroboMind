/*
 * EXSInterface.cpp
 *
 *  Created on: Sep 23, 2012
 *      Author: morl
 */

#include "EXSInterface.h"

EXSInterface::EXSInterface()
{
	// TODO Auto-generated constructor stub
	steering_angle_updated = false;
	cmd_vel_updated = false;
	ok = false;


	engine_speed = 50;
	steering_angle_rad = 0;
	steering_angle_to_int = 20;
	cmd_vel_ms = 0;
	cmd_vel_to_int = 20;

}

EXSInterface::~EXSInterface()
{
	// TODO Auto-generated destructor stub
}

void EXSInterface::onCANMsg(const fmMsgs::can::ConstPtr& msg)
{
	switch (msg->id) {
		case can_id_rx_t::CAN_ID_HAKO_IO:

			break;
		case can_id_rx_t::CAN_ID_HAKOSTATE:

			break;
		case can_id_rx_t::CAN_ID_GEAR_AND_POWER:

			break;
		case can_id_rx_t::CAN_ID_HORN_ACK:
			ROS_DEBUG_NAMED("HORN","Received ack msg");
			break;
		case can_id_rx_t::CAN_ID_CVT_ACK:
			break;
		case can_id_rx_t::CAN_ID_ODOM:
			// XXX: unpack info from can msg
			enc_msg.encoderticks = (msg->data[0] | (msg->data[1] << 8) | (msg->data[2] << 16) | (msg->data[3] << 24));
			enc_msg.header.stamp = msg->header.stamp;
			encoder_pub.publish(enc_msg);
			break;
		case can_id_rx_t::CAN_ID_STEERING_ACK:
			break;
		case can_id_rx_t::CAN_ID_STEERING_ANGLE_ACK:
			break;
		case can_id_rx_t::CAN_ID_STEERING_REPORT:
			break;
		case can_id_rx_t::CAN_ID_SWITCH_ACK:
			break;
		default:
			ROS_DEBUG_NAMED("UnknownCan","Received unknown frame with id: %d",msg->id);
			break;
	}
}

void EXSInterface::onTimer(const ros::TimerEvent& e)
{
	if(steering_angle_updated || cmd_vel_updated)
	{
		steering_angle_updated = cmd_vel_updated = false;

		can_msg.id = can_id_tx.CAN_ID_STEERING_ANGLE_CMD;
		can_msg.length = 8;

		// write steering angle
		int16_t tmp = (int16_t)(steering_angle_rad*steering_angle_to_int);
		can_msg.data[0] = tmp;
		can_msg.data[1] = tmp >> 8;

		// write velocity command
		tmp = cmd_vel_ms * cmd_vel_to_int;
		can_msg.data[2] = tmp;
		can_msg.data[3] = tmp >> 8;

		// wirte engine speed
		can_msg.data[4] = engine_speed;
		can_msg.data[6] = 'A';

		can_tx_pub.publish(can_msg);
	}



}

void EXSInterface::onSteeringAngle(
		const fmMsgs::steering_angle_cmd::ConstPtr& msg)
{
	steering_angle_updated = true;
	steering_angle_rad = msg->steering_angle;

}

void EXSInterface::onCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
{
	cmd_vel_updated = true;
	cmd_vel_ms = msg->linear.x;
}
