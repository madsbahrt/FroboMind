/*
 * SocketCan.cpp
 *
 *  Created on: Mar 26, 2012
 *      Author: morl
 */

#include "SocketCan.h"

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/thread.hpp>


SocketCan::SocketCan() : descriptor_(ioService_)
{

}

SocketCan::~SocketCan()
{
	// TODO Auto-generated destructor stub
	close(s);
}

int SocketCan::initInterface(std::string interface)
{
	struct sockaddr_can addr;
	struct ifreq ifr;
	int ret;

	/* open socket */
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		ROS_ERROR("Cannot create CAN socket err: %d",s);
		return -1;
	}

	addr.can_family = AF_CAN;

	strcpy(ifr.ifr_name, interface.c_str());
	if ((ret = ioctl(s, SIOCGIFINDEX, &ifr)) < 0) {
		ROS_ERROR("Error setting interface name: %d",ret);
		return -1;
	}
	addr.can_ifindex = ifr.ifr_ifindex;

	if ((ret = bind(s, (struct sockaddr *)&addr, sizeof(addr))) < 0) {
		ROS_ERROR("Error Binding to socket: %d",ret);
		return -1;
	}

	descriptor_.assign(s);

	canReadSome();

    boost::thread t(boost::bind(&boost::asio::io_service::run, &ioService_));

	return 0;
}

void SocketCan::processCanTxEvent(const fmMsgs::can::ConstPtr& msg)
{
	can_frame tx;

	tx.can_dlc = msg->length;
	tx.can_id = msg->id;
	for(int i=0; i<msg->length; i++)
	{
		tx.data[i] = msg->data[i];
	}
	boost::asio::write(descriptor_, boost::asio::buffer(&tx, sizeof(tx)));
}

void SocketCan::canRxHandler(const boost::system::error_code& error, size_t bytes_transferred)
{
    can_rx_msg_.header.stamp = ros::Time::now();
    if (bytes_transferred)
    {
      can_rx_msg_.flags = 0;
      can_rx_msg_.cob = rx_.can_id;
      can_rx_msg_.id = rx_.can_id;
      can_rx_msg_.length = rx_.can_dlc;
      for (int i = 0; i <rx_.can_dlc; i++)
      {
        can_rx_msg_.data[i] = rx_.data[i];
      }
      can_rx_publisher_.publish(can_rx_msg_);
    }
    canReadSome();
}

void SocketCan::canReadSome()
{
	descriptor_.async_read_some(boost::asio::buffer(&rx_, sizeof(rx_)),
			boost::bind(&SocketCan::canRxHandler, this,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));
}


