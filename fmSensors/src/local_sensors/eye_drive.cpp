/*
 * eye_drive.cpp
 *
 *  Created on: Mar 26, 2012
 *      Author: morl
 */
#include <ros/ros.h>
#include <fmMsgs/can.h>
#include <fmMsgs/claas_row_cam.h>


typedef enum
{
	INIT1 = 0x20,
	INIT2 = 0x21,
	INIT3 = 0x22,
	NO_ENTRY = 0x00

}CMDControl_t;

class CameraConfig
{
public:
	CameraConfig()
	{
		debug_setting = 0x1C;
		program = 0x03;
		height_cm = 160;
		angle_deg = 45;
		threshold = 0x80;
		target_width_cm = 0x0F;
		target_height_cm = 0x0C;
		target_distance_cm = 0x4B;
		target_minor_distance_cm = 0x00;
		number_of_rows = 0x01;
		rows_between_wheels = 0x03;
	}

	void print_config()
	{

	}

	void to_init_message(uint8_t* msg)
	{
		msg[0] = INIT1;
		msg[1] = program;
		msg[2] = NO_ENTRY;
		msg[3] = height_cm;
		msg[4] = angle_deg;
		msg[5] = threshold;
		msg[6] = debug_setting;
		msg[7] = NO_ENTRY;

		msg[8] 	= INIT2;
		msg[9] 	= target_width_cm;
		msg[10] = NO_ENTRY;
		msg[11] = target_height_cm;
		msg[12] = NO_ENTRY;
		msg[13] = target_distance_cm;
		msg[14] = target_minor_distance_cm;
		msg[15] = number_of_rows;

		msg[16] = INIT3;
		msg[17] = rows_between_wheels;
		msg[18] = msg[19] = msg[20] = msg[21] = msg[22] = msg[23] = NO_ENTRY;
	}

	int program;
	int height_cm;
	int angle_deg;
	int threshold;
	int target_width_cm;
	int target_height_cm;
	int target_distance_cm;
	int target_minor_distance_cm;
	int number_of_rows;
	int rows_between_wheels;
	int debug_setting;


};

class EyeDrive
{
public:
	EyeDrive(CameraConfig config)
	{
		this->config = config;
		timeout = 1;
		communication_timeout = 2;
		is_initialised = false;
		quality = heading = offset = 0;
		last_update = ros::Time::now();
	}

	~EyeDrive()
	{

	}

	void initCamera()
	{
		uint8_t buf[24];
		config.to_init_message(buf);

		for(int i=0;i<3;i++)
		{
			transmitInitMsg(&(buf[i*8]));
		}
	}

	void processCanRxEvent(const fmMsgs::can::ConstPtr& msg)
	{
		int temp_val;
		if(msg->id == (uint32_t)(0x142000C8 | (1 << 31)))
		{
			quality = msg->data[1];

			temp_val = msg->data[2] * 0x100 + msg->data[3];
			if (temp_val > 0x8000) /* Convert Two's complement: */
			{
			        temp_val =  temp_val - 0x10000;
			}
			temp_val =  temp_val/10;
			offset = temp_val;

			temp_val = msg->data[4] * 0x100 +msg->data[5];
			if (temp_val > 0x8000) /* Convert Two's complement: */
			{
			        temp_val =  temp_val - 0x10000;
			}
			temp_val =  temp_val/100;
			heading = temp_val;
			debug = ((msg->data[6] * 0x100 + msg->data[7])/100);

			if((msg->header.stamp - last_update).toSec() > timeout)
			{
				ROS_WARN("Timeout value exceeded");
			}

			last_update = msg->header.stamp;
		}
	}

	void processTimerEvent(const ros::TimerEvent& e)
	{
		if(!is_initialised)
		{
			initCamera();
			is_initialised = true;
			last_update = ros::Time::now();
		}
		else
		{
			ros::Time t = ros::Time::now();
			if( (t - last_update).toSec() > communication_timeout)
			{
				is_initialised = false;
				ROS_WARN("Lost Connection to eye drive, retrying");
				quality = heading = offset = 0;
			}
			cam_tx_msg.header.stamp = ros::Time::now();
			cam_tx_msg.quality = quality;
			cam_tx_msg.heading = heading;
			cam_tx_msg.offset = offset;

			cam_row_pub.publish(cam_tx_msg);
		}


	}

	ros::Publisher can_tx_pub;
	ros::Subscriber can_rx_sub;
	ros::Publisher cam_row_pub;

private:

	void transmitInitMsg(uint8_t buf[8])
	{
		can_tx_msg.header.stamp = ros::Time::now();
		can_tx_msg.id = 0x1424003C | (1 << 31); // set bit 31 to indicate EFF
		can_tx_msg.length = 8;
		for(int i=0;i<8;i++)
		{
			can_tx_msg.data[i] =buf[i];
		}

		can_tx_pub.publish(can_tx_msg);
	}

	CameraConfig config;


	fmMsgs::can can_tx_msg;
	fmMsgs::claas_row_cam cam_tx_msg;

	ros::Time last_update;

	int quality;
	int heading;
	int offset;
	int debug;

	bool is_initialised;

	double timeout;
	double communication_timeout;
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "EyeDriveNode");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	std::string can_rx_topic,can_tx_topic,row_topic;
	double publish_rate;

	ros::Timer t;

	n.param<std::string>("Can_rx_subscriber_topic",can_rx_topic,"fmBSP/can_0_rx");
	n.param<std::string>("Can_tx_publisher_topic",can_tx_topic,"fmBSP/can_0_tx");
	n.param<std::string>("row_publisher_topic",row_topic,"fmSensor/rows");

	CameraConfig conf;

	n.param<int>("cam_program",conf.program,0x03);
	n.param<int>("cam_height",conf.height_cm,160);
	n.param<int>("cam_angle",conf.angle_deg,45);
	n.param<int>("cam_threshold",conf.threshold,0x80);
	n.param<int>("cam_target_width",conf.target_width_cm,15);
	n.param<int>("cam_target_height",conf.target_height_cm,12);
	n.param<int>("cam_target_distance",conf.target_distance_cm,75);
	n.param<int>("cam_minor_distance",conf.target_minor_distance_cm,0);
	n.param<int>("cam_number_of_rows",conf.number_of_rows,1);
	n.param<int>("cam_rows_between_wheels",conf.rows_between_wheels,0x03);

	n.param<double>("publish_rate",publish_rate,25.0);


	EyeDrive camera(conf);

	camera.can_rx_sub  = nh.subscribe<fmMsgs::can> (can_rx_topic.c_str(),10,&EyeDrive::processCanRxEvent,&camera);
	camera.can_tx_pub  = nh.advertise<fmMsgs::can> (can_tx_topic.c_str(),10);
	camera.cam_row_pub = nh.advertise<fmMsgs::claas_row_cam> (row_topic.c_str(),1);

	t= nh.createTimer(ros::Duration(1.0/publish_rate),&EyeDrive::processTimerEvent,&camera);

	while(camera.can_tx_pub.getNumSubscribers() == 0)
	{
		ROS_INFO_THROTTLE(1,"Waiting for can node to subscribe");
	}

	ROS_INFO("Can has subscribed");

	camera.initCamera();

	ros::spin();

}
