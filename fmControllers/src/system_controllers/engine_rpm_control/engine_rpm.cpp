/** \file engine_rpm.cpp
 *  \ingroup fmControllers
 *  \brief Receives "change rpm state" from the Wiimote and sends the new state to the actuators
 *
 * Initially designed for:
 * AMS - The Hako: Official webpage: https://mpt.uni-hohenheim.de/ams
 * AMS - The Hako: Documentation webpage: http://mpt-internal.uni-hohenheim.de/doku.php?id=robots:hako:welcome
 *
 * Button assignment for the Wiimote is located at http://www.ros.org/doc/api/wiimote/html/msg/State.html
 *
 *	\todo Add rev and date to git-format
 *
 *  \author Claes Jæger-Hansen
 *  $Rev$
 *  $Date$
 */

#include <stdio.h>
#include <string.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/ByteMultiArray.h"
#include "fmMsgs/engine_rpm.h"
#include "boost/tokenizer.hpp"
#include "boost/lexical_cast.hpp"
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/Joy.h>

ros::Publisher engine_rpm_pub;		//<! Publihser for the engine_rpm message
fmMsgs::engine_rpm engine_rpm_msg;	//<! Engine RPM message from fmMsgs


#define MSG_BTN_PLUS 4 				//<! Plus button assignment on Wiimote
#define MSG_BTN_MINUS 5 			//<! Minus button assignment on Wiimote

#define IDLE_RPM_STATE 1 			//<! Define the state "idle" for the statehandler
#define LOW_RPM_STATE 2				//<! Define the state "low" for the statehandler
#define MEDIUM_RPM_STATE 3			//<! Define the state "medium" for the statehandler
#define HIGH_RPM_STATE 4			//<! Define the state "high" for the statehandler

int rpm_state_handler = 1;			//<! State handler which contains the current rpm state from the Wiimote
int idle_rpm = 0;					//<! Used to store the ilde rpm value defined in the launch-file
int low_rpm = 0;					//<! Used to store the low rpm value defined in the launch-file
int medium_rpm = 0;					//<! Used to store the medium rpm value defined in the launch-file
int high_rpm = 0;					//<! Used to store the high rpm value defined in the launch-file

bool pressedPlus = false;
bool pressedMinus = false;

/*
 * \breif Set the rpm value in the message and publish it.
 *
 * Read the rpm_state_handler and assign the correct rpm value to the message.
 * When the value is assigned it is published.
 *
 * The function is called by engine_rpm_CallbackString(), each time a message is received from the Wiimote.
 *
 */
void engine_rpm_out(){
	switch(rpm_state_handler){
		case IDLE_RPM_STATE:
			engine_rpm_msg.rpm = idle_rpm;
			break;
		case LOW_RPM_STATE:
			engine_rpm_msg.rpm = low_rpm;
		break;
		case MEDIUM_RPM_STATE:
			engine_rpm_msg.rpm = medium_rpm;
		break;
		case HIGH_RPM_STATE:
			engine_rpm_msg.rpm = high_rpm;
		break;
		default:
			engine_rpm_msg.rpm = idle_rpm;
		break;
	}
	engine_rpm_pub.publish(engine_rpm_msg);
}

/*
 * \breif Handles the input from the Wiimote and assigns the correct state.
 *
 * Each time a Wiimote message is published this function is called. It only looks at plus and minus buttons on the Wiimote.
 * When the plus or minus button is pressed, the state handler is updated and the function engine_rpm_out() is
 * called to publish the new value.
 *
 * \param msg Message from the Wiimote.
 */

void engine_rpm_CallbackString(const sensor_msgs::Joy::ConstPtr& msg){

	if(msg->buttons[MSG_BTN_PLUS] == 1){
		if(!pressedPlus){
			if(rpm_state_handler < HIGH_RPM_STATE){
				rpm_state_handler++;
			}
			else{
				rpm_state_handler = HIGH_RPM_STATE;
			}
			pressedPlus = true;
			engine_rpm_out();
		}
	}
	else if(msg->buttons[MSG_BTN_PLUS] == 0){
		if(pressedPlus){pressedPlus = false;}
	}

	if(msg->buttons[MSG_BTN_MINUS] == 1){
		if(!pressedMinus){
			if(rpm_state_handler > IDLE_RPM_STATE){
				rpm_state_handler--;
			}
			else{
				rpm_state_handler = IDLE_RPM_STATE;
			}
			pressedMinus = true;
			engine_rpm_out();
		}
	}
	else if(msg->buttons[MSG_BTN_MINUS] == 0){
			if(pressedMinus){pressedMinus = false;}
	}
}

/*
 * \brief Read parameters from the launch-files and create publishers and subscribers.
 *
 *
 */
int main(int argc, char **argv){

	ros::init(argc, argv, "engine_rpm_control");	//Initialze the node
	ros::NodeHandle nh;								//Create the node handler
	ros::NodeHandle n("~");							//Create the node handler in the local namespace

	std::string subscribe_topic_id;
	std::string publish_topic_id;

	n.param<std::string>("subscribe_topic_id", subscribe_topic_id,"/fmHMI/joy");				//Read the id, from the launch-file to subscribe to
	//n.param<std::string>("subscribe_topic_id", subscribe_topic_id,"wiimote/State");			//Read the id, from the launch-file to subscribe to - corrected to above
	n.param<std::string>("publish_topic_id", publish_topic_id,"fmControllers/engine_rpm_msg");	//Read the id, from the launch-file to publish to
	engine_rpm_pub = n.advertise<fmMsgs::engine_rpm>(publish_topic_id, 1);						//Create pulisher for the engine_rpm msg

	//Read the rpm valus from the launch file and store them in the node.
	n.param<int>("idle_rpm",idle_rpm,1000);
	n.param<int>("low_rpm",low_rpm,1300);
	n.param<int>("medium_rpm",medium_rpm,1700);
	n.param<int>("high_rpm",high_rpm,2000);

	ros::Subscriber sub = n.subscribe(subscribe_topic_id, 10, engine_rpm_CallbackString);	//Subscribe to Wiimote messages

	ros::spin();
	return 0;
}

