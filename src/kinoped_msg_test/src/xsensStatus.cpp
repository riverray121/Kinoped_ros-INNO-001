#include <ros/ros.h>
#include "kinoped_msgs/kinoped_message.h"

void statusSubCallback(const kinoped_msgs::kinoped_message::ConstPtr& msg)
{
    char type[128];
	char code[128];
	char node[128];
	memset(type, 0, sizeof(type));
	memset(code, 0, sizeof(code));
	memset(node, 0, sizeof(node));
	
	switch(msg->type)
	{
	    case kinoped_msgs::kinoped_message::KINOPED_MSG_FATAL:
		    strcpy(type, "KINOPED_MSG_FATAL");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_MSG_ERROR:
		    strcpy(type, "KINOPED_MSG_ERROR");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_MSG_WARNING:
		    strcpy(type, "KINOPED_MSG_WARNING");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_MSG_INFO:
		    strcpy(type, "KINOPED_MSG_INFO");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_MSG_DEBUG:
		    strcpy(type, "KINOPED_MSG_DEBUG");
		    break;
	    default:
		    sprintf(type, "Unknown type %d", msg->type);
		    break;
	}
	switch(msg->code)
	{
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_OK :
		    strcpy(code, "KINOPED_STATUS_CODE_OK");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GENERAL_ERROR :
		    strcpy(code, "KINOPED_STATUS_CODE_GENERAL_ERROR");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_XSENS_NOT_FOUND :
		    strcpy(code, "KINOPED_STATUS_CODE_XSENS_NOT_FOUND");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_XSENS_LOW_BATTERY :
		    strcpy(code, "KINOPED_STATUS_CODE_XSENS_LOW_BATTERY");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_PLC_NOT_FOUND :
		    strcpy(code, "KINOPED_STATUS_CODE_PLC_NOT_FOUND");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_WAITING_FOR_SENSOR_MAP :
		    strcpy(code, "KINOPED_STATUS_CODE_WAITING_FOR_SENSOR_MAP");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_WAITING_FOR_START_COMMAND :
		    strcpy(code, "KINOPED_STATUS_CODE_WAITING_FOR_START_COMMAND");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_PAUSE_RECEIVED :
		    strcpy(code, "KINOPED_STATUS_CODE_PAUSE_RECEIVED");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_STOP_RECEIVED :
		    strcpy(code, "KINOPED_STATUS_CODE_STOP_RECEIVED");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_END_RECEIVED :
		    strcpy(code, "KINOPED_STATUS_CODE_END_RECEIVED");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_REBOOT_RECEIVED :
		    strcpy(code, "KINOPED_STATUS_CODE_REBOOT_RECEIVED");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GOTO_HOME_RECEIVED :
		    strcpy(code, "KINOPED_STATUS_CODE_GOTO_HOME_RECEIVED");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_GOTO_LOAD_RECEIVED :
		    strcpy(code, "KINOPED_STATUS_CODE_GOTO_LOAD_RECEIVED");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_USER_MEASUREMENTS_RECEIVED :
		    strcpy(code, "KINOPED_STATUS_CODE_USER_MEASUREMENTS_RECEIVED");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_TAG_WRITE_RECEIVED :
		    strcpy(code, "KINOPED_STATUS_CODE_TAG_WRITE_RECEIVED");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_DEBUG_ON_RECEIVED :
		    strcpy(code, "KINOPED_STATUS_CODE_DEBUG_ON_RECEIVED");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_DEBUG_OFF_RECEIVED :
		    strcpy(code, "KINOPED_STATUS_CODE_DEBUG_OFF_RECEIVED");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_RUN_RECEIVED :
		    strcpy(code, "KINOPED_STATUS_CODE_RUN_RECEIVED");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_START_RECEIVED :
		    strcpy(code, "KINOPED_STATUS_CODE_START_RECEIVED");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_RESTART_RECEIVED :
		    strcpy(code, "KINOPED_STATUS_CODE_RESTART_RECEIVED");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_WAITING_FOR_GOTO_LOAD_COMMAND :
		    strcpy(code, "KINOPED_STATUS_CODE_WAITING_FOR_GOTO_LOAD_COMMAND");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_STATUS_CODE_SHUTDOWN_RECEIVED :
		    strcpy(code, "KINOPED_STATUS_CODE_SHUTDOWN_RECEIVED");
		    break;
	    default:
		    sprintf(code, "Unknown code %d", msg->code);
		    break;
	}
	switch(msg->node)
	{
	    case kinoped_msgs::kinoped_message::KINOPED_NODE_TYPE_PLC :
		    strcpy(node, "KINOPED_NODE_TYPE_PLC");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_NODE_TYPE_XSENS :
		    strcpy(node, "KINOPED_NODE_TYPE_XSENS");
		    break;
	    case kinoped_msgs::kinoped_message::KINOPED_NODE_TYPE_WEB_UI :
		    strcpy(node, "KINOPED_NODE_TYPE_WEB_UI");
		    break;
	    default:
		    sprintf(node, "Unknown node %d", msg->node);
		    break;
	}
    ROS_INFO("I heard: Type[%s] code[%s] node[%s] description:\"%s\"", type, code, node, msg->description.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xsensStatus");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("xsens_status", 1000, statusSubCallback);
    ros::spin();
    return 0;
}
