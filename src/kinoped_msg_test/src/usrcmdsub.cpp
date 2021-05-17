#include <ros/ros.h>
//#include "std_msgs/String.h"
#include "kinoped_msgs/user_command.h"

void umsubCallback(const kinoped_msgs::user_command::ConstPtr& msg)
{
  ROS_INFO("I heard: command[%s.%s.%s]", msg->command.c_str(), msg->sub_command.c_str(), msg->option.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usrcmdsub");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("user_command", 1000, umsubCallback);
  ros::spin();

  return 0;
}
