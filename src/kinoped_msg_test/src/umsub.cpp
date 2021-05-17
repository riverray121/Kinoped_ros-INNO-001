#include <ros/ros.h>
//#include "std_msgs/String.h"
#include "kinoped_msgs/user_measurements.h"

void umsubCallback(const kinoped_msgs::user_measurements::ConstPtr& msg)
{
  ROS_INFO("I heard: height %lf weight %lf", msg->height, msg->weight);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "umsub");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("user_measurements", 1000, umsubCallback);
  ros::spin();

  return 0;
}
