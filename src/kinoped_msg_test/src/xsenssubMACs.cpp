#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <ros/ros.h>
#include "kinoped_msgs/xsens_sensors.h"

void xsenssubMACsCallback(const kinoped_msgs::xsens_sensors::ConstPtr& msg)
{
  for (int i = 0; i < msg->mac_addresses.size(); i++)
  {
     ROS_INFO("Received Xsens Sensor[%d] MAC: %s   (Seq: %d)", i, msg->mac_addresses[i].c_str(), msg->header.seq);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xsenssubMACs");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("xsens_macs", 1000, xsenssubMACsCallback);
  ros::spin();

  return 0;
}
