#include <ros/ros.h>
#include "std_msgs/String.h"
#include "kinoped_msgs/xsens_sensors.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xsenspub");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<kinoped_msgs::xsens_sensors>("/xsens_sensors", 1000);

  ros::Rate loop_rate(1);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
       int indices[6]  = { 2, 4, 0, 5, 1, 3 };
    // Get the array of names to be sorted 
    const char* arr[]  = { "00B46F51", "00B46EDF", "00B46F59", "00B46F5F", "00B46F5C", "00B46EE2" }; 
    const char* desc[] = {"POSITION_LEFT_KNEE", "POSITION_LEFT_HIP", "POSITION_LEFT_FOOT", "POSITION_RIGHT_HIP", "POSITION_RIGHT_FOOT", "POSITION_RIGHT_KNEE"}; 

  int count = 0;
  kinoped_msgs::xsens_sensors msg;
  
  msg.header.frame_id = "xsens_sensors";
  for (int i = 0; i < 6; i++)
  {
     msg.mac_addresses.push_back(arr[i]);
     msg.positions.push_back(indices[i]);
     msg.descriptions.push_back(desc[i]);
  }


  loop_rate.sleep();
  while (ros::ok())
  {
    msg.header.stamp = ros::Time::now();
    ROS_INFO("Sent xsens_message %d", count);

    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }


  return 0;
}
