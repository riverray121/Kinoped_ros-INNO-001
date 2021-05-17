#include <ros/ros.h>
#include "std_msgs/String.h"
#include "kinoped_msgs/user_command.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usrcmdpub");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<kinoped_msgs::user_command>("/user_command", 1000);

  ros::Rate loop_rate(1);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  kinoped_msgs::user_command msg;

  switch (argc)
  {
      case 4:
          msg.option = argv[3];
      case 3:
          msg.command = argv[1];
          msg.sub_command = argv[2];
          break;
      default:
          printf("Usage: rosrun kinoped usrcmdpub command sub_command [option]");
          break;
  }
  msg.header.frame_id = "kinoped";

  loop_rate.sleep();
  while (count < 3)
  {
    msg.header.stamp = ros::Time::now();
    ROS_INFO("Sent user_command %s %s", msg.command.c_str(), msg.sub_command.c_str());

    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }


  return 0;
}
