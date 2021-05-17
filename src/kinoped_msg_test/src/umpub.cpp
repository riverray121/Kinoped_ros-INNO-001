#include <ros/ros.h>
#include "std_msgs/String.h"
#include "kinoped_msgs/user_measurements.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "umpub");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<kinoped_msgs::user_measurements>("/user_measurements", 1000);

  ros::Rate loop_rate(1);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  kinoped_msgs::user_measurements msg;
  
  msg.height = 1.0;
  msg.weight = 2.0;
  msg.foot_length = 3.0;
  msg.foot_width = 4.0;
  msg.ground_to_ankle = 5.0;
  msg.ground_to_knee = 6.0;
  msg.ground_to_hip = 7.0;
  msg.ground_to_small_of_back = 8.0;
  msg.ground_to_shoulder = 9.0;
  msg.ground_to_c1_vertebra = 10.0;
  msg.ground_to_back_of_head = 11.0;
  msg.hip_width = 12.0;
  msg.shoulder_width = 13.0;
  msg.shoulder_to_elbow = 14.0;
  msg.shoulder_to_wrist = 15.0;
  msg.shoulder_to_middle_finger_tip = 16.0;
  msg.header.frame_id = "kinoped";


  loop_rate.sleep();
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    msg.header.stamp = ros::Time::now();
    msg.height += 0.1;
    msg.weight += 0.1;
    msg.foot_length += 0.1;
    msg.foot_width += 0.1;
    msg.ground_to_ankle += 0.1;
    msg.ground_to_knee += 0.1;
    msg.ground_to_hip += 0.1;
    msg.ground_to_small_of_back += 0.1;
    msg.ground_to_shoulder += 0.1;
    msg.ground_to_c1_vertebra += 0.1;
    msg.ground_to_back_of_head += 0.1;
    msg.hip_width += 0.1;
    msg.shoulder_width += 0.1;
    msg.shoulder_to_elbow += 0.1;
    msg.shoulder_to_wrist += 0.1;
    msg.shoulder_to_middle_finger_tip += 0.1;

    ROS_INFO("Sent user_measurements %d", count);

    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }


  return 0;
}
