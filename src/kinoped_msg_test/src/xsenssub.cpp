#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <ros/ros.h>
//#include "std_msgs/String.h"
#include "kinoped_msgs/xsens_sensors.h"

int map[6];
const char *sorted_array[6];


// Defining comparator function as per the requirement 
static int myCompare(const void* a, const void* b) 
{ 
    // setting up rules for comparison 
    return strcmp(*(const char**)a, *(const char**)b); 
} 
  
// Function to sort the array 
void sort(const char* arr[], int n) 
{ 
    // calling qsort function to sort the array 
    // with the help of Comparator 
    qsort(arr, n, sizeof(const char*), myCompare); 
} 

void xsensSensorsCallback(const kinoped_msgs::xsens_sensors::ConstPtr& msg)
{
  ROS_INFO("I heard: Xsens Sensor First MAC[%s] (Seq: %d)", msg->mac_addresses[0].c_str(), msg->header.seq);
  int n = msg->mac_addresses.size();
  int i;
  for (i = 0; i < n; i ++)
  {
     sorted_array[i] = msg->mac_addresses[i].c_str();
  }

  // Sort the given names 
  sort(sorted_array, n); 
  
    for (i = 0; i < n; i++)
    {
       char found = 0;
       for (int j = 0; (j < n) && !found; j++)
       {
          if (!strcmp(msg->mac_addresses[i].c_str(), sorted_array[j]))
          {
             found = 1;
             int position = msg->positions[i];
             map[j] = position;
        }
       }
    }
    
        // Print the sorted names 
    printf("\nSorted and mapped array is\n"); 
    for (i = 0; i < n; i++) 
	{
        printf("%d: %s mapped Index: %d Description: ", i, sorted_array[i], map[i]); 
		switch (map[i])
		{
		   case 0:
		       printf("POSITION_LEFT_FOOT\n");
			   break;
		   case 1:
		       printf("POSITION_RIGHT_FOOT\n");
			   break;
		   case 2:
		       printf("POSITION_LEFT_KNEE\n");
			   break;
		   case 3:
		       printf("POSITION_RIGHT_KNEE\n");
			   break;
		   case 4:
		       printf("POSITION_LEFT_HIP\n");
			   break;
		   case 5:
		       printf("POSITION_RIGHT_HIP\n");
			   break;
		   case 6:
		       printf("POSITION_ALTERNATE_1\n");
			   break;
		   case 7:
		       printf("POSITION_ALTERNATE_2\n");
			   break;
		}
	}
		

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xsenssub");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("xsens_sensors", 1000, xsensSensorsCallback);
  ros::spin();

  return 0;
}
