
//#include <cstdio>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  ROS_INFO("I heard this value: [%f]", msg->data[0]);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "test_sub");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("al5d", 1000, chatterCallback);

  ros::spin();

  return 0;
}

