
//#include <cstdio>

#include "RobotArm.h"

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"


void al5d_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {

  ROS_INFO("I heard this value: [%f]", msg->data[0]);
  
}

int main(int argc, char **argv) {

    RobotArm arm;
    
    arm.create();
    
    arm.commandRobotUsingIK(1500, 1500, 1500, 1500, 1500, 1500, 0);
    
    ROS_INFO("Init");
    //ROS_INFO("Arm get [%d]", arm.get());

    ros::init(argc, argv, "test_sub");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("al5d", 1000, al5d_callback);

    ros::spin();

    return 0;
    
}

