
//#include <cstdio>

#include "RobotArm.h"

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

RobotArm arm;

void al5d_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {

    //ROS_INFO("I heard this value: [%f]", msg->data[0]);
    arm.commandRobotUsingIK(
        msg->data[0],
        msg->data[1],
        msg->data[2],
        msg->data[3],
        msg->data[4],
        msg->data[5],
        msg->data[6]);
        
}

int main(int argc, char **argv) {
    
    arm.create();
    
    //arm.commandRobotUsingIK(1500, 1500, 1500, 1500, 1500, 1500, 0);
    
    ROS_INFO("Init");
    //ROS_INFO("Arm get [%d]", arm.get());

    ros::init(argc, argv, "test_sub");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("al5d", 10, al5d_callback);

    ros::spin();

    return 0;
    
}

