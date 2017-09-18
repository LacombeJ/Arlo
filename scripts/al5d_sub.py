#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

import arlo.output.al5d as al5d



arm = al5d.RobotArm()


def al5d_callback(msg):
    data = msg.data
    
    #arm.move_IK(data[0],data[1],data[2],data[3],data[4],data[5])
    arm.displace(data)

def main():

    arm.create()
    
    rospy.init_node('al5d_sub', anonymous=True)
    
    rospy.Subscriber('al5d', Float32MultiArray, al5d_callback) 
    
    rospy.spin()

    arm.destroy()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
