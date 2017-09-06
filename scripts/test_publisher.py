#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

def main():

    pub = rospy.Publisher('al5d', Float32MultiArray, queue_size=1000)
    rospy.init_node('test_pub', anonymous=True)
    
    rate = rospy.Rate(60)
    
    while not rospy.is_shutdown():
    
        a = [10.0, 20.0, 30.0]
        
        rospy.loginfo(a)
        
        msg = Float32MultiArray()
        msg.data = a
        
        pub.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
