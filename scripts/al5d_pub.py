#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

import arlo.utils.config as config
import arlo.input.ps4 as ps4

def main():

    pub = rospy.Publisher('al5d', Float32MultiArray, queue_size=10)
    rospy.init_node('al5d_pub', anonymous=True)
    
    rate = rospy.Rate(60)
    
    pc = ps4.PS4Controller()
    created = pc.create()
    if not created:
        print "Error finding PS4 controller"
    
    read = False
    write = False
    
    fname = "temp_ald5.json"
    base = "c"
    data = {}
    count = 0
    num = 0
    if read:
        data = config.read(fname)
        num = data['num']
    
    while not rospy.is_shutdown():
            
        pc.poll()
        
        if pc.home():
            break
        
        if read:
            if count == num - 1:
                break
            C = data[base + str(count)]
        else:
            # x
            # y
            # z
            # wrist_degree
            # wrist_rotate_degree
            # open
            # reward
            C = [
                -pc.RX(),    # X
                pc.RY(),    # Y
                0,          # Z
                pc.LY(),    # wrist_degree
                pc.LT(),    # wrise_rotate_degree
                pc.RT(),    # open
                0           # reward
            ]
            data[base + str(count)] = C
        
        
        
        rospy.loginfo(C)
        
        msg = Float32MultiArray()
        msg.data = C
        
        pub.publish(msg)
        
        count = count + 1
        
        rate.sleep()
        
    pc.destroy()
        
    if write:
        data['num'] = count
        print "Writing to: {}".format(fname)
        config.write(fname,data)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
