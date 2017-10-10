#!/usr/bin/env python

from time import sleep

import math

import arlo.input.Leap as Leap
import arlo.input.ps4 as ps4
import arlo.output.al5d as al5d

import arlo.data.leap_arm as leap_arm

import sys

def leap_control():
    arm = al5d.RobotArm()
    arm.create()
    
    #listener = leap_arm.LeapListener()
    listener = leap_arm.LeapIKListener()
    listener.create(arm)
    
    #listener = SampleListener()
    #listener = LeapListener(arm)
    controller = Leap.Controller()
    
    controller.add_listener(listener)
    
    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)
        arm.destroy()
        
        
leap_control()







