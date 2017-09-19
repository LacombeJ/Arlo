#!/usr/bin/env python

import arlo.utils.config as config
import arlo.input.ps4 as ps4
import arlo.output.al5d as al5d

import time

def main():
    
    pc = ps4.PS4Controller()
    
    created = pc.create()
    if not created:
        print "Error finding PS4 controller"
    
    arm = al5d.RobotArm()
    arm.create()
    
    while True:
            
        pc.poll()
        
        if pc.home():
            break
        
        
        C = [
            pc.RX(),    # BASE
            -pc.RY(),    # SHOULDER
            0,          # ELBOW
            0,    # WRIST
            0,    # WRIST_ROTATE
            0,    # GRIPPER
        ]
        
        # Trigger mod
        lt = pc.LT() + 1
        rt = pc.RT() + 1
        
        #Displacement
        C = [
            pc.RX(),                # BASE
            -pc.RY(),               # SHOULDER
            -pc.LY(),               # ELBOW
            -lt + rt,               # WRIST
            pc.LX(),                # WRIST_ROTATE
            (-pc.L1() + pc.R1()),   # GRIPPER 
        ]
        
        # Multipliers
        C[0] = 100.0 * C[0] * C[0] * C[0]
        C[1] = 100.0 * C[1] * C[1] * C[1]
        C[2] = 100.0 * C[2] * C[2] * C[2]
        C[3] = 100.0 * C[3] * C[3] * C[3]
        C[4] = 100.0 * C[4] * C[4] * C[4]
        C[5] = 100.0 * C[5] * C[5] * C[5]
        C = [int(c) for c in C]
        arm.displace(C)
        
        #time.sleep(0.1)
        
    pc.destroy()
    arm.destroy()


main()
