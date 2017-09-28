#!/usr/bin/env python

from time import sleep

import math

import arlo.input.Leap as Leap
import arlo.input.ps4 as ps4
import arlo.output.al5d as al5d

import arlo.data.leap_arm as leap_arm



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
        
        
def ps4_control():
    pc = ps4.PS4Controller()
    
    created = pc.create()
    
    if not created:
        print 'Error finding PS4 controller'
        quit()
    
    arm = al5d.RobotArm()
    arm.create()
        
    while True:
        sleep(0.04)
        
        pc.poll()
        
        if pc.home():
            break
            
        pos = arm.get_pos()
        #print pos
            
        C = control2(pc,pos)
        
        moved = arm.displace(C)
            
    arm.destroy()
        
def control1(pc,pos):
    # Trigger mod
    lt = pc.LT() + 1
    rt = pc.RT() + 1
        
    #Displacement
    C = [
        pc.RX(),                # BASE
        pc.RY(),               # SHOULDER
        pc.LY(),               # ELBOW
        -lt + rt,               # WRIST
        pc.LX(),                # WRIST_ROTATE
        (-pc.L1() + pc.R1()),   # GRIPPER 
    ]
    
    sensitivity1(C)
    
    return [int(c) for c in C]



def control2(pc,pos):
    # Trigger mod
    lt = pc.LT() + 1
    rt = pc.RT() + 1
    
    #S, E = calculate(pos,-pc.LY(),-pc.RY())
    S = -pc.LY()
    E = -pc.RY()
    
    #Displacement
    C = [
        int(pc.LX()),                # BASE
        S,               # SHOULDER
        E,               # ELBOW
        0,               # WRIST
        0,                # WRIST_ROTATE
        0,   # GRIPPER 
    ]
    
    sensitivity2(C)
    
    return [int(c) for c in C]



def sensitivity1(C):
    # Multipliers
    C[0] = 30.0 * C[0] #BASE
    C[1] = 10.0 * C[1] #SHOULDER
    C[2] = 10.0 * C[2] #ELBOW
    C[3] = 10.0 * C[3] #WRIST
    C[4] = 10.0 * C[4] #WRIST_ROTATE
    C[5] = 100.0 * C[5] #GRIPPER
    return C
    

def sensitivity2(C):
    # Multipliers
    C[0] = 30.0 * C[0] #BASE
    C[1] = C[1] #SHOULDER
    C[2] = C[2] #ELBOW
    C[3] = 10.0 * C[3] #WRIST
    C[4] = 10.0 * C[4] #WRIST_ROTATE
    C[5] = 100.0 * C[5] #GRIPPER
    return C


def calculate(C,x,y):
    
    # TODO jonathan move these constants somewhere later
    # Constants
    
    SL = 14.1 # Length of shoulder (cm)
    EL = 18.5 # Length of elbow (cm)
    S_MID = 1530.0 # Shoulder servo pos at 0 radians
    E_MID = 1210.0 # Elbow servo pos at 0 radians
    DS = 430.0 # Shoulder servo difference equivalent to 45 degrees (pi/4 radians)
    DE = 430.0 # Elbow servo difference equivalent to 45 degrees (pi/4 radians)
    PI_2 = math.pi / 2 # PI over 2 (90 deg)
    PI_4 = math.pi / 4 # PI over 4 (45 deg)
    
    # Arm servos
    PS = C[1] # Position of shoulder servo
    PE = C[2] # Position of elbow servo
    
    # Angle of shoulder (0 up, pos left, neg right)
    theta_s = ((S_MID - PS) / DS) * PI_4
    theta_e = ((E_MID - PE) / DE) * PI_4
    
    # Distance of wrist to shoulder servo (cm)
    dWS = math.sqrt( SL**2 + EL**2 - 2*SL*EL*math.cos(theta_e + PI_2) )
    #dWS = math.sqrt( (EL*math.sin(theta_e) + SL )**2 + (EL*math.cos(theta_e))**2 )
    
    # Angle between wrist and shoulder
    w = math.acos( ( EL*math.sin(theta_e) + SL ) / dWS )
    
    # Angle between wrist and up
    phi = PI_2 - theta_s - w
    
    #print dWS, phi
    
    # --------------------------------------------------
    
    # Find pos (x,y) in cm of wrist
    pos_wx = dWS * math.cos(phi)
    pos_wy = dWS * math.sin(phi)
    
    # Displace pos (cm)
    new_wx = pos_wx + x
    new_wy = pos_wy + y
    
    # --------------------------------------------------
    
    # Find appropriate angles of shoulder and elbow for the new pos
    dWS_targ = math.sqrt(new_wx**2 + new_wy**2)
    phi_targ = math.acos(new_wx / dWS_targ)
    
    angDWS = math.acos( alim( ( SL**2 + EL**2 - dWS_targ**2) / (2*SL*EL) ) )
    angEL = math.acos( alim ( ( dWS_targ**2 + SL**2 - EL**2) / (2*dWS_targ*SL) ) )
    
    theta_s_targ = PI_2 - phi_targ - angEL
    
    #TODO jonathan find right method to compensate for acos() mapping
    theta_e_targ = angDWS - PI_2
    
    # Find corresponding new servo pos
    PS_targ = S_MID - theta_s_targ * DS / PI_4
    PE_targ = E_MID - theta_e_targ * DE / PI_4
    
    return PS_targ-PS, PE_targ-PE



# Limit acos and asin
def alim(cos):
    if cos > 1.0:
        return 1.0
    if cos < -1.0:
        return -1.0
    return cos


#leap_control()
ps4_control()





