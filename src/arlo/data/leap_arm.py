
import math
from time import sleep

import arlo.input.Leap as Leap
import arlo.output.al5d as al5d

# Modification of Vaclav Mach's leap_to_hand.cpp
# https://github.com/lager1/leap_hand


# Dimension
X = 0
Y = 1
Z = 2


#TODO jonathan move offsets to al5d RobotArm if needed

gripper_offset = 220        # offset which equals whole range of motion for gripper
                            # divided by 5 fingers
                                       
elbow_offset = 2.2          # whole range is 1100
                            # scanned range max is roughly 530
                            # scanned range min is roughly 30

base_offset = 850           # whole range is 1950
                            # scanned range max is 1.0
                            # scanned range min is 1.0

wrist_rotate_offset = 10    # whole range is 1850
                            # scanned range max is roughly 90.0
                            # scanned range min is roughly -100.0


wrist_offset = 6.5          # whole range is 1900
                            # scanned range max is roughly 130.0
                            # scanned range min is roughly -170.0

shoulder_offset = 2         # whole range is 1000
                            # scanned range max is roughly 420
                            # scanned range min is roughly 150


class LeapListener(Leap.Listener):

    # arm al5d RobotArm
    def create(self,arm):
        self._arm = arm
        self._pos = None
    
    # ---------- Leap.Listener callbacks ---------- #
    
    def on_init(self,controller):
        self._pos = list(self._arm.get_pos())
        print "On init"
        
    def on_connect(self,controller):
        print "On connect"
        
    def on_disconnect(self,controller):
        print "On disconnect"
        
    def on_exit(self,controller):
        print "On exit"
        
    
    def on_frame(self,controller):
        sleep(0.04)
        
        frame = controller.frame()
        
        if len(frame.hands) != 1:
            return
            
        # GRIPPER
        self._pos[al5d.GRIPPER] = al5d.RANGES[al5d.GRIPPER][al5d.MAX]\
            - len(frame.fingers.extended()) * gripper_offset
        
        hands = frame.hands
        
        for hand in hands:
        
            arm = hand.arm
            direction = hand.direction
            normal = hand.palm_normal
        
            # ELBOW
            self._pos[al5d.ELBOW] = al5d.RANGES[al5d.ELBOW][al5d.MAX]\
                - int(hand.palm_position[Y] * elbow_offset)
        
            # TODO jonathan no point of doing '< 0' checks? if abs() is not called?
        
            # BASE
            if arm.direction[X] < 0:
                self._pos[al5d.BASE] = al5d.RANGES[al5d.BASE][al5d.CENTER]\
                    - int(abs(arm.direction[X]) * base_offset)
            else:
                self._pos[al5d.BASE] = al5d.RANGES[al5d.BASE][al5d.CENTER]\
                    + int(abs(arm.direction[X]) * base_offset)
            
            # WRIST ROTATE
            normal_roll = math.degrees(normal.roll)
            if normal_roll > 0:
                self._pos[al5d.WRIST_ROTATE] = al5d.RANGES[al5d.WRIST_ROTATE][al5d.CENTER]\
                    - int(abs(normal_roll) * wrist_rotate_offset)
            else:
                self._pos[al5d.WRIST_ROTATE] = al5d.RANGES[al5d.WRIST_ROTATE][al5d.CENTER]\
                    + int(abs(normal_roll) * wrist_rotate_offset)
            
            # WRIST
            direction_pitch = math.degrees(direction.pitch)
            if direction_pitch > 0:
                self._pos[al5d.WRIST] = al5d.RANGES[al5d.WRIST][al5d.CENTER]\
                    + int(abs(direction_pitch) * wrist_offset)
            else:
                self._pos[al5d.WRIST] = al5d.RANGES[al5d.WRIST][al5d.CENTER]\
                    - int(abs(direction_pitch) * wrist_offset)
            
            # SHOULDER
            self._pos[al5d.SHOULDER] = al5d.RANGES[al5d.SHOULDER][al5d.MIN]\
                + int(arm.elbow_position[Z] * shoulder_offset)
            
            
            # move robot arm
            self._arm.move(self._pos)
            
        
    def on_focus_gained(self,controller):
        print "On focus gained"
        
    def on_focus_lost(self,controller):
        print "On focus lost"
        
    def on_device_change(self,controller):
        print "On device change"
        
    def on_service_connect(self,controller):
        print "On service connect"
        
    def on_service_disconnect(self,controller):
        print "On service disconnect"

    # --------------------------------------------- #
        
        
    
    
    
    
class LeapIKListener(Leap.Listener):

    # arm al5d RobotArm
    def create(self,arm):
        self._arm = arm
        self._pos = None
    
    # ---------- Leap.Listener callbacks ---------- #
    
    def on_init(self,controller):
        self._pos = list(self._arm.get_pos())
        print "On init"
        
    def on_connect(self,controller):
        print "On connect"
        
    def on_disconnect(self,controller):
        print "On disconnect"
        
    def on_exit(self,controller):
        print "On exit"
        
    
    def on_frame(self,controller):
        sleep(0.04)
        
        frame = controller.frame()
        
        speed = 2
        
        if len(frame.hands) != 1:
            return
            
        # GRIPPER
        gripper = 0
        #TODO
        
        hands = frame.hands
        
        for hand in hands:
        
            arm = hand.arm
        
            x       = speed *  hand.palm_position[Z] + 0
            y       = speed * -hand.palm_position[X] + 250
            z       = speed *  hand.palm_position[Y] - 200
            roll    = hand.palm_normal.roll
            pitch   = hand.palm_normal.pitch
            yaw     = hand.palm_normal.yaw
            d_roll  = hand.direction.roll
            d_pitch = hand.direction.pitch
            d_yaw   = hand.direction.yaw
            
            wrist_degree = roll
            wrist_rotate_degree = d_yaw
            print x, y, z, wrist_degree, wrist_rotate_degree, gripper
            # move robot arm
            self._arm.move_IK(x,y,z,wrist_degree,wrist_rotate_degree,gripper)

        
    def on_focus_gained(self,controller):
        print "On focus gained"
        
    def on_focus_lost(self,controller):
        print "On focus lost"
        
    def on_device_change(self,controller):
        print "On device change"
        
    def on_service_connect(self,controller):
        print "On service connect"
        
    def on_service_disconnect(self,controller):
        print "On service disconnect"

    # --------------------------------------------- #
        
    
    
    
