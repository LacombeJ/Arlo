
import os
import termios
import math

# Modification of Vaclav Mach's leap_to_hand.cpp
# https://github.com/lager1/leap_hand

# http://man7.org/linux/man-pages/man2/open.2.html
# https://docs.python.org/2/library/os.html
# https://docs.python.org/2/library/termios.html
# http://pubs.opengroup.org/onlinepubs/7908799/xsh/termios.h.html

# Call python >> dir(termios) for termios constants and functions


# TTY attrib array indices from tcgetattr
IFLAG   = 0
OFLAG   = 1
CFLAG   = 2
LFLAG   = 3
ISPEED  = 4
OSPEED  = 5
CC      = 6


# Dimension
X = 0
Y = 1
Z = 2


# Range
MIN         = 0
CENTER      = 1
MAX         = 2
RANGE_SIZE  = 3 # Number of ranges


# Servos
BASE            = 0
SHOULDER        = 1
ELBOW           = 2
WRIST           = 3
WRIST_ROTATE    = 4
GRIPPER         = 5
NUM_SERVOS      = 6 # Number of servos


# Ranges for each servo
RANGES = [
    #MIN    CENTER  MAX
    [600,   1500,   2400], # Base
    [600,   1500,   2200], # Shoulder
    [600,   1250,   2200], # Elbow
    [600,   1500,   2400], # Wrist
    [600,   1350,   2400], # Wrist rotate
    [600,   1600,   2400], # Gripper
]

# IK movement constants
BASE_HGT    = 67.31             # base hight 2.65"
HUMERUS     = 146.05            # shoulder-to-elbow "bone" 5.75"
ULNA        = 187.325           # elbow-to-wrist "bone" 7.375"
GRIPPER_VAL = 100.00            # gripper (incl.heavy duty wrist rotate mechanism) length 3.94"
hum_sq      = HUMERUS*HUMERUS;
uln_sq      = ULNA*ULNA;


class RobotArm(object):

    # Constructor
    def __init__(self):
    
        # TTY USB file loc
        self._usb_file = '/dev/ttyUSB0'
        
        # Initialize internal position array to center position for each servo
        self._position = [
            1500,
            1500,
            1250,
            1500,
            1350,
            1600
        ]
        
    # Attempts to initialize al5d and open the usb file
    # Returns false if failed
    def create(self):
        if not self._initialize():
            print "Failed to initialize usb connection to robotic hand"
            return False
        return True
        
    # Closes the USB file
    def destroy(self):
        os.close(self._USB)
        
    # Move to the new pos defined by the given array if move is within ranges
    def move(self,new_pos):
        command = ""
        for i in range(NUM_SERVOS):
            if self._check_move_servo(i,new_pos[i]):
                command += "#{}P{}".format(i,self._position[i])
                
        if len(command) > 0:
            # Append end
            command += "T400\r" # time(ms) to complete, was on 800 using c++ RobotArm class
            print new_pos
            
            # Write command string
            os.write(self._USB,command)
        
    # Displaces current position by values of the given array
    def displace(self,displacement):
        new_pos = [self._position[i] + displacement[i] for i in range(NUM_SERVOS)]
        self.move(new_pos)
        
    def move_IK(self,x,y,z,wrist_degree,wrist_rotate_degree,open_gripper):
        ik_pos = self._position_IK(x,y,z,wrist_degree,wrist_rotate_degree,open_gripper)
        self.move(ik_pos)
        
    # Sets the robot arm to its center position
    def set_to_mid(self):
        cmd = "#0P1500S200#1P1600S200#2P1400S200#3P1500S200#4P1450S200#5P1400S250\r"
        os.write(self._USB,cmd)
        
    # Checks if the servo move is valid and if it is, update internal position
    def _check_move_servo(self,i,pos):
    
        if pos > RANGES[i][MAX]:
            pos = RANGES[i][MAX]
        if pos < RANGES[i][MIN]:
            pos = RANGES[i][MIN]
    
        if self._position[i] != pos:
        
            # Safe move
            self._position[i] = pos
            return True
        
        return False # No need to move
        
    # Opens the tty USB file and sets the Robot to it's mid position
    def _initialize(self):
    
        #TODO jonathan remove sync parameter if OSError Errno 11 still occurs on "constant" write
        #     or if synchronization is slow
        #sync = os.O_SYNC # flushes all write data
        sync = os.O_DSYNC # flushes write data needed for a read
        self._USB = os.open(self._usb_file, os.O_RDWR | os.O_NONBLOCK | os.O_NDELAY | sync)
        
        if(self._USB < 0):
            print "Error({}) opening '{}'".format(self._USB,self._usb_file)
        
        tty = termios.tcgetattr(self._USB)
        if tty == None:
            print 'Error getting tty attributes'
        
        self._memset_tty(tty)
        
        # Set Baud rate
        tty[OSPEED] = termios.B9600
        tty[ISPEED] = termios.B9600
        
        # Set TTY flags
        tty[CFLAG]              &= ~termios.PARENB  # make 8n1
        tty[CFLAG]              &= ~termios.CSTOPB
        tty[CFLAG]              &= ~termios.CSIZE
        tty[CFLAG]              |=  termios.CS8
        tty[CFLAG]              &= ~termios.CRTSCTS # no flow control
        tty[LFLAG]              =   0               # no signal chars, no echo, no canonical processing
        tty[OFLAG]              =   0               # no remapping, no delays
        tty[CC][termios.VMIN]   =   0               # read doesn't block
        tty[CC][termios.VTIME]  =   5               # 0.5 seconds read timeout
        
        tty[CFLAG] |= termios.CREAD | termios.CLOCAL # turn on READ & ignore ctrl lines
        tty[IFLAG] &= ~(termios.IXON | termios.IXOFF | termios.IXANY) # turn off s/w flow ctrl
        tty[LFLAG] &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG) # make raw
        tty[OFLAG] &= ~termios.OPOST #makr raw
        
        # Flush port
        termios.tcflush(self._USB, termios.TCIFLUSH)
        
        # Apply attributes, check for error
        error = termios.tcsetattr(self._USB, termios.TCSANOW, tty)
        
        if error != None:
            print "Error({}) setting tty attributes".format(error)
            return False
            
        self.set_to_mid()
        
        return True
    
    # Sets the first 6 values of the tty attrib array to 0
    def _memset_tty(self,tty):
        for i in range(6):
            tty[i] = 0
            
    # Move robot arm using IK commands
    def _position_IK(self,x,y,z,wrist_degree,wrist_rotate_degree,open_gripper):
    
        # grip angle in radians for use in calculations
        grip_angle_d = 0 #TODO jonathan why is this 0? can we simplify?
        grip_angle_r = grip_angle_d * math.pi / 180.0
        
        # Base angle and radian distance from x,y coordinates
        bas_angle_r = math.atan2(x,y)
    
        # rdist i y coordinate for the arm
        rdist = math.sqrt(x*x + y*y)
        y = rdist
        
        # Grip offsets calculated based on grip angle
        grip_off_z = math.sin(grip_angle_r) * GRIPPER_VAL
        grip_off_y = math.cos(grip_angle_r) * GRIPPER_VAL

        # Wrist position
        wrist_z = (z - grip_off_z) - BASE_HGT
        wrist_y = y - grip_off_y
        
        # Shoulder to wrist distance (sw)
        s_w = (wrist_z * wrist_z) + (wrist_y * wrist_y)
        s_w_sqrt = math.sqrt(s_w)
        
        # s_w angle to ground
        a1 = math.atan2(wrist_z, wrist_y)
        
        # s_w angle to humerus
        cos = (( hum_sq - uln_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt )
        a2 = math.acos(cos)
        
        # Shoulder angle
        shl_angle_r = a1 + a2
        if math.isnan(shl_angle_r) or math.isinf(shl_angle_r):
            return
        shl_angle_d = shl_angle_r * 180.0 / math.pi
    
        # Elbow angle
        elb_angle_r = math.acos(( hum_sq + uln_sq - s_w ) / ( 2 * HUMERUS * ULNA ))
        elb_angle_d = elb_angle_r * 180.0 / math.pi
        elb_angle_dn = -( 180.0 - elb_angle_d )
        
        # Wrist angle
        wri_angle_d = ( grip_angle_d - elb_angle_dn ) - shl_angle_d
        
        ik_pos = [
            RANGES[BASE][CENTER]            - int((( bas_angle_r * 180.0 / math.pi) * 11.11 )),
            RANGES[SHOULDER][CENTER]        + int((( shl_angle_d - 90.0 ) * 6.6 )),
            RANGES[ELBOW][CENTER]           - int((( elb_angle_d - 90.0 ) * 6.6 )),
            RANGES[WRIST][CENTER]           + int(( wri_angle_d  * 11.1 / 2 ) - (wrist_degree) * 1000 - 600),
            RANGES[WRIST_ROTATE][CENTER]    + int((( bas_angle_r * 180.0 / math.pi) * 11.11 ) + wrist_rotate_degree * 1200),
            RANGES[GRIPPER][MAX]            - int( open_gripper * 1400)
        ]

        return ik_pos






    
    
