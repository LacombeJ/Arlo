
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

DEFAULT = [
    1500,
    1500,
    1250,
    1500,
    1350,
    1600
]

# IK movement constants
BASE_HGT    = 67.31             # base hight 2.65"
HUMERUS     = 146.05            # shoulder-to-elbow "bone" 5.75"
ULNA        = 187.325           # elbow-to-wrist "bone" 7.375"
GRIPPER_VAL = 100.00            # gripper (incl.heavy duty wrist rotate mechanism) length 3.94"
hum_sq      = HUMERUS*HUMERUS;
uln_sq      = ULNA*ULNA;

# Shoulder-elbow IK movement constants
SL      = 14.1          # Length of shoulder (cm)
EL      = 18.5          # Length of elbow (cm)
S_MID   = 1530.0        # Shoulder servo pos at 0 radians
E_MID   = 1210.0        # Elbow servo pos at 0 radians
DS      = 430.0         # Shoulder servo difference equivalent to 45 degrees (pi/4 radians)
DE      = 430.0         # Elbow servo difference equivalent to 45 degrees (pi/4 radians)
PI_2    = math.pi / 2   # PI over 2 (90 deg)
PI_4    = math.pi / 4   # PI over 4 (45 deg)
def _elastic():
    theta_max = ((E_MID - RANGES[ELBOW][MIN]) / DE) * PI_4
    alpha = theta_max + PI_2
    elastic = math.sqrt( EL**2 + SL**2 - 2*EL*SL*math.cos(alpha) )
    return elastic
ELASTIC = _elastic()

class RobotArm(object):

    # Constructor
    def __init__(self):
    
        # TTY USB file loc
        self._usb_file = '/dev/ttyUSB0'
        
        # Initialize internal position array to center position for each servo
        self._position = list(DEFAULT)
        
    
    # Attempts to initialize al5d and open the usb file
    # Returns None if succeeded or error_string if failed
    def create(self):
        
        try:
            os.stat(self._usb_file)
            exists = True
        except OSError:
            exists = False
        
        if not exists: 
            return "File {} does not exist".format(self._usb_file)
        
        self._USB = os.open(self._usb_file, os.O_RDWR | os.O_NONBLOCK | os.O_NDELAY)
        
        if(self._USB < 0):
            return "Error({}) opening '{}'".format(self._USB,self._usb_file)
        
        tty = termios.tcgetattr(self._USB)
        if tty == None:
            return 'Error getting tty attributes'
        
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
            return "Error({}) setting tty attributes".format(error)
            
        self.set_to_mid()
        
        return None
        
    
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
            
            # Write command string
            os.write(self._USB,command)
            return True
            
        return False
        
    # Displaces current position by values of the given array
    def displace(self,displacement):
        new_pos = [self._position[i] + displacement[i] for i in range(NUM_SERVOS)]
        return self.move(new_pos)
        
    # IK values x,y are given by displacement[1], displacement[2]
    def displace_IK(self,displacement):
        d = self._displace_IK(displacement)
        return self.displace(d)
        
    def move_IK(self,x,y,z,wrist_degree,wrist_rotate_degree,open_gripper):
        ik_pos = self._position_IK(x,y,z,wrist_degree,wrist_rotate_degree,open_gripper)
        return self.move(ik_pos)
        
    def center(self):
        return self.move(DEFAULT)
        
    # Sets the robot arm to its center position
    def set_to_mid(self):
        cmd = ""
        cmd += "#0P{}S200".format(DEFAULT[0])
        cmd += "#1P{}S200".format(DEFAULT[1])
        cmd += "#2P{}S200".format(DEFAULT[2])
        cmd += "#3P{}S200".format(DEFAULT[3])
        cmd += "#4P{}S200".format(DEFAULT[4])
        cmd += "#5P{}S200".format(DEFAULT[5])
        cmd += "\r"
        os.write(self._USB,cmd)
        
    # Returns copy of the positions of the arms servos
    def get_pos(self):
        return list(self._position)
        
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
        if cos < 0: cos = 0 #Added these two lines because of math error
        if cos > 1: cos = 1
        
        a2 = math.acos(cos);
        
        # Shoulder angle
        shl_angle_r = a1 + a2
        if math.isnan(shl_angle_r) or math.isinf(shl_angle_r):
            return
        shl_angle_d = shl_angle_r * 180.0 / math.pi
    
        # Elbow angle
        cos = ( hum_sq + uln_sq - s_w ) / ( 2 * HUMERUS * ULNA )
        if cos < 0: cos = 0 #Added these two lines because of math error
        if cos > 1: cos = 1
        elb_angle_r = math.acos(cos)
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
    
    
    def _displace_IK(self,C):
        # We call displace_IK twice because of 'elastic limiting'
        # Analogy: Suppose we want to go up stairs
        # ----
        #     |b   c
        #     ----
        #         |a
        #         ----
        #             |
        # Trying to go directly from point a to point b will cause us to collide with the steps
        # Instead we go from point a to point c, and then from point c to point b
        x = C[1]
        y = C[2]
        C[1] = x
        C[2] = 0
        C = self._displace_IK2(C)
        ds = C[1]
        de = C[2]
        C[1] = 0
        C[2] = y
        C = self._displace_IK2(C)
        C[1] += ds
        C[2] += de
        return C
    
    # Jonathan
    def _displace_IK2(self,C):
        
        # X,Y displacement in cm
        x = C[1]
        y = C[2]
        
        # Servo positions
        P = self.get_pos()
        
        # Arm servos
        PS = P[1] # Position of shoulder servo
        PE = P[2] # Position of elbow servo
        
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
        
        # Limit elastic band
        if dWS_targ > ELASTIC and dWS_targ > dWS:
            C[1] = 0
            C[2] = 0
            return C
        
        phi_targ = math.acos(new_wx / dWS_targ)
        
        angDWS = math.acos( alim( ( SL**2 + EL**2 - dWS_targ**2) / (2*SL*EL) ) )
        angEL = math.acos( alim ( ( dWS_targ**2 + SL**2 - EL**2) / (2*dWS_targ*SL) ) )
        
        theta_s_targ = PI_2 - phi_targ - angEL
        
        #TODO jonathan find right method to compensate for acos() mapping
        theta_e_targ = angDWS - PI_2
        
        # Find corresponding new servo pos
        PS_targ = S_MID - theta_s_targ * DS / PI_4
        PE_targ = E_MID - theta_e_targ * DE / PI_4
        
        C[1] = int(PS_targ-PS)
        C[2] = int(PE_targ-PE)

        return C


# Limit acos and asin
def alim(cos):
    if cos > 1.0:
        return 1.0
    if cos < -1.0:
        return -1.0
    return cos



    
    
