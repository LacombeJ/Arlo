
'''
record.py

This module contains functions for recording data

'''

import rospy
from std_msgs.msg import Float32MultiArray
import cv2

import _recording_config as rc

import arlo.input.ps4 as ps4
import arlo.utils.config as config
import arlo.utils.log as log
import arlo.utils.term as term

# Creates and returns a logger
def create_logger(level):
    logs = {
        "debug" : {"level":0, "term":term.END},
        "info"  : {"level":1, "term":term.CYAN},
        "warn"  : {"level":2, "term":term.YELLOW},
        "error" : {"level":3, "term":term.RED}
    }
    return log.Logger(logs,level)

# Returns the config property read from a json file
def prop(key,rc_config,config_set):
    return rc_config[key], config.read(config_set[key][1])[rc_config[key]]

# Loads the config file and properties in supporting config files
def load():
    rc_config = rc.read_or_create_config()
    config_set = rc.config_set()
    user,       user_prop       = prop('user',rc_config,config_set)
    task,       task_prop       = prop('task',rc_config,config_set)
    camera,     camera_prop     = prop('camera',rc_config,config_set)
    control,    control_prop    = prop('control',rc_config,config_set)
    log_level,  log_prop        = prop('log level',rc_config,config_set)
    save_exit,  save_on_exit    = prop('save on exit',rc_config,config_set)
    ps4_config, ps4_config_prop = prop('PS4 config',rc_config,config_set)
    
    if user         == 'None' : user        = None
    if task         == 'None' : task        = None
    if camera       == 'None' : camera      = None
    if control      == 'None' : control     = None
    if log_level    == 'None' : log_level   = None
    if save_exit    == 'None' : save_exit   = None
    if ps4_config   == 'None' : ps4_config  = None
    
    config_data = {
        'user'          : user,
        'task'          : task,
        'camera'        : camera,
        'control'       : control,
        'log_prop'      : log_prop,
        'save_on_exit'  : save_on_exit,
        'ps4_config_id' : ps4_config_prop
    }
    
    return config_data

# Initializes variables

config_data = load()

user            = config_data['user']
task            = config_data['task']
camera          = config_data['camera']
control         = config_data['control']
log_prop        = config_data['log_prop']
save_on_exit    = config_data['save_on_exit']
ps4_config_id   = config_data['ps4_config_id']

logger = create_logger(log_prop)

# Recording Module Interface
class RecordingModule(object):

    def start(self):
        pass
        
    # Return False if should stop all recording
    def loop(self):
        return True
        
    # Return (bool,bool,bool)
    # exited, save_prop, save
    # exited if this module caused the exit
    # save_prop if this module should change the save property
    # save if save_prop and if this module should save or not
    def finish(self):
        False, False, False



class CameraModule(RecordingModule):
    
    def start(self):
        self._cap = cv2.VideoCapture(0)
        
        if self._cap.isOpened():
        
            if logger.level('debug'):
                self._frame_name = "Recording Window"
                
                print 'Press '+term.BOLD+term.CYAN+'ESC'+term.END+' in {} to finish.'.format(self._frame_name)
            else:
                logger.log('warn','Set logging level to debug to view recording video')
    
    def loop(self):
    
        ret, frame = self._cap.read()
        
        if ret == False:
            return False
    
        if logger.level('debug'):
            if cv2.waitKey(1) & 0xFF == 27:
                return False
                
            cv2.imshow(self._frame_name,frame)
    
        return True
        
    def finish(self):
        self._cap.release()
        cv2.destroyAllWindows()
        return False, False, False
       

class ControlModule(RecordingModule):

    def start(self):
        
        self._exited = False
        self._save_prop = False
        self._save = False
        
        self._pub = rospy.Publisher('al5d', Float32MultiArray, queue_size=10)
        rospy.init_node('al5d_pub', anonymous=True)
    
        self._rate = rospy.Rate(60)
        
        self._pc = ps4.PS4Controller(ps4_config_id)
        created = self._pc.create()
        
        if not created:
            logger.log('error','Error finding PS4 controller')
            quit()
            
        print 'Press '+term.BOLD+term.CYAN+'HOME'+term.END+' on the PS4 controller to finish.'
        print 'Press '+term.BOLD+term.PURPLE+'SQUARE'+term.END+' on the PS4 controller to save and exit.'
        print 'Press '+term.BOLD+term.RED+'CIRCLE'+term.END+' on the PS4 controller to discard and exit.'
        
    def loop(self):
        self._pc.poll()
        
        if rospy.is_shutdown():
            return False
        
        if self._pc.home():
            return False
            
        if self._pc.square():
            self._exited = True
            self._save_prop = True
            self._save = True
            return False
            
        if self._pc.circle():
            self._exited = True
            self._save_prop = True
            self._save = False
            return False
        
        C = [
            -self._pc.RX(), # X
            self._pc.RY(),  # Y
            0,              # Z
            self._pc.LY(),  # wrist_degree
            self._pc.LT(),  # wrise_rotate_degree
            self._pc.RT(),  # open
            0               # reward
        ]
        
        msg = Float32MultiArray()
        msg.data = C
        
        self._pub.publish(msg)
        
        self._rate.sleep()
         
        return True
            
        
    def finish(self):
        return self._exited, self._save_prop, self._save

camera_module = CameraModule()
control_module = ControlModule()

# RECORD
def record():

    # Add recording modules
    modules = []
    if camera==None:
        logger.log('warn','Camera setting is None, no video will be recorded')
    else:
        modules.append(camera_module)
    if control == None:
        logger.log('warn','Control setting is None, no controls will be recorded')
    else:
        modules.append(control_module)
    
    # Start recording
    print 'Starting to record...'
    print 'Press '+term.BOLD+term.RED+'CTRL+C'+term.END+' in terminal to stop and discard video.'
    
    for module in modules:
        module.start()
        
    try:
        loop = True
        while loop:
            for module in modules:
                mod_loop = module.loop()
                if not mod_loop:
                    loop = False
                    break
    except KeyboardInterrupt:
        print ""
        quit()
    
    print 'Recording finished.'
    
    b_save_prop = False
    b_save = False
        
    for module in modules:
        m_exited, m_save_prop, m_save = module.finish()
        if m_exited:
            if m_save_prop:
                b_save_prop = True
                b_save = m_save
    
    if not b_save_prop:
        # Ask user if should save recording
        save = True
        if not save_on_exit:
            save_input = raw_input(term.BLUE+'Do you wish to save the recording? '+term.END+'[Y/n] ')
            if save_input=='y' or save_input=='Y' or save_input=='':
                save = True
            else:
                save = False
    else:
        save = b_save
            
    # Save recording
    if save:
        print 'Recording saved.'
    else:
        print 'Recording discarded.'


# CONFIG
def config():
    pass

# PLAYBACK
def playback():
    pass





 


