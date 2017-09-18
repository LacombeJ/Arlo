
'''
record.py

This module contains functions for recording data

'''

import rospy
from std_msgs.msg import Float32MultiArray
import cv2

import node

import _recording_config as rc

import arlo.input.ps4 as ps4
import arlo.output.al5d as al5d #TODO jonathan remove using ROS messages

import arlo.utils.config as config
import arlo.utils.ext as ext
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
    recording_path = rc.recording_path()
    
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
        'recording_path'    : recording_path,
        'user'              : user,
        'task'              : task,
        'camera'            : camera,
        'control'           : control,
        'log_prop'          : log_prop,
        'save_on_exit'      : save_on_exit,
        'ps4_config_id'     : ps4_config_prop
    }
    
    return config_data

# Initializes variables

config_data = load()

recording_path  = config_data['recording_path']
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

    # Prepares the recording module
    def start(self,save_path):
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

    # Saves supporting files and adds metadata to save_data
    def save(self,save_data):
        pass



# Camera Module
# Module for recording video from webcam
class CameraModule(RecordingModule):
    
    def start(self,save_path):
        self._cap = cv2.VideoCapture(0)
        
        self._exited = False
        self._save_prop = False
        self._save = False
        
        self._path = save_path
        self._file = 'video.avi'
        self._time_file = 'video_frames.json'
        
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        fps = 30.0
        self._width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self._out = cv2.VideoWriter(self._path+self._file, fourcc, fps, (self._width,self._height))
        
        self._first_time = None
        self._first_frame = True
        self._frame_count = 0
        
        self._frame_times = []
        
        if self._cap.isOpened():
        
            if logger.level('debug'):
                self._frame_name = "Recording Window"
                
                print 'Press '+term.BOLD+term.CYAN+'ESC'+term.END+' in {} to finish.'.format(self._frame_name)
                print 'Press '+term.BOLD+term.GREEN+'S'+term.END+' in {} to finish.'.format(self._frame_name)
                print 'Press '+term.BOLD+term.RED+'N'+term.END+' in {} to finish.'.format(self._frame_name)
        
            else:
                logger.log('warn','Set logging level to debug to view recording video')
        else:
            logger.log('error','Video capture failed - no video capture device found')
            quit()
    
    def loop(self):
    
        ret, frame = self._cap.read()
        
        if ret == False:
            return False
    
        self._out.write(frame)
        
        if self._first_frame:
            self._first_time = ext.datetime.now()
            self._first_frame = False
            frame_time = ext.datetime.now() - ext.datetime.now()
        else:
            frame_time = ext.datetime.now() - self._first_time
        
        self._frame_count = self._frame_count + 1
        self._frame_times.append(ext.delta_ms(frame_time))
    
        if logger.level('debug'):
            wait_key = cv2.waitKey(1) & 0xFF
            
            if wait_key == 27: #ESC to escape
                return False
                
            if wait_key == ord('s'): #S to save
                self._exited = True
                self._save_prop = True
                self._save = True
                return False
                
            if wait_key == ord('n'): #N to discard
                self._exited = True
                self._save_prop = True
                self._save = False
                return False
            
            cv2.imshow(self._frame_name,frame)
    
        return True
        
    def finish(self):
        self._cap.release()
        cv2.destroyAllWindows()
        return self._exited, self._save_prop, self._save
       
    def save(self, data):
        self._out.release() #Video out file is saved
        
        time_file_data = {
            "data" : self._frame_times
        }
        config.write(self._path+self._time_file,time_file_data)
        
        data['video_file'] = self._file
        data['video_datetime'] = ext.pack_datetime(self._first_time)
        data['video_width'] = self._width
        data['video_height'] = self._height
        data['video_frame_count'] = self._frame_count
        data['video_frame_times'] = self._time_file
        data['video_duration'] = self._frame_times[-1]


# Control Module
# Module for recording input from PS4 controller
class ControlModule(RecordingModule):

    def start(self,save_path):
        
        self._exited = False
        self._save_prop = False
        self._save = False
        
        self._path = save_path
        self._file = 'control.json'
        self._time_file = 'control_frames.json'
        
        self._controls = []
        self._time_stamper = ext.TimeStamper()
        
        self._pc = ps4.PS4Controller(ps4_config_id)
        created = self._pc.create()
        
        if not created:
            logger.log('error','Error finding PS4 controller')
            quit()
            
        self._arm = al5d.RobotArm()
        self._arm.create()
            
        print 'Press '+term.BOLD+term.CYAN+'HOME'+term.END+' on the PS4 controller to finish.'
        print 'Press '+term.BOLD+term.GREEN+'TRIANGLE'+term.END+' on the PS4 controller to save and exit.'
        print 'Press '+term.BOLD+term.RED+'CIRCLE'+term.END+' on the PS4 controller to discard and exit.'
        
    def loop(self):
        self._pc.poll()
        
        if self._pc.home(): #Home to exit
            return False
            
        if self._pc.triangle(): #Triangle to save
            self._exited = True
            self._save_prop = True
            self._save = True
            return False
            
        if self._pc.circle(): #Circle to discard
            self._exited = True
            self._save_prop = True
            self._save = False
            return False
        
        # Trigger mod
        lt = self._pc.LT() + 1
        rt = self._pc.RT() + 1
        
        #Displacement
        C = [
            self._pc.RX(),                      # BASE
            -self._pc.RY(),                     # SHOULDER
            -self._pc.LY(),                     # ELBOW
            -lt + rt,                           # WRIST
            self._pc.LX(),                      # WRIST_ROTATE
            (-self._pc.L1() + self._pc.R1()),   # GRIPPER 
        ]
        
        # Multipliers
        C[0] = 100.0 * C[0] * C[0] * C[0]
        C[1] = 100.0 * C[1] * C[1] * C[1]
        C[2] = 100.0 * C[2] * C[2] * C[2]
        C[3] = 100.0 * C[3] * C[3] * C[3]
        C[4] = 100.0 * C[4] * C[4] * C[4]
        C[5] = 100.0 * C[5] * C[5] * C[5]
        
        C = [int(c) for c in C]
        
        self._arm.displace(C)
        
        self._controls.append(C)
        
        self._time_stamper.stamp()
         
        return True
            
        
    def finish(self):
        #TODO
        #Currently an error when destroy is called,
        #segmentation fault causes by Pygame
        #self._pc.destroy()
        self._arm.destroy()
        return self._exited, self._save_prop, self._save

    def save(self, data):
        control_file_data = {
            "data" : self._controls
        }
        config.write(self._path+self._file,control_file_data)
    
        frame_times = self._time_stamper.times_ms()
        time_file_data = {
            "data" : frame_times
        }
        config.write(self._path+self._time_file,time_file_data)
        
        data['control_file'] = self._file
        data['control_datetime'] = ext.pack_datetime(self._time_stamper.initial())
        data['control_frame_count'] = len(frame_times)
        data['control_frame_times'] = self._time_file
        data['control_duration'] = frame_times[-1]



# Control Module
# Module for recording input from PS4 controller
class ControlModuleROS(RecordingModule):

    def start(self,save_path):
        
        self._exited = False
        self._save_prop = False
        self._save = False
        
        self._path = save_path
        self._file = 'control.json'
        self._time_file = 'control_frames.json'
        
        self._pub = rospy.Publisher('al5d', Float32MultiArray, queue_size=10)
        rospy.init_node('al5d_pub', anonymous=True)
    
        self._rate = rospy.Rate(60)
        
        self._controls = []
        self._time_stamper = ext.TimeStamper()
        
        self._pc = ps4.PS4Controller(ps4_config_id)
        created = self._pc.create()
        
        if not created:
            logger.log('error','Error finding PS4 controller')
            quit()
            
        print 'Press '+term.BOLD+term.CYAN+'HOME'+term.END+' on the PS4 controller to finish.'
        print 'Press '+term.BOLD+term.GREEN+'TRIANGLE'+term.END+' on the PS4 controller to save and exit.'
        print 'Press '+term.BOLD+term.RED+'CIRCLE'+term.END+' on the PS4 controller to discard and exit.'
        
    def loop(self):
        self._pc.poll()
        
        if rospy.is_shutdown():
            return False
        
        if self._pc.home(): #Home to exit
            return False
            
        if self._pc.triangle(): #Triangle to save
            self._exited = True
            self._save_prop = True
            self._save = True
            return False
            
        if self._pc.circle(): #Circle to discard
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
        self._controls.append(C)
        
        
        msg = Float32MultiArray()
        msg.data = C
        
        self._time_stamper.stamp()
        
        self._pub.publish(msg)
        
        self._rate.sleep()
         
        return True
            
        
    def finish(self):
        #TODO
        #Currently an error when destroy is called,
        #segmentation fault causes by Pygame
        #self._pc.destroy()
        return self._exited, self._save_prop, self._save

    def save(self, data):
        control_file_data = {
            "data" : self._controls
        }
        config.write(self._path+self._file,control_file_data)
    
        frame_times = self._time_stamper.times_ms()
        time_file_data = {
            "data" : frame_times
        }
        config.write(self._path+self._time_file,time_file_data)
        
        data['control_file'] = self._file
        data['control_datetime'] = ext.pack_datetime(self._time_stamper.initial())
        data['control_frame_count'] = len(frame_times)
        data['control_frame_times'] = self._time_file
        data['control_duration'] = frame_times[-1]





# Translator
def translate(node,otype,value):
    #TODO jonathan fill, come up with video format
    if otype=='video':
        return None
    if otype=='video_frames':
        return None
    if otype=='control':
        return None
    if otype=='control_frames':
        return None
    return None


# RECORD
def record_session():

    root, new = node.load(recording_path, translate)
    if new:
        root.set('record_count',0)
        root.set('record_directories',[])
        root.save()
        
    record_count = root.get('record_count')
    record_directories = root.get('record_directories')
    
    sub, _ = root.load('recording{}'.format(record_count))
    sub_path = sub.path()
    
    sub_data = {
        'user' : user,                  #user who recorded
        'task' : task,                  #task recorded
        'camera' : camera,              #camera configuration
        'control' : control,            #control configuration
        
        'video_file' : None,            #video file name
        'video_datetime' : None,        #datetime object of first frame
        'video_width' : None,           #width of video frames
        'video_height' : None,          #height of video frames
        'video_frame_count' : None,     #number of video frames
        'video_frame_times' : None,     #array of ms time difference from first frame for each frame
        'video_duration' : None,        #duration of video = frame_times[-1]
        
        'control_file' : None,          #control file name
        'control_datetime' : None,      #datetime object of start
        'control_frame_count' : None,   #number of controls recorded
        'control_frame_times' : None,   #ms time differences from start
        'control_duration' : None,      #duration of controls recorded
        
        'annotation' : None,            #Can be anything, (ex: 'success','failure',array of data)
        'comments' : None               #Any extra comments for video
    }
    
    camera_module = CameraModule()
    control_module = ControlModule()
    
    # Add recording modules
    modules = []
    if camera == None:
        logger.log('warn','Camera setting is None, no video will be recorded')
    else:
        modules.append(camera_module)
    if control == None:
        logger.log('warn','Control setting is None, no controls will be recorded')
    else:
        modules.append(control_module)
    
    # Start recording
    logger.log('info','Recording index: {}'.format(record_count))
    print 'Starting to record...'
    print 'Press '+term.BOLD+term.RED+'CTRL+C'+term.END+' in terminal to stop and discard video.'
    
    for module in modules:
        module.start(sub_path)
        
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
        for module in modules:
            module.save(sub_data)
        sub.setValues(sub_data)
        sub.save()
        
        record_count += 1
        record_directories.append(sub.relative_path())
        root.set('record_count',record_count)
        root.set('record_directories',record_directories)
        root.save()
        
        print 'Recording saved.'
    else:
        print 'Recording discarded.'


# CONFIG
def config_session():
    rc.edit_config()







#TODO jonathan separate playback in its own module


# Video Module
# Module for playing bacl video from file
class VideoModule(RecordingModule):
    
    def start(self,save_path):
        self._cap = cv2.VideoCapture(0)
        
        self._exited = False
        self._save_prop = False
        self._save = False
        
        self._path = save_path
        self._file = 'video.avi'
        self._time_file = 'video_frames.json'
        
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        fps = 30.0
        self._width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self._out = cv2.VideoWriter(self._path+self._file, fourcc, fps, (self._width,self._height))
        
        self._first_time = None
        self._first_frame = True
        self._frame_count = 0
        
        self._frame_times = []
        
        if self._cap.isOpened():
        
            if logger.level('debug'):
                self._frame_name = "Recording Window"
                
                print 'Press '+term.BOLD+term.CYAN+'ESC'+term.END+' in {} to finish.'.format(self._frame_name)
                print 'Press '+term.BOLD+term.GREEN+'S'+term.END+' in {} to finish.'.format(self._frame_name)
                print 'Press '+term.BOLD+term.RED+'N'+term.END+' in {} to finish.'.format(self._frame_name)
        
            else:
                logger.log('warn','Set logging level to debug to view recording video')
        else:
            logger.log('error','Video capture failed - no video capture device found')
            quit()
    
    def loop(self):
    
        ret, frame = self._cap.read()
        
        if ret == False:
            return False
    
        self._out.write(frame)
        
        if self._first_frame:
            self._first_time = ext.datetime.now()
            self._first_frame = False
            frame_time = ext.datetime.now() - ext.datetime.now()
        else:
            frame_time = ext.datetime.now() - self._first_time
        
        self._frame_count = self._frame_count + 1
        self._frame_times.append(ext.delta_ms(frame_time))
    
        if logger.level('debug'):
            wait_key = cv2.waitKey(1) & 0xFF
            
            if wait_key == 27: #ESC to escape
                return False
                
            if wait_key == ord('s'): #S to save
                self._exited = True
                self._save_prop = True
                self._save = True
                return False
                
            if wait_key == ord('n'): #N to discard
                self._exited = True
                self._save_prop = True
                self._save = False
                return False
            
            cv2.imshow(self._frame_name,frame)
    
        return True
        
    def finish(self):
        self._cap.release()
        cv2.destroyAllWindows()
        return self._exited, self._save_prop, self._save
       
    def save(self, data):
        self._out.release() #Video out file is saved
        
        time_file_data = {
            "data" : self._frame_times
        }
        config.write(self._path+self._time_file,time_file_data)
        
        data['video_file'] = self._file
        data['video_datetime'] = ext.pack_datetime(self._first_time)
        data['video_width'] = self._width
        data['video_height'] = self._height
        data['video_frame_count'] = self._frame_count
        data['video_frame_times'] = self._time_file
        data['video_duration'] = self._frame_times[-1]


# Replay Module
# Module for replaying input from saved control output files
class ReplayModule(RecordingModule):

    def start(self,save_path):
        
        self._exited = False
        self._save_prop = False
        self._save = False
        
        self._path = save_path
        self._file = 'control.json'
        self._time_file = 'control_frames.json'
        
        self._controls = []
        self._time_stamper = ext.TimeStamper()
        
        self._pc = ps4.PS4Controller(ps4_config_id)
        created = self._pc.create()
        
        if not created:
            logger.log('error','Error finding PS4 controller')
            quit()
            
        self._arm = al5d.RobotArm()
        self._arm.create()
            
        print 'Press '+term.BOLD+term.CYAN+'HOME'+term.END+' on the PS4 controller to finish.'
        print 'Press '+term.BOLD+term.GREEN+'TRIANGLE'+term.END+' on the PS4 controller to save and exit.'
        print 'Press '+term.BOLD+term.RED+'CIRCLE'+term.END+' on the PS4 controller to discard and exit.'
        
    def loop(self):
        self._pc.poll()
        
        if self._pc.home(): #Home to exit
            return False
            
        if self._pc.triangle(): #Triangle to save
            self._exited = True
            self._save_prop = True
            self._save = True
            return False
            
        if self._pc.circle(): #Circle to discard
            self._exited = True
            self._save_prop = True
            self._save = False
            return False
        
        # Trigger mod
        lt = self._pc.LT() + 1
        rt = self._pc.RT() + 1
        
        #Displacement
        C = [
            self._pc.RX(),                      # BASE
            -self._pc.RY(),                     # SHOULDER
            -self._pc.LY(),                     # ELBOW
            -lt + rt,                           # WRIST
            self._pc.LX(),                      # WRIST_ROTATE
            (-self._pc.L1() + self._pc.R1()),   # GRIPPER 
        ]
        
        # Multipliers
        C[0] = 100.0 * C[0] * C[0] * C[0]
        C[1] = 100.0 * C[1] * C[1] * C[1]
        C[2] = 100.0 * C[2] * C[2] * C[2]
        C[3] = 100.0 * C[3] * C[3] * C[3]
        C[4] = 100.0 * C[4] * C[4] * C[4]
        C[5] = 100.0 * C[5] * C[5] * C[5]
        
        C = [int(c) for c in C]
        
        self._arm.displace(C)
        
        self._controls.append(C)
        
        self._time_stamper.stamp()
         
        return True
            
        
    def finish(self):
        #TODO
        #Currently an error when destroy is called,
        #segmentation fault causes by Pygame
        #self._pc.destroy()
        self._arm.destroy()
        return self._exited, self._save_prop, self._save

    def save(self, data):
        control_file_data = {
            "data" : self._controls
        }
        config.write(self._path+self._file,control_file_data)
    
        frame_times = self._time_stamper.times_ms()
        time_file_data = {
            "data" : frame_times
        }
        config.write(self._path+self._time_file,time_file_data)
        
        data['control_file'] = self._file
        data['control_datetime'] = ext.pack_datetime(self._time_stamper.initial())
        data['control_frame_count'] = len(frame_times)
        data['control_frame_times'] = self._time_file
        data['control_duration'] = frame_times[-1]





# PLAYBACK
def playback_session():
    root, new = node.load(recording_path, translate)
    if new:
        root.set('record_count',0)
        root.set('record_directories',[])
        root.save()
        logger.log('error','No recordings in new path')
        return
        
    record_count = root.get('record_count')
    record_directories = root.get('record_directories')
    
    last_dir = record_directories[-1]
    
    sub, _ = root.load(last_dir)
    sub_path = sub.path()
    
    
    # Add playback modules
    modules = []
    if camera == None:
        logger.log('warn','Camera setting is None, no video will be recorded')
    else:
        modules.append(camera_module)
    if control == None:
        logger.log('warn','Control setting is None, no controls will be recorded')
    else:
        modules.append(control_module)
    
    
    # Start playback
    logger.log('info','Recording index: {}'.format(record_count))
    print 'Starting to record...'
    print 'Press '+term.BOLD+term.RED+'CTRL+C'+term.END+' in terminal to stop and discard video.'
    
    for module in modules:
        module.start(sub_path)
        
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
    
    print 'Playback finished.'
    
    






 


