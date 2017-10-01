
'''
playback.py

This module contains functions for playing back recorded data

'''

import rospy
from std_msgs.msg import Float32MultiArray
import cv2

import node
import util
from util import FrameModule



import arlo.input.ps4 as ps4
import arlo.output.al5d as al5d

import arlo.utils.config as config
import arlo.utils.ext as ext
import arlo.utils.term as term





# Initializes variables

config_data = util.load_config_file(output=False)

recording_path  = config_data['recording_path']
log_prop        = config_data['log_prop']

logger = util.create_logger(log_prop)

window_pos = [(128,64), (900,64), (128, 500), (900,500)]
window_handler = util.WindowHandler(window_pos)



# -------------------------------------------------------------------------- #
# --------------------------------- RECORD --------------------------------- #
# -------------------------------------------------------------------------- #

# -1 index = last session
def playback_session(index=-1):

    root, new = node.load(recording_path, util.translate)
    if new:
        root.set('record_count',0)
        root.set('record_directories',[])
        root.save()
        
    record_count = root.get('record_count')
    record_directories = root.get('record_directories')
    
    if record_count == 0:
        logger.log('error',"No recordings available")
        quit()
    
    rec_dir = record_directories[index]
    
    sub, _ = root.load(rec_dir)
    
    # Read sub node
    print term.GREEN+"Opened directory: '{}'".format(sub.path())+term.END
    
    user = sub.get('user')
    task = sub.get('task')
    camera = sub.get('camera')
    control = sub.get('control')
    
    print term.GREEN+'User: '+term.END+str(user)
    print term.GREEN+'Task: '+term.END+str(task)
    print term.GREEN+'Camera: '+term.END+str(camera)
    print term.GREEN+'Control: '+term.END+str(control)
    
    video_datetime = sub.get('video_datetime',otype='datetime')
    control_datetime = sub.get('control_datetime',otype='datetime')
    
    # Video - control synchronization
    diff_ms = ext.delta_ms(control_datetime - video_datetime)
    sync_ms = 3000 # Give it an extra 3000 seconds, to start
    
    video_ms = sync_ms
    control_ms = sync_ms + diff_ms
    start_time = ext.datetime.now()
    
    video_module = VideoModule('Playback Window',start_time,video_ms)
    output_module = OutputModuleArm('Capture Window',start_time,control_ms) #Arm | ROS
    
    # Add playback modules
    modules = []
    if camera == None:
        logger.log('warn','No camera setting found for recording, no video will be played')
    else:
        modules.append(video_module)
    if control == None:
        logger.log('warn','No control setting found for recording, no controls will be played back')
    else:
        modules.append(output_module)
    
    # Start playback
    logger.log('info','Playback index: {}'.format(index))
    print 'Starting to playback...'
    print 'Press '+term.BOLD+term.RED+'CTRL+C'+term.END+' in terminal to stop.'
    
    #for module in modules:
    #    module.start(sub)
        
    modules = [module for module in modules if module.start(sub)]
    
    started_modules = list(modules)
    
    try:
        loop = True
        while loop:
            modules = [module for module in modules if module.loop()]
            
            # All modules are done
            if len(modules)==0:
                break
    
    except KeyboardInterrupt:
        print ""
        quit()
    
    print 'Playback finished.'
    
    for module in started_modules:
        module.finish()









# --------------------------------- VIDEO ---------------------------------- #
# ----------------------- Module for playing videos ------------------------ #

class VideoModule(FrameModule):
    
    def __init__(self,frame_name='Playback Window',start=None,sync=0):
        FrameModule.__init__(self)
        self._frame_name = frame_name
        self._start = start
        self._sync = sync
    
    def start(self,sub):
        
        duration = sub.get('video_duration')
        print 'Video duration: {} s'.format(duration/1000)
        
        self._cap = sub.get('video_file',otype='video_cap')
        self._frame_times = sub.get('video_frame_times',otype='json_data')
        
        self._time_sync = ext.TimeSync(self._frame_times,self._start,self._sync)
        
        self._first_frame = True
        self._has_frame = False
        self._frame = None
        if self._cap.isOpened():
            print 'Press '+term.BOLD+term.CYAN+'ESC'+term.END+" in '{}' to finish.".format(self._frame_name)
        else:
            logger.log('error','Video playback failed - video cannot be opened')
            
            self._cap.release()
            return False
            
        return True
    
    def loop(self):
        
        exit = False
        
        next, index, done = self._time_sync.sync()
        if done:
            return False
        if next:
            ret, self._frame = self._cap.read()
            self._has_frame = True
            if ret == False:
                exit = True
        
        if exit == True:
            return False
        
        if self._has_frame:
            wait_key = cv2.waitKey(1) & 0xFF
            
            if wait_key == 27: #ESC to escape
                return False
            
            cv2.imshow(self._frame_name,self._frame)
            
            if self._first_frame:
                window_handler.new_window(self._frame_name)
                self._first_frame = False
    
        return True
        
    def finish(self):
        self._cap.release()
        cv2.destroyAllWindows()





# -------------------------------- OUTPUT ---------------------------------- #
# ------------- Module for recording input from PS4 controller ------------- #

class OutputModule(FrameModule):
    
    def __init__(self,frame_name,start=None,sync=0):
        FrameModule.__init__(self)
        self._frame_name = frame_name
        self._start = start
        self._sync = sync
    
    # ---------------------------------------------------------------------- #
    
    # Override this method to create objects in start method
    def _create(self):
        pass
        
    # Override this method to add updates in loop method (C=robot controls)
    def _update(self, C):
        pass
        
    # Override this method to destroy object in finish method
    def _destroy(self):
        pass
    
    # ---------------------------------------------------------------------- #
    
    def start(self,sub):
        
        self._cap = cv2.VideoCapture(0)
        
        duration = sub.get('control_duration')
        print 'Control duration: {} s'.format(duration/1000)
        
        self._controls = sub.get('control_file',otype='json_data')
        self._frame_times = sub.get('control_frame_times',otype='json_data')
        
        self._time_sync = ext.TimeSync(self._frame_times,self._start,self._sync)
        
        self._frame = None
        
        self._first_frame = True
        
        created = self._create()
        if not created:
            self._cap.release()
            return False
        
        self._has_cap = False
        if self._cap.isOpened():
            self._has_cap = True
            print 'Press '+term.BOLD+term.CYAN+'ESC'+term.END+" in '{}' to finish.".format(self._frame_name)
        else:
            logger.log('error','No video capture device found')
            
        return True

        

    def loop(self):
    
        next, index, done = self._time_sync.sync()
        if done:
            return False
        if next:
            C = self._controls[index]
            if not self._update(C):
                return False
        
        ret, frame = self._cap.read()
        
        if ret == True:

            wait_key = cv2.waitKey(1) & 0xFF
            
            if wait_key == 27: #ESC to escape
                return False
            
            cv2.imshow(self._frame_name,frame)
            if self._first_frame:
                window_handler.new_window(self._frame_name)
                self._first_frame = False
        
        return True
            
        
    def finish(self):
        self._cap.release()
        cv2.destroyAllWindows()
        self._destroy()


# ------------------------------ CONTROL-ARM ------------------------------- #
# Input read DIRECTLY controls the arm bot, no ROS messages are being sent
class OutputModuleArm(OutputModule):

    def _create(self):
        self._arm = al5d.RobotArm()
        
        error = self._arm.create()
        
        if not error:
            return True
            
        logger.log('error','AL5D arm failed to initialize: {}'.format(error))
        return False
        
    def _update(self,C):
        self._arm.move(C)
        return True
        
    def _destroy(self):
        self._arm.destroy()



# ------------------------------ CONTROL-ROS ------------------------------- #
# Publishes to ROS topic
class OutputModuleROS(OutputModule):

    def _create(self):
        self._pub = rospy.Publisher('al5d', Float32MultiArray, queue_size=10)
        rospy.init_node('al5d_pub', anonymous=True)
        self._rate = rospy.Rate(60)
        return True
        
    def _update(self,C):
        if rospy.is_shutdown():
            return False
        msg = Float32MultiArray()
        msg.data = C
        self._pub.publish(msg)
        self._rate.sleep()
        return True
        
    def _destroy(self):
        pass




 


