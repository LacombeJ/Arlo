
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

window_pos = [(32,32), (700,32), (32, 500), (700,500)]
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
    
    video_module = VideoModule('Playback Window')
    output_module = OutputModuleROS('Capture Window') #Arm | ROS
    
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
    
    for module in modules:
        module.start(sub)
    
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
    
    for module in modules:
        module.finish()









# --------------------------------- VIDEO ---------------------------------- #
# ----------------------- Module for playing videos ------------------------ #

class VideoModule(FrameModule):
    
    def __init__(self,frame_name='Playback Window'):
        FrameModule.__init__(self)
        self._frame_name = frame_name
    
    def start(self,sub):
        
        duration = sub.get('video_duration')
        print 'Video duration: {} s'.format(duration/1000)
        
        self._cap = sub.get('video_file',otype='video_cap')
        self._frame_times = sub.get('video_frame_times',otype='json_data')
        
        self._time_stamper = ext.TimeStamper()
        
        self._index = 0
        self._frame = None
        if self._cap.isOpened():
            print 'Press '+term.BOLD+term.CYAN+'ESC'+term.END+" in '{}' to finish.".format(self._frame_name)
        else:
            logger.log('error','Video capture failed - no video capture device found')
            quit()
    
    def loop(self):
        
        first_frame = self._index == 0
        exit = False
        
        if self._index >= len(self._frame_times):
            return False
        
        if self._index==0 or self._time_stamper.last_time_ms() > self._frame_times[self._index]:
            ret, self._frame = self._cap.read()
            self._index += 1
            if ret == False:
                exit = True
        
        if exit == True:
            return False
        
        self._time_stamper.stamp()
        
        wait_key = cv2.waitKey(1) & 0xFF
        
        if wait_key == 27: #ESC to escape
            return False
        
        cv2.imshow(self._frame_name,self._frame)
        
        if first_frame:
            window_handler.new_window(self._frame_name)
    
        return True
        
    def finish(self):
        self._cap.release()
        cv2.destroyAllWindows()





# -------------------------------- OUTPUT ---------------------------------- #
# ------------- Module for recording input from PS4 controller ------------- #

class OutputModule(FrameModule):
    
    def __init__(self,frame_name):
        FrameModule.__init__(self)
        self._frame_name = frame_name
    
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
        
        self._time_stamper = ext.TimeStamper()
        
        self._index = 0
        self._frame = None
        
        self._has_cap = False
        if self._cap.isOpened():
            self._has_cap = True
            print 'Press '+term.BOLD+term.CYAN+'ESC'+term.END+" in '{}' to finish.".format(self._frame_name)
        else:
            logger.log('error','No video capture device found')
            
        self._create()

        

    def loop(self):
        
        first_frame = self._index == 0
        
        if self._index >= len(self._frame_times):
            return False
        
        if self._index==0 or self._time_stamper.last_time_ms() > self._frame_times[self._index]:
            C = self._controls[self._index]
            if not self._update(C):
                return False
            self._index += 1
        
        self._time_stamper.stamp()
        
        ret, frame = self._cap.read()
        
        if ret == True:

            wait_key = cv2.waitKey(1) & 0xFF
            
            if wait_key == 27: #ESC to escape
                return False
            
            cv2.imshow(self._frame_name,frame)
            if first_frame:
                window_handler.new_window(self._frame_name)
        
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
        self._arm.create()
        
    def _update(self,C):
        self._arm.displace(C)
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




 


