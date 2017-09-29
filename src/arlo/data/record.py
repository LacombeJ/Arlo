
'''
record.py

This module contains functions for recording data

'''

import os
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

config_data = util.load_config_file()

recording_path  = config_data['recording_path']
user            = config_data['user']
task            = config_data['task']
camera          = config_data['camera']
control         = config_data['control']
log_prop        = config_data['log_prop']
save_on_exit    = config_data['save_on_exit']
ps4_config_id   = config_data['ps4_config_id']

logger = util.create_logger(log_prop)





# -------------------------------------------------------------------------- #
# --------------------------------- RECORD --------------------------------- #
# -------------------------------------------------------------------------- #

def record_session():

    root, new = node.load(recording_path, util.translate)
    if new:
        root.set('record_count',0)
        root.set('record_directories',[])
        root.save()
        
    record_count = root.get('record_count')
    record_directories = root.get('record_directories')
    
    sub, _ = root.load('recording{}/'.format(record_count))
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
    control_module = ControlModuleArm() #Arm | ROS
    
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
        for module in modules:
            module.delete()
        sub.unsafe_erase()
        
        print 'Recording discarded.'


















# --------------------------------- CAMERA --------------------------------- #
# ---------------------- Module for recording videos ----------------------- #

class CameraModule(FrameModule):
    
    def start(self,save_path):
        FrameModule.start(self,save_path)
        
        self._cap = cv2.VideoCapture(0)
        
        self._path = save_path
        self._file = 'video.avi'
        self._time_file = 'video_frames.json'
        
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        fps = 30.0
        self._width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self._out = cv2.VideoWriter(self._path+self._file, fourcc, fps, (self._width,self._height))
        
        self._time_stamper = ext.TimeStamper()
        
        if self._cap.isOpened():
        
            if logger.level('debug'):
                self._frame_name = "Recording Window"
                
                print 'Press '+term.BOLD+term.CYAN+'ESC'+term.END+" in '{}' to finish.".format(self._frame_name)
                print 'Press '+term.BOLD+term.GREEN+'S'+term.END+" in '{}' to finish.".format(self._frame_name)
                print 'Press '+term.BOLD+term.RED+'N'+term.END+" in '{}' to finish.".format(self._frame_name)
        
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
        
        self._time_stamper.stamp()
    
        if logger.level('debug'):
            wait_key = cv2.waitKey(1) & 0xFF
            
            if wait_key == 27: #ESC to escape
                return False
                
            if wait_key == ord('s'): #S to save
                self.setFinishValues(True,True,True)
                return False
                
            if wait_key == ord('n'): #N to discard
                self.setFinishValues(True,True,False)
                return False
            
            cv2.imshow(self._frame_name,frame)
    
        return True
        
    def finish(self):
        self._out.release() #Video out file is saved
        self._cap.release()
        cv2.destroyAllWindows()
        return self.getFinishValues()
       
    def save(self, data):
        
        frame_times = self._time_stamper.times_ms()
        time_file_data = {
            "data" : frame_times
        }
        config.write(self._path+self._time_file,time_file_data,compact=True)
        
        data['video_file'] = self._file
        data['video_datetime'] = ext.pack_datetime(self._time_stamper.initial())
        data['video_width'] = self._width
        data['video_height'] = self._height
        data['video_frame_count'] = len(frame_times)
        data['video_frame_times'] = self._time_file
        data['video_duration'] = frame_times[-1]

    def delete(self):
    
        os.remove(self._path+self._file)







# ---------------------------------- LEAP ---------------------------------- #
# ------------- Module for recording input from PS4 controller ------------- #

#TODO jonathan create a super class that LeapModule and ControlModule extends
#TODO update this module to match updates ControlModule
class LeapModule(FrameModule):
    
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
    
    def start(self,save_path):
        FrameModule.start(self,save_path)
        
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
            
        self._create()
        
        print 'Press '+term.BOLD+term.GREEN+'TRIANGLE'+term.END+' on the PS4 controller to save and exit.'
        print 'Press '+term.BOLD+term.RED+'CIRCLE'+term.END+' on the PS4 controller to discard and exit.'
        

    def loop(self):
        self._pc.poll()
        
        if self._pc.triangle(): #Triangle to save
            self.setFinishValues(True,True,True)
            return False
            
        if self._pc.circle(): #Circle to discard
            self.setFinishValues(True,True,False)
            return False
        
        cont, write, POS = self._update(self._pc)
            
        if not cont:
            return False
        
        if write:
            self._time_stamper.stamp()
            self._controls.append(POS)
         
        return True
            
        
    def finish(self):
        #TODO jonathan do I need to fix or uncomment?
        # Currently an error when _pc.destroy is called,
        # segmentation fault causes by Pygame
        # Program still runs fine with code commented
        
        #self._pc.destroy()
        self._destroy()
        return self.getFinishValues()


    def save(self, data):
        control_file_data = {
            "data" : self._controls
        }
        config.write(self._path+self._file,control_file_data,compact=True)
    
        frame_times = self._time_stamper.times_ms()
        time_file_data = {
            "data" : frame_times
        }
        config.write(self._path+self._time_file,time_file_data,compact=True)
        
        data['control_file'] = self._file
        data['control_datetime'] = ext.pack_datetime(self._time_stamper.initial())
        data['control_frame_count'] = len(frame_times)
        data['control_frame_times'] = self._time_file
        data['control_duration'] = frame_times[-1]









# -------------------------------- CONTROL --------------------------------- #
# ------------- Module for recording input from PS4 controller ------------- #

class ControlModule(FrameModule):
    
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
    
    def start(self,save_path):
        FrameModule.start(self,save_path)
        
        self._path = save_path
        self._file = 'control.json'
        self._time_file = 'control_frames.json'
        
        self._controls = []
        self._time_stamper = ext.TimeStamper()
        self._time_rate = ext.TimeRate()
        
        self._pc = ps4.PS4Controller(ps4_config_id)
        created = self._pc.create()
        
        if not created:
            logger.log('error','Error finding PS4 controller')
            quit()
            
        self._create()
        
        print 'Press '+term.BOLD+term.GREEN+'TRIANGLE'+term.END+' on the PS4 controller to save and exit.'
        print 'Press '+term.BOLD+term.RED+'CIRCLE'+term.END+' on the PS4 controller to discard and exit.'
        

    def loop(self):
    
        # This is to avoid sending too much data which may cause an IOError
        if self._time_rate.rate(40):
        
            self._pc.poll()
            
            if self._pc.triangle(): #Triangle to save
                self.setFinishValues(True,True,True)
                return False
                
            if self._pc.circle(): #Circle to discard
                self.setFinishValues(True,True,False)
                return False
            
            cont, write, POS = self._update(self._pc)
            
            if not cont:
                return False
            
            if write:
                self._time_stamper.stamp()
                self._controls.append(POS)
            
         
        return True
            
        
    def finish(self):
        #TODO jonathan do I need to fix or uncomment?
        # Currently an error when _pc.destroy is called,
        # segmentation fault causes by Pygame
        # Program still runs fine with code commented
        
        #self._pc.destroy()
        self._destroy()
        return self.getFinishValues()


    def save(self, data):
        control_file_data = {
            "data" : self._controls
        }
        config.write(self._path+self._file,control_file_data,compact=True)
    
        frame_times = self._time_stamper.times_ms()
        time_file_data = {
            "data" : frame_times
        }
        config.write(self._path+self._time_file,time_file_data,compact=True)
        
        data['control_file'] = self._file
        data['control_datetime'] = ext.pack_datetime(self._time_stamper.initial())
        data['control_frame_count'] = len(frame_times)
        data['control_frame_times'] = self._time_file
        data['control_duration'] = frame_times[-1]


# ------------------------------ CONTROL-ARM ------------------------------- #
# Input read DIRECTLY controls the arm bot, no ROS messages are being sent
class ControlModuleArm(ControlModule):

    def _create(self):
        self._arm = al5d.RobotArm()
        self._arm.create()
        self._control = Control_IK_SE()
        
    def _update(self,pc):
    
        if pc.home():
            POS = True, self._arm.center(), self._arm.get_pos()
        else:
            POS = True, self._control.control(self._arm,pc), self._arm.get_pos()
        
        return POS
        
    def _destroy(self):
        self._arm.destroy()



# ------------------------------ CONTROL-ROS ------------------------------- #
# Publishes to ROS topic
class ControlModuleROS(ControlModule):

    def _create(self):
        self._pub = rospy.Publisher('al5d', Float32MultiArray, queue_size=10)
        rospy.init_node('al5d_pub', anonymous=True)
        self._rate = rospy.Rate(60)
        
    def _update(self,C,pc):
        if rospy.is_shutdown():
            return False, None, None
        msg = Float32MultiArray()
        msg.data = C
        self._pub.publish(msg)
        self._rate.sleep()
        return True, True, C
        
    def _destroy(self):
        pass


class Control_Interface(object):
    # Sends commands to arm
    # Returns arm servo positions
    def control(self,arm,pc):
        pass

class Control_Basic(Control_Interface):

    def control(self,arm,pc):
    
        # Trigger mod
        lt = pc.LT() + 1
        rt = pc.RT() + 1
            
        #Displacement
        C = [
            pc.RX(),                # BASE
            pc.RY(),                # SHOULDER
            pc.LY(),                # ELBOW
            -lt + rt,               # WRIST
            pc.LX(),                # WRIST_ROTATE
            (-pc.L1() + pc.R1()),   # GRIPPER 
        ]
        
        # Multipliers
        C[0] = int(30.0  * C[0])    #BASE
        C[1] = int(10.0  * C[1])    #SHOULDER
        C[2] = int(10.0  * C[2])    #ELBOW
        C[3] = int(10.0  * C[3])    #WRIST
        C[4] = int(10.0  * C[4])    #WRIST_ROTATE
        C[5] = int(100.0 * C[5])    #GRIPPER
    
        return arm.displace(C)
        

class Control_IK_SE(Control_Interface):
    
    def control(self,arm,pc):
    
        # Trigger mod
        lt = pc.LT() + 1
        rt = pc.RT() + 1
        
        #Displacement
        C = [
            pc.LX(),                # BASE
            -pc.LY(),               # SHOULDER
            -pc.RY(),               # ELBOW
            -lt + rt,               # WRIST
            pc.LX(),                # WRIST_ROTATE
            (-pc.L1() + pc.R1()),   # GRIPPER 
        ]
        
        # Multipliers
        C[0] = int(30.0  * C[0])    #BASE
        
        C[3] = int(10.0  * C[3])    #WRIST
        C[4] = int(10.0  * C[4])    #WRIST_ROTATE
        C[5] = int(100.0 * C[5])    #GRIPPER
        
        return arm.displace_IK(C)
        
        




