
'''
module.py

This module contains functions for creating and extending module objects
'''

import arlo.input.ps4 as ps4
import arlo.output.al5d as al5d

import arlo.utils.config as config
import arlo.utils.ext as ext
import arlo.utils.log as log
import arlo.utils.term as term

import os
import cv2






# Types of modules: record, playback, edit, play
class Module(object):

    def __init__(self):
        self._parent = None
        self._children = []
        self._exited = False
        self._save_property = False
        self._save = False
        
        # Used by timesync
        self._index = 0
        self._times = None
        self._datetime = None #For synchronzing multiple modules
        
        # Set externally
        self._log = None
        self._node = None
        
    def get(self,key,otype=None):
        return self._node.get(key,otype)
        
    def getIndex(self):
        return self._index
        
    def setIndex(self,index):
        self._index = index
        
    def getTimes(self):
        return self._times
        
    def setTimes(self,times):
        self._times = times
        
    def getDatetime(self):
        return self._datetime
        
    def setDatetime(self,datetime):
        self._datetime = datetime
        
    # ---------------------------------------------------------------------------------- #
        
    def append(self,child):
        self._children.append(child)
        child._parent = self
        
    def setExitFlags(self,exit_flags):
        self._save_property, self._save = exit_flags
        
    def getExitFlags(self):
        return self._save_property, self._save
        
    def exitSave(self):
        self._save_property = True
        self._save = True
        return False
        
    def exitDiscard(self):
        self._save_property = True
        self._save = False
        return False
    
    # ---------------------------------------------------------------------------------- #
      
    # Set universal variables and passes along submodules  
    def init(self,log=log.Logger(),node=None):
        
        self._log = log
        self._node = node
        
        self.onInit(self._parent,log,node)
        for child in self._children:
            child.init(log,node)
        
    def start(self,path):
        if not self.onStart(self._parent,path):
            return False
        for child in self._children:
            if not child.start(path):
                return False
        return True
    
    def update(self):
        if not self.onUpdate(self._parent):
            return False
        for child in self._children:
            if not child.update():
                self.setExitFlags(child.getExitFlags())
                return False
        return True
            
    def finish(self):
        self.onFinish(self._parent)
        for child in self._children:
            child.finish()
            
    def save(self,data):
        self.onSave(self._parent,data)
        for child in self._children:
            child.save(data)
            
    def delete(self):
        self.onDelete(self._parent)
        for child in self._children:
            child.delete()
    
    # ---------------------------------------------------------------------------------- #
    
    # Implemented by ( record, playback, edit, play )
    def onInit(self,parent,log,node):
        pass
    
    # Implemented by ( record, playback, edit, play )
    def onStart(self,parent,path):
        return True
        
    # Implemented by ( record, playback,       play )
    def onUpdate(self,parent):
        return True
        
    # Implemented by ( record, playback, edit, play )
    def onFinish(self,parent):
        pass
        
    # Implemented by ( record,           edit       )
    def onSave(self,parent,data):
        pass
        
    # Implemented by ( record,           edit       )
    def onDelete(self,parent):
        pass
    
    # ---------------------------------------------------------------------------------- #
    












# ----------------------------------- Generic Modules ----------------------------------- #

# Child module of a frame module
# frame modules implement frame() which returns an image
class FrameModule_CV(Module):
    
    def __init__(self, name, pos=(0,0)):
        Module.__init__(self)
        self._frame_name = name
        self._pos = pos
        
    def onStart(self,parent,path):
        self._first_frame = True
        self._log.debug('Press '+term.BOLD+term.CYAN+'ESC'+term.END+" in '{}' to finish.".format(self._frame_name))
        return True
        
    def onUpdate(self,parent):
        cv2.imshow(self._frame_name,parent.frame())
        
        if self._first_frame:
            cv2.moveWindow(self._frame_name,self._pos[0],self._pos[1])
            self._first_frame = False
        
        wait_key = cv2.waitKey(1) & 0xFF
        
        if wait_key == 27: #ESC to escape
            return False
        
        return True
        
    def onFinish(self,parent):
        cv2.destroyAllWindows()


# Does not implement onStart
class VideoCaptureModule(Module):
    
    def __init__(self):
        Module.__init__(self)
    
    # Init: self._cap [cv2.VideoCapture]
    def onStart(self,parent,path):
        raise NotImplementedError('onStart')
    
    def onUpdate(self,parent):
        ret, self._frame = self._cap.read()
        if ret == False:
            return False
        return True
        
    def onFinish(self,parent):
        self._cap.release()

    def cap(self):
        return self._cap

    def frame(self):
        return self._frame



class Al5dModule(Module):

    def __init__(self):
        Module.__init__(self)
        
    def onStart(self,parent,path):
        self._arm = al5d.RobotArm()
        err = self._arm.create()
        if err != None:
            self._log.warn('AL5D arm failed to be initialized - {}'.format(err))
            return False
        
        return True
        
    def onFinish(self,parent):
        self._arm.center()
        self._arm.destroy()
        
    def center(self):
        return self._arm.center()
        
    def move(self,C):
        return self._arm.move(C)
        
    def displace(self,C):
        return self._arm.displace(C)
        
    def displace_IK(self,C):
        return self._arm.displace_IK(C)
        
    def get_pos(self):
        return self._arm.get_pos()


# Synchronizes module update function
class SingleSyncModule(Module):
    
    def __init__(self,mod):
        Module.__init__(self)
        self._mod = mod
    
    def onInit(self,parent,log,node):
        self._mod.init(log,node)
        
    def onStart(self,parent,path):
        if not self._mod.start(path):
            return False
        
        sync_ms = 1000 # Give an extra 1000 seconds to start
        start_time = ext.datetime.now()
        self._time_sync = ext.TimeSync(self._mod.getTimes(),start_time,sync_ms)
        
        return True
        
    def onUpdate(self,parent):
        next, index, done = self._time_sync.sync()
        
        if done:
            return False
            
        if next:
            self._mod.setIndex(index)
            if not self._mod.update():
                return False
                
        return True
        
    def onFinish(self,parent):
        self._mod.finish()
        
    def onSave(self,parent,data):
        self._mod.save(data)
        
    def onDelete(self,parent):
        self._mod.delete()


# Synchronizes two modules update function
class DoubleSyncModule(Module):
    
    def __init__(self,mod_a,mod_b):
        Module.__init__(self)
        self._mod_a = mod_a
        self._mod_b = mod_b
    
    def onInit(self,parent,log,node):
        self._mod_a.init(log,node)
        self._mod_b.init(log,node)
        
    def onStart(self,parent,path):
        if not self._mod_a.start(path):
            return False
        
        
        if not self._mod_b.start(path):
            return False
        
        diff_ms = ext.delta_ms( self._mod_a.getDatetime() - self._mod_b.getDatetime() )
        
        sync_ms = 3000 # Give an extra 3000 seconds to start
        
        b_ms = 0        + sync_ms
        a_ms = diff_ms  + sync_ms
        
        start_time = ext.datetime.now()
        
        self._time_sync_a = ext.TimeSync(self._mod_a.getTimes(),start_time,a_ms)
        self._time_sync_b = ext.TimeSync(self._mod_b.getTimes(),start_time,b_ms)
        
        return True
        
    def onUpdate(self,parent):
        
        # mod_a
        next, index, done = self._time_sync_a.sync()
        
        if done:
            return False
            
        if next:
            self._mod_a.setIndex(index)
            if not self._mod_a.update():
                return False
                
        # mod_b
        next, index, done = self._time_sync_b.sync()
        
        if done:
            return False
            
        if next:
            self._mod_b.setIndex(index)
            if not self._mod_b.update():
                return False
                
        return True
        
    def onFinish(self,parent):
        self._mod_a.finish()
        self._mod_b.finish()
        
    def onSave(self,parent,data):
        self._mod_a.save(data)
        self._mod_b.save(data)
        
    def onDelete(self,parent):
        self._mod_a.delete()
        self._mod_b.delete()



















# ----------------------------------- Recording Modules ----------------------------------- #
    
    

class ModuleCamera(VideoCaptureModule):
    
    def __init__(self):
        VideoCaptureModule.__init__(self)
    
    def onStart(self,parent,path):
        self._cap = cv2.VideoCapture(0)
        if not self._cap.isOpened():
            self._log.warn('Video capture failed - no video capture device found')
            return False
        return True


class ModuleTime(Module):
    
    def __init__(self,name):
        Module.__init__(self)
        self._name = name
    
    def onStart(self,parent,path):
        self._path = path
        self._time_file = '{}_frames.json'.format(self._name)        
        self._time_stamper = ext.TimeStamper()
        return True
    
    def onUpdate(self,parent):
        self._time_stamper.stamp()
        return True

    def onSave(self,parent,data):
        frame_times = self._time_stamper.times_ms()
        time_file_data = {
            "data" : frame_times
        }
        config.write(self._path+self._time_file,time_file_data,compact=True)
        
        data['{}_datetime'.format(self._name)] = ext.pack_datetime(self._time_stamper.initial())
        data['{}_frame_count'.format(self._name)] = len(frame_times)
        data['{}_frame_times'.format(self._name)] = self._time_file
        data['{}_duration'.format(self._name)] = frame_times[-1]


# Child module of CameraModule
class ModuleCamera_Video(Module):
    
    def __init__(self):
        Module.__init__(self)
    
    def onStart(self,parent,path):
        self._path = path
        self._file = 'video.avi'
        
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        fps = 30.0
        self._width = int(parent.cap().get(cv2.CAP_PROP_FRAME_WIDTH))
        self._height = int(parent.cap().get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self._out = cv2.VideoWriter(self._path+self._file, fourcc, fps, (self._width,self._height))
        
        return True
    
    def onUpdate(self,parent):
        self._out.write(parent.frame())
        return True
        
    def onFinish(self,parent):
        self._out.release()
        
    def onSave(self,parent,data):
        data['video_file'] = self._file
        data['video_width'] = self._width
        data['video_height'] = self._height
        
    def onDelete(self,parent):
        os.remove(self._path+self._file)

        
class ModuleCameraExtended(ModuleCamera):
    
    def __init__(self):
        ModuleCamera.__init__(self)
        self.append(ModuleTime('video'))
        self.append(ModuleCamera_Video())

class ModuleCameraExtendedcv(ModuleCamera):
    
    def __init__(self,pos=(0,0)):
        ModuleCamera.__init__(self)
        self.append(ModuleTime('video'))
        self.append(ModuleCamera_Video())
        self.append(FrameModule_CV('Recording Window',pos))


class ModulePs4(Module):
    
    def __init__(self, controller=None, ps4_config_id=0):
        Module.__init__(self)
        self._controller = controller
        self._ps4_config_id = 0
        
    def onStart(self,parent,path):
        
        self._handle_controller = False
        
        if self._controller == None:
            self._controller = ps4.PS4Controller(self._ps4_config_id)
            created = self._controller.create()
            if not created:
                self._log.warn('PS4 controller failed to be created')
                return False
            self._handle_controller = True
        
        self._log.debug('Press '+term.BOLD+term.GREEN+'TRIANGLE'+term.END+' on the PS4 controller to save and exit.')
        self._log.debug('Press '+term.BOLD+term.RED+'CIRCLE'+term.END+' on the PS4 controller to discard and exit.')
        
        return True
        
    def onUpdate(self,parent):
        
        self._controller.poll()
            
        if self._controller.triangle(): #Triangle to save
            return self.exitSave()
            
        if self._controller.circle(): #Circle to discard
            return self.exitDiscard()
        
        return True
        
    def onFinish(self,parent):
        #TODO jonathan do I need to fix or uncomment?
        # Currently an error when _controller.destroy is called,
        # segmentation fault caused by Pygame
        # Program still runs fine with code commented
        
        if self._handle_controller:
            #self._controller.destroy()
            pass
    
    def controller(self):
        return self._controller

# Mapping of PS4 controls to IK-SE controls for an AL5D
class ModulePs4_ikal5d(Module):
    
    def __init__(self):
        Module.__init__(self)
        
    def onStart(self,parent,path):
        self._IK_C = None
        return True
        
    def onUpdate(self,parent):
        
        pc = parent.controller()
        
        # Trigger mod
        lt = pc.LT() + 1
        rt = pc.RT() + 1
        
        #Displacement
        C = [
            pc.LX(),                # BASE
            -pc.LY(),               # SHOULDER
            -pc.RY(),               # ELBOW
            -lt + rt,               # WRIST
            pc.RX(),                # WRIST_ROTATE
            (-pc.L1() + pc.R1()),   # GRIPPER 
        ]
        
        # Multipliers
        C[0] = int(30.0  * C[0])    #BASE
        
        C[3] = int(10.0  * C[3])    #WRIST
        C[4] = int(10.0  * C[4])    #WRIST_ROTATE
        C[5] = int(100.0 * C[5])    #GRIPPER
        
        self._IK_C = C
        
        return True
        
    def controls(self):
        return self._IK_C


class ModuleAl5dps4(Module):

    def __init__(self, controller=None, ps4_config_id=0):
        Module.__init__(self)
        
        # Rather than appending submodules, we create modules and call
        # onMethods ourselves because we need to handle when ps4 polling update
        # method is called (REF#1)
        self._mod_ps4 = ModulePs4(controller,ps4_config_id)
        self._mod_ps4_ikal5d = ModulePs4_ikal5d()
        self._mod_arm = Al5dModule()
        
        self._mod_ps4.append(self._mod_ps4_ikal5d)
        
    def onInit(self,parent,log,node):
        self._mod_ps4.init(log,node)
        self._mod_ps4_ikal5d.init(log,node)
        self._mod_arm.init(log,node)
        
    def onStart(self,parent,path):
        
        if not self._mod_arm.start(path):
            return False
        
        if not self._mod_ps4.start(path):
            return False
            
        self._position = None
        self._time_rate = ext.TimeRate()
        
        return True
        
    def onUpdate(self,parent):
    
        self._changed = False
    
        # This is to avoid sending too much data to arm which may cause an IOError (REF#1)
        if self._time_rate.rate(40):
        
            if not self._mod_ps4.update():
                self.setExitFlags(self._mod_ps4.getExitFlags())
                return False
                
            if self._mod_ps4.controller().home():
                self._changed = self._mod_arm.center()
                self._position = self._mod_arm.get_pos()
            else:
                self._changed = self._mod_arm.displace_IK(self._mod_ps4_ikal5d.controls())
                self._position = self._mod_arm.get_pos()
        
        return True
        
    def onFinish(self,parent):
        self._mod_arm.finish()
        self._mod_ps4.finish()
    
    def changed(self):
        return self._changed
        
    def position(self):
        return self._position


class ModuleAl5dps4_Control(Module):

    def __init__(self):
        Module.__init__(self)
        self._mod_time = ModuleTime('control')
        
    def onInit(self,parent,log,node):
        self._mod_time.init(log,node)
        
    def onStart(self,parent,path):
    
        if not self._mod_time.start(path):
            return False
    
        self._path = path
        self._file = 'control.json'
        
        self._controls = []
        
        return True
        
    def onUpdate(self,parent):
        
        if parent.changed():
            if not self._mod_time.update():
                return False
                
            self._controls.append(parent.position())
    
        return True
        
    def onSave(self,parent,data):
        
        control_file_data = {
            "data" : self._controls
        }
        config.write(self._path+self._file,control_file_data,compact=True)
        
        data['control_file'] = self._file
        
        self._mod_time.save(data)
    
    
class ModuleAl5dps4Extended(ModuleAl5dps4):
    
    def __init__(self, controller=None, ps4_config_id=0):
        ModuleAl5dps4.__init__(self, controller, ps4_config_id)
        self.append(ModuleAl5dps4_Control())










# ----------------------------------- Playback Modules ----------------------------------- #

class ModuleVideo(VideoCaptureModule):
    
    def __init__(self):
        VideoCaptureModule.__init__(self)
    
    def onStart(self,parent,path):
        self._cap = self.get('video_file',otype='video_cap')
        
        if self._cap == None:
            self._log.warn('Video file not found.')
            return False
        
        if not self._cap.isOpened():
            self._log.warn('Video file could not be opened.')
            return False
        
        return True

class ModuleVideoCv(ModuleVideo):

    def __init__(self,pos=(0,0)):
        ModuleVideo.__init__(self)
        self.append(FrameModule_CV('Playback Window',pos))

class ModuleAl5dplayback(Module):

    def __init__(self):
        Module.__init__(self)
        self._mod_arm = Al5dModule()
        
    def onInit(self,parent,log,node):
        self._mod_arm.init(log,node)
        
    def onStart(self,parent,path):
        
        if not self._mod_arm.start(path):
            return False
        
        self._controls = self.get('control_file',otype='json_data')
        
        return True
        
    def onUpdate(self,parent):
        
        index = self.getIndex()
        
        if index >= len(self._controls):
            return False
        
        self._mod_arm.move(self._controls[index])
        
        self.setIndex(index+1)
    
        return True
        
    def onFinish(self,parent):
        self._mod_arm.finish()


class ModuleVideoSync(ModuleVideo):

    def __init__(self):
        ModuleVideo.__init__(self)
        
    def onStart(self,parent,path):
        if not ModuleVideo.onStart(self,parent,path):
            return False
        self.setTimes(self.get('video_frame_times',otype='json_data'))
        self.setDatetime(self.get('video_datetime',otype='datetime'))
        return True

class ModuleVideoSyncCv(ModuleVideoSync):

    def __init__(self, pos=(0,0)):
        ModuleVideoSync.__init__(self)
        self.append(FrameModule_CV('Playback Window',pos))

class ModuleAl5dplaybackSync(ModuleAl5dplayback):

    def __init__(self):
        ModuleAl5dplayback.__init__(self)
        
    def onStart(self,parent,path):
        if not ModuleAl5dplayback.onStart(self,parent,path):
            return False
        self.setTimes(self.get('control_frame_times',otype='json_data'))
        self.setDatetime(self.get('control_datetime',otype='datetime'))
        return True



