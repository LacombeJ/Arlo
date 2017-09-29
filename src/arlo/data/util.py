
'''
util.py

This module contains some utility functions for handling data

'''

import cv2

import data_config as dc

import arlo.utils.config as config
import arlo.utils.log as log
import arlo.utils.term as term
import arlo.utils.ext as ext


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
def prop(key,dc_config,config_set):
    return dc_config[key], config.read(config_set[key][1])[dc_config[key]]



# Loads the config file and properties in supporting config files
def load_config_file(output=True):
    recording_path = dc.recording_path()
    
    dc_config = dc.read_or_create_config(output)
    config_set = dc.config_set()
    
    user,       user_prop       = prop('user',dc_config,config_set)
    task,       task_prop       = prop('task',dc_config,config_set)
    camera,     camera_prop     = prop('camera',dc_config,config_set)
    control,    control_prop    = prop('control',dc_config,config_set)
    log_level,  log_prop        = prop('log level',dc_config,config_set)
    save_exit,  save_on_exit    = prop('save on exit',dc_config,config_set)
    ps4_config, ps4_config_prop = prop('PS4 config',dc_config,config_set)
    
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
    
    
    
    
# Frame Module Interface
class FrameModule(object):

    # Prepares the frame module
    def start(self,save_path):
        self._exited = False
        self._save_prop = False
        self._save = False
        
    # Return False if should stop the loop
    def loop(self):
        return True
        
    # Return (bool,bool,bool)
    # exited, save_prop, save
    # exited if this module caused the exit
    # save_prop if this module should change the save property
    # save if save_prop and if this module should save or not
    def finish(self):
        return self.getFinishValues()

    # Saves supporting files and adds metadata to save_data
    def save(self,save_data):
        pass
        
    def delete(self):
        pass
        
    def setFinishValues(self, exited, save_prop, save):
        self._exited = exited
        self._save_prop = save_prop
        self._save = save
        
    def getFinishValues(self):
        return self._exited, self._save_prop, self._save
        
        

# Translator
def translate(node,otype,value):
    if otype=='video_cap':
        return cv2.VideoCapture(node.path()+value)
    if otype=='json_data':
        return config.read(node.path()+value).get('data')
    if otype=='datetime':
        return ext.unpack_datetime(value)
    return None
    
    
# Simple handling of window position
window_count = 0
window_multiplier = 32
def new_window(frame_name):
    dxy = window_count * window_multiplier
    cv2.moveWindow(frame_name,100+dxy,100+dxy)
    window_count += 1
    
# Advanced handling of window position
class WindowHandler(object):
    
    # positions [(x,y),...]
    def __init__(self,positions):
        self._positions = positions
        self._index = 0
        
    def new_window(self,frame_name):
        if self._index >= len(self._positions):
            self._index = 0
        posx, posy = self._positions[self._index]
        cv2.moveWindow(frame_name,posx,posy)
        self._index += 1


