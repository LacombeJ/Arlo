
'''
project.py

This module contains functions for loading and managing projects

'''

import entry
import module
import _util as util

import os

import arlo.data.node as node
import arlo.utils.config as config
import arlo.utils.log as log

def load(path):
    root, new = node.load(path)
    if new:
        root.set('data_increment',0)
        root.set('data_directories',[])
        root.set('project','project.json')
        root.save()
    proj = config.read(root.get('project'))
    if proj == None:
        proj = {
            'version' : '1.0.0'
        }
        config.write(path+root.get('project'),proj)
    return Project(root,proj)
    

class Project(object):

    # ----------------------------------------------- #

    def __init__(self,root,proj):
        self._node = root
        self._name = self._node.directory_name()
        self._config = None
        self._proj = proj
        self._data_dir = self._node.get('data_directories')
        self._entry_count = len(self._data_dir)
        self._rec_modules = []
        self._logger = log.Logger()
        
    # ----------------------------------------------- #
        
    def path(self):
        return self._node.absolute_path()
        
    def relative_path(self):
        return self._node.path()
        
    def name(self):
        return self._name
        
    # ----------------------------------------------- #
        
    def getConfig(self):
        return self._config
        
    def setConfig(self,config):
        self._config = config
        
    def get(self,key):
        return self._config[key]
        
    def set(self,key,value):
        self._config[key] = value
        
    def entry_count(self):
        return self._entry_count
        
    def entry(self,index):
        sub_name = self._data_dir[index]
        sub, new = self._node.load(sub_name)
        if new:
            sub.unsafe_erase()
            return None
        return entry.Entry(self,sub)
        
    def getLogger(self):
        return self._logger
        
    def setLogger(self,logger):
        self._logger = logger
        
    # ----------------------------------------------- #
        
    def addRecordingModule(self,mod):
        self._rec_modules.append(mod)
        
    def record(self):
        return util.record(self)
        
    # ----------------------------------------------- #
        
        


        
        
