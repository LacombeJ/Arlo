
'''
project.py

This module contains functions for loading and managing projects

'''

import module
import _util as util

import os

import arlo.data.node as node
import arlo.utils.config as config
import arlo.utils.log as log



# Constants

META_LABEL = 'meta_type'
META = 'arlo_project'

VERSION_LABEL = 'version'
VERSION = '1.0.0'

ENTRY_LABEL = 'entry'
ENTRY = 'data'

INCREMENT = 'data_increment'
DIRECTORIES = 'data_directories'



def load(path):
    root, new = node.load(path)
    if new:
        root.set(META_LABEL,    META)
        root.set(VERSION_LABEL, VERSION)
        root.set(ENTRY_LABEL,   ENTRY)
        root.set(INCREMENT,     0)
        root.set(DIRECTORIES,   [])
        root.save()
    else:
        if root.get(META_LABEL) != META:
            return None
        if root.get(VERSION_LABEL) != VERSION:
            return None
    return Project(root)
    

class ProjectNode(object):
    
    def __init__(self,project,node):
        self._project = project
        self._node = node
        self._name = self._node.directory_name()
        
    def path(self):
        return self._node.absolute_path()
        
    def relative_path(self):
        return self._node.path()
        
    def name(self):
        return self._name
        
    def get(self,key,otype=None):
        return util.translate(self._project._translators,self.relative_path(),otype,self._node.get(key))
        
    def set(self,key,value):
        self._node.set(key,value)
        
    def save(self):
        self._node.save()


        

class Project(ProjectNode):

    # ----------------------------------------------- #

    def __init__(self,root):
        ProjectNode.__init__(self,self,root)
        self._data_dir = self._node.get('data_directories')
        self._entry_count = len(self._data_dir)
        self._logger = log.Logger()
        self._translators = []
    
    # ----------------------------------------------- #
    
    def entry_count(self):
        return self._entry_count
        
    def entry(self,index):
        sub_name = self._data_dir[index]
        sub, new = self._node.load(sub_name)
        if new:
            sub.unsafe_erase()
            return None
        return Entry(self,sub)
        
    def getLogger(self):
        return self._logger
        
    def setLogger(self,logger):
        self._logger = logger
        
    def addTranslator(self,translator):
        self._translators.append(translator)
        
    # ----------------------------------------------- #
        
    def record(self, modules):
        return util.record(self, modules)
        
    # ----------------------------------------------- #
        
        



class Entry(ProjectNode):

    def __init__(self,project,sub):
        ProjectNode.__init__(self,project,sub)


        
        
