
'''
node.py

This module contains functions for reading and writing data in
a file structure along with metadata

version 1.3
'''

import os

import arlo.utils.config as config
import arlo.utils.io as io
import arlo.utils.trash as trash

# Loads or creates a directory and meta data
# Returns node, new if directory is created
def load(name,translate=None):
    if not io.dir_exists(name+'/meta.json'):
        io.make_dir(name,False)
        return _Node(name,translate,False,''), True
    return _Node(name,translate,True,''), False


class _Node(object):

    def __init__(self,name,translate,load,path):
        self._name = name
        if self._name[-1] != '/':
            self._name += '/'
        self._translate = translate
        self._path = path
        self._node_path = self._path + self._name
        self._config_path = self._node_path + 'meta.json'
        self._config = { }
        if load:
            self._config = config.read(self._config_path)
            if self._config == None:
                print "Error reading: '{}'".format(self._config_path)
        else:
            config.write(self._config_path, self._config)
        
    # Returns value from given key or value given by translator if otype is not None
    def get(self,key,otype=None):
        value = self._config.get(key)
        
        if otype==None:
            return value
        else:
            return self._translate(self.path(),otype,value)
    
    # Adds key and value pair to meta data
    def set(self,key,value):
        self._config[key] = value
    
    # Returns a copy of key-value dict
    def getValues(self):
        return dict(self._config)
    
    # Adds multiple key, value pairs
    def setValues(self,configs):
        for key in configs:
            self._config[key] = configs[key]
    
    # Saves meta data
    def save(self):
        config.write(self._config_path, self._config)
    
    # Trashes node
    def delete(self):
        trash.delete(self._node_path)
    
    # Directory must be empty
    def unsafe_erase(self):
        os.remove(self._config_path)
        try:
            os.rmdir(self._node_path)
        except OSError:
            print "OSError caught trying to remove {}".format(self._node_path)
    
    # Loads or creates a sub directory and meta data
    # Returns node, new if directory is created
    def load(self,name):
        if not io.dir_exists(self._node_path + name):
            io.make_dir(self._node_path + name,False)
            return _Node(name,self._translate,False,self._node_path), True
        return _Node(name,self._translate,True,self._node_path), False
        
    # Returns whether a given file exists in this node directory
    def fileExists(self, fname):
        return io.dir_exists(self.path()+'/'+fname)
        
    # Returns the path relative to the parent node ('node_path/')
    def relative_path(self):
        return self._name
        
    # Returns the path of this node relative to the working directory ('parent_path/node_path/')
    def path(self):
        return self._node_path
        
    # Returns the absolute path of this node ('/../../../parent_path/node_path/')
    def absolute_path(self):
        return os.path.realpath(self._node_path)
        
    def directory_name(self):
        _, dir_name = os.path.split(self.absolute_path())
        return dir_name
        
    

