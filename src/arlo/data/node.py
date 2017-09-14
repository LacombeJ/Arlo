
'''
node.py

This module contains functions for reading and writing data in
a file structure along with metadata

'''

import arlo.utils.config as config
import arlo.utils.io as io


# Loads or creates a directory and meta data
def load(name,translate=None):
    if not io.dir_exists(name):
        io.make_dir(name,False)
        return _Node(name,translate,False,'')
    return _Node(name,translate,True,'')


class _Node(object):

    def __init__(self,name,translate,load,path):
        self._name = name
        self._translate = translate
        self._path = path
        self._node_path = self._path + self._name + '/' 
        self._config_path = self._node_path + 'meta.json'
        self._config = { }
        if load:
            self._config = config.read(self._config_path)
        else:
            config.write(self._config_path, self._config)
        
    # Returns value from given key or value given by translator if otype is not None
    def get(self,key,otype=None):
        value = self._config.get(key)
        if otype==None:
            return value
        else:
            return self._translate(self,otype,value)
    
    # Adds key and value pair to meta data
    def set(self,key,value):
        self._config[key] = value
    
    # Saves meta data
    def save(self):
        config.write(self._config_path, self._config)
    
    # Loads or creates a sub directory and meta data
    def load(self,name):
        if not io.dir_exists(self._node_path + name):
            io.make_dir(self._node_path + name,False)
            return _Node(name,self._translate,False,self._node_path)
        return _Node(name,self._translate,True,self._node_path)
        
    # Returns the path of this node
    def path(self):
        return self._node_path
        

