
'''
trash.py

This module contains utility functions for trashing items

'''

import os

import time
import datetime


TRASH_DIR = os.path.expanduser('~/.arlo/trash/')


def load_root():
    root, new = load(TRASH_DIR)
    if new:
        root.set('trash_count',0)
        root.set('trash_dates',[])
        root.set('trash_items',[])
        root.save()
    return root
    
def delete(file_path):
    
    file_dir, file_name = os.path.split(file_path)
    
    if file_name == "":
        print 'Error: File path ends in "".'
        return
    
    trash_date = _ymd()
    trash_dates = root.get('trash_dates')
    if trash_date not in trash_dates:
        trash_dates.append(trash_date)
        root.set('trash_dates',trash_dates)
        root.save()
    
    trash_node, new = root.load(trash_date)
    if new:
        trash_node.set('trash_count',0)
        trash_node.set('trash_items',[])
        trash_node.save()
        
    trash_count = trash_node.get('trash_count')
    trash_items = trash_node.get('trash_items')
    
    real_path = os.path.realpath(file_path)
    
    item_node, _ = trash_node.load('item{}'.format(trash_count))
    item_node.set('original_path',real_path)
    item_node.save()
    
    trash_count += 1
    trash_items.append(file_name)
    trash_node.set('trash_count',trash_count)
    trash_node.set('trash_items',trash_items)
    trash_node.save()
    
    root.set('trash_items',trash_items)
    root.save()

    os.rename(file_path, item_node.path() + file_name)





def _ymd():
    time = datetime.datetime.now()
    return "{}_{}_{}".format(time.year,time.month,time.day)

    




# TODO update trash._Node and trash.delete function with new version of node (if needed)
# From arlo.utils.node (to avoid circular dependency)
# Delete method from node.py is removed
'''
node.py

This module contains functions for reading and writing data in
a file structure along with metadata

version 1.1
'''
    
import arlo.utils.config as config
import arlo.utils.io as io

# Loads or creates a directory and meta data
# Returns node, new if directory is created
def load(name,translate=None):
    if not io.dir_exists(name):
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
            return self._translate(self,otype,value)
    
    # Adds key and value pair to meta data
    def set(self,key,value):
        self._config[key] = value
    
    # Adds multiple key, value pairs
    def setValues(self,configs):
        for key in configs:
            self._config[key] = configs[key]
    
    # Saves meta data
    def save(self):
        config.write(self._config_path, self._config)
    
    # Removed Delete Method
    
    # Loads or creates a sub directory and meta data
    # Returns node, new if directory is created
    def load(self,name):
        if not io.dir_exists(self._node_path + name):
            io.make_dir(self._node_path + name,False)
            return _Node(name,self._translate,False,self._node_path), True
        return _Node(name,self._translate,True,self._node_path), False
        
    # Returns the path relative to the parent node ('node_path/')
    def relative_path(self):
        return self._name
        
    # Returns the path of this node ('parent_path/node_path/')
    def path(self):
        return self._node_path
        
        





root = load_root()


        
