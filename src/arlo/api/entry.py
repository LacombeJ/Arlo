
'''
entry.py

This module contains functions for creating and editing entries

'''


class Entry(object):

    def __init__(self,project,sub):
        self._project = project
        
        self._node = sub
        self._name = self._node.directory_name()
        
        
    def path(self):
        return self._node.absolute_path()
        
    def relative_path(self):
        return self._node.path()
        
    def name(self):
        return self._name
