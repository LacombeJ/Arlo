
'''
entry.py

This module contains functions for creating and editing entries

'''

import _util as util

class Entry(util.ProjectNode):

    def __init__(self,project,sub):
        util.ProjectNode.__init__(self,project,sub)
        
    
