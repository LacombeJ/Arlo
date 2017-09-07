
'''
io.py

This module contains input/output and directory utilities
'''

import os

#Makes directory if it doesn't exist, and outputs that it created the directory if it did
#Return true if directory was made, false if it existed
def make_dir(path,output=True):
    if not os.path.exists(path):
        os.makedirs(path)
        if output:
            print "Created directory: '{}'".format(path)
        return True
    return False
    
    

