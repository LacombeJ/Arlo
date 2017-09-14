
'''
log.py

This module is a utility for handling logs and log levels

'''

import term


class Logger(object):

    def __init__(self, logs, level):
        self._logs = logs
        self._level = level
    
    def level(self, key):
        return self._logs[key]['level'] >= self._level
        
    def log(self, key, output):
        if self.level(key):
            print self._logs[key]['term'] + output + term.END
    
