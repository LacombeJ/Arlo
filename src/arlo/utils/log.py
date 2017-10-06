
'''
log.py

This module is a utility for handling logs and log levels

'''

import term

# Generic logger keys
# -------------------
# debug
# info
# warn
# error


# Base interface for a logger
class Logger(object):

    def level(self, key):
        pass
        
    def log(self, key, output):
        pass

    def debug(self, output):
        self.log('debug',output)
        
    def info(self, output):
        self.log('info',output)
        
    def warn(self, output):
        self.log('warn',output)
        
    def error(self, output):
        self.log('error',output)


class LoggerConsole(Logger):

    def __init__(self, logs, level):
        self._logs = logs
        self._level = level
    
    def level(self, key):
        return self._logs[key]['level'] >= self._level
        
    def log(self, key, output):
        if self.level(key):
            print self._logs[key]['term'] + output + term.END
    
    

