#!/usr/bin/env python

import arlo.api.project as project
import arlo.api.module as module

import arlo.utils.log as log
import arlo.utils.term as term


# -------------------------------------------------------------------- #
# Load project

proj = project.load("TestAPI/")



# -------------------------------------------------------------------- #
# Set Logger

logs = {
    "debug" : {"level":0, "term":term.END},
    "info"  : {"level":1, "term":term.CYAN},
    "warn"  : {"level":2, "term":term.YELLOW},
    "error" : {"level":3, "term":term.RED}
}
logger = log.LoggerConsole(logs,0)

proj.setLogger(logger)



# -------------------------------------------------------------------- #
# Set Recording Modules

class EntryRecorder(module.Module):

    def __init__(self):
        module.Module.__init__(self)
        
    def onSave(self,parent,data):
    
        data['user'] = 'jon'
        data['task'] = 1000
        data['camera'] = None
        data['control'] = None
        
        data['annotation'] = ""
        data['comments'] = ""

record_modules = (
    EntryRecorder,
    module.ModuleCameraExtendedcv,
    #module.ModuleAl5dps4Extended
)


# -------------------------------------------------------------------- #
# Set Playback Modules






# -------------------------------------------------------------------- #

print 'Entry count', proj.entry_count()

entry = proj.record(record_modules)



print entry





