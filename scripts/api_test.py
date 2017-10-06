#!/usr/bin/env python

import arlo.api.project as project
import arlo.api.entry as entry
import arlo.api.module as module

import arlo.utils.log as log
import arlo.utils.term as term


logs = {
    "debug" : {"level":0, "term":term.END},
    "info"  : {"level":1, "term":term.CYAN},
    "warn"  : {"level":2, "term":term.YELLOW},
    "error" : {"level":3, "term":term.RED}
}
logger = log.LoggerConsole(logs,0)

proj = project.load("TestAPI/")
proj.setLogger(logger)

class EntryRecorder(module.Module):
    def __init__(self):
        module.Module.__init__(self)
    def onSave(self,parent,data):
        data['user'] = 'jon'
        data['task'] = 1000
        data['camera'] = None
        data['control'] = None

proj.addRecordingModule(EntryRecorder)
proj.addRecordingModule(module.ModuleCameraExtended)
proj.addRecordingModule(module.ModuleAl5dps4Extended)



print 'Entry count', proj.entry_count()

entry0 = proj.entry(5)
print entry0.name()

entry = proj.record()

print entry





