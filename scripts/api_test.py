#!/usr/bin/env python

import arlo.api.project as project
import arlo.api.module as module

import arlo.input.ps4 as ps4

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

ps4c = ps4.PS4Controller()
created = ps4c.create()
if not created:
    logger.warn('PS4 controller could not be created')
    ps4c = None

def CustomPs4Module():
    mod = module.ModuleAl5dps4Extended(ps4c)
    return mod

record_modules = (
    EntryRecorder,
    module.ModuleCameraExtendedcv,
    CustomPs4Module
)


# -------------------------------------------------------------------- #
# Set Playback Modules

class EntryPlayback(module.Module):

    def __init__(self):
        module.Module.__init__(self)
        
    def onStart(self,parent,path):
        user = self.get('user')
        task = self.get('task')
        camera = self.get('camera')
        control = self.get('control')
        print "User: {}".format(user)
        print "Task: {}".format(task)
        print "Camera: {}".format(camera)
        print "Control: {}".format(control)
        return True
        
    def onUpdate(self,parent):
        return False
        
def ModuleVideoSync():
    mod_a = module.ModuleVideoSyncCv()
    #mod_b = module.ModuleAl5dplaybackSync()
    sync = module.SingleSyncModule(mod_a)
    #sync = module.DoubleSyncModule(mod_a,mod_b)
    return sync
    
        
playback_modules = (
    EntryPlayback,
    ModuleVideoSync
)


# -------------------------------------------------------------------- #

#print 'Entry count', proj.entry_count()

#entry = proj.record(record_modules)

'''

record_loop = True
while record_loop:

    entry = proj.record(record_modules)

    logger.debug('Press '+term.BOLD+term.CYAN+'Options'+term.END+' on the PS4 controller to record.')
    logger.debug('Press '+term.BOLD+term.YELLOW+'Share'+term.END+' on the PS4 controller to quit.')

    while True:
        
        ps4c.poll()
        
        if ps4c.share():
            record_loop = False
            break
            
        if ps4c.options():
            break


'''

proj.playback(playback_modules)





