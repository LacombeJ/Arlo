
import arlo.api.project as project
import arlo.api.module as module

import arlo.input.ps4 as ps4

import arlo.utils.log as log
import arlo.utils.term as term



class ConsoleProject(object):

    def __init__(self):
        
        self._proj = project.load("Recording/")
        self._proj.setLogger(ProjectLogger())
        
        self._record_modules = (
            EntryRecorder,
            module.ModuleCameraExtendedcv,
            module.ModuleAl5dps4Extended
        )
        
        self._playback_modules = (
            EntryPlayback,
            ModuleCameraCv,
            ModuleVideoSync
        )
        
    def project(self):
        return self._proj

    def record(self):
        self._proj.record(self._record_modules)
        
    def playback(self,index=-1):
        self._proj.playback(self._playback_modules,index)



# -------------------------------------------------------------------- #
# Set Logger
def ProjectLogger():
    logs = {
        "debug" : {"level":0, "term":term.END},
        "info"  : {"level":1, "term":term.CYAN},
        "warn"  : {"level":2, "term":term.YELLOW},
        "error" : {"level":3, "term":term.RED}
    }
    logger = log.LoggerConsole(logs,0)
    return logger


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
        
def ModuleCameraCv():
    mod = module.ModuleCamera()
    mod.append(module.FrameModule_CV('Webcam window',pos=(64,64)))
    return mod
        
def ModuleVideoSync():
    mod_a = module.ModuleVideoSyncCv(pos=(800,64))
    mod_b = module.ModuleAl5dplaybackSync()
    #sync = module.SingleSyncModule(mod_a)
    sync = module.DoubleSyncModule(mod_a,mod_b)
    return sync
    
        

