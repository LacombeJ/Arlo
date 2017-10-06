
'''
_util.py

This module contains utility functions for api calls

'''

import project

import arlo.utils.config as config
import arlo.utils.ext as ext
import arlo.utils.term as term

import cv2





# Default Translator
def default_translate(path,otype,value):
    if otype=='video_cap':
        return cv2.VideoCapture(path+value)
    if otype=='json_data':
        return config.read(path+value).get('data')
    if otype=='datetime':
        return ext.unpack_datetime(value)
    return None

# Translate
def translate(translators,path,otype,value):
    if otype==None:
        return value
    for T in translators:
        trans = T(path,otype,value)
        if trans != None:
            return trans
    return default_translate(path,otype,value)







# -------------------------------------------------------------------------- #
# --------------------------------- RECORD --------------------------------- #
# -------------------------------------------------------------------------- #

def record(proj,modules,require=True):

    log = proj.getLogger()

    data_increment = proj._node.get(project.INCREMENT)
    data_directories = proj._node.get(project.DIRECTORIES)
    
    sub, _ = proj._node.load('{}{}/'.format(project.ENTRY,data_increment))
    sub_path = sub.path()
    
    sub_data = { }
    
    # Create and add modules
    modules = [module() for module in modules]
    
    # Run module.init
    for module in modules:
        module.init(log)
    
    # Run module.start and filter successful modules
    modules_dropped = False
    modules_start = []
    for module in modules:
        if module.start(proj._node.path()):
            modules_start.append(module)
        else:
            modules_dropped = True
            
    if require and modules_dropped:
        log.error('All modules are required to record yet not all were successful.')
        sub.unsafe_erase()
        return None
            
    modules = modules_start
    
    start = True
    b_save_prop = False
    b_save = False
        
    if len(modules) == 0:
        log.error('No modules available to record')
        start = False
        exit = True

    if start:

        # Start recording
        log.debug('Recording index: {}'.format(data_increment))
        log.debug('Starting to record...')
        log.debug('Press '+term.BOLD+term.RED+'CTRL+C'+term.END+' in terminal to stop.')
        
        # Run module.update loop
        try:
            loop = True
            while loop and len(modules)!=0:
                for module in modules:
                    mod_loop = module.update()
                    if not mod_loop:
                        loop = False
                        b_save_prop, b_save = module.getExitFlags()
                        break
                    else:
                        loop = True
        except KeyboardInterrupt:
            pass
        
        log.debug('Recording finished.')
        
        # Run module.finish
        for module in modules:
            module.finish()
        
    if not b_save_prop:
        # Ask user if should save recording
        save = True
        save_input = raw_input(term.BLUE+'Do you wish to save the recording? '+term.END+'[Y/n] ')
        if save_input=='y' or save_input=='Y' or save_input=='':
            save = True
        else:
            save = False
    else:
        save = b_save
            
    # Save recording
    if save:
    
        for module in modules:
            module.save(sub_data)
        sub.setValues(sub_data)
        sub.save()
        
        data_increment += 1
        data_directories.append(sub.relative_path())
        proj._node.set(project.INCREMENT,data_increment)
        proj._node.set(project.DIRECTORIES,data_directories)
        proj._node.save()
        
        log.debug('Recording saved.')
        
        return project.Entry(proj,sub)
        
    # Discard recording and delete created files
    else:
        for module in modules:
            module.delete()
        sub.unsafe_erase()
        
        log.debug('Recording not saved.')
        
        return None
        



















# -------------------------------------------------------------------------- #
# -------------------------------- PLAYBACK -------------------------------- #
# -------------------------------------------------------------------------- #


def playback(proj,modules,index=-1,require=True):

    log = proj.getLogger()

    data_increment = proj._node.get(project.INCREMENT)
    data_directories = proj._node.get(project.DIRECTORIES)
    
    rec_dir = data_directories[index]
    sub, new = root.load(rec_dir)
    if new:
        log.error('Playback failed, entry does not exist.')
        sub.unsafe_erase()
        return None
    sub_path = sub.path()
    
    sub_data = sub.getData()
    
    # Create and add modules
    modules = [module() for module in modules]
    
    def mod_translate(path,otype,value):
        return translate(proj._translators,path,otype,value)
    
    # Run module.init
    for module in modules:
        module.init(log,sub_data,mod_translate)
    
    # Run module.start and filter successful modules
    modules_dropped = False
    modules_start = []
    for module in modules:
        if module.start(proj._node.path()):
            modules_start.append(module)
        else:
            modules_dropped = True
            
    if require and modules_dropped:
        log.error('All modules are required to playback yet not all were successful.')
        sub.unsafe_erase()
        return None
            
    modules = modules_start
    
    b_save_prop = False
    b_save = False
        
    if len(modules) == 0:
        log.error('No modules available to playback')
        return

    # Start playback
    log.debug('Playback index: {}'.format(data_increment))
    log.debug('Starting to playback...')
    log.debug('Press '+term.BOLD+term.RED+'CTRL+C'+term.END+' in terminal to stop.')
    
    # Run module.update loop
    loop_modules = list(modules)
    try:
        while True:
            loop_modules = [module for module in loop_modules if module.update()]
            
            # All modules are done
            if len(loop_modules)==0:
                break
    except KeyboardInterrupt:
        pass
    
    log.debug('Playback finished.')
    
    # Run module.finish
    for module in modules:
        module.finish()




