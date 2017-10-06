
'''
_util.py

This module contains utility functions for api calls

'''

import arlo.api.entry as entry

import arlo.utils.term as term

def record(project):

    log = project.getLogger()

    data_increment = project._node.get('data_increment')
    data_directories = project._node.get('data_directories')
    
    sub, _ = project._node.load('data{}/'.format(data_increment))
    sub_path = sub.path()
    
    sub_data = { }
    
    # Add recording modules
    modules = [module() for module in project._rec_modules]
    
    for module in modules:
        module.setLogger(log)
    
    modules = [module for module in modules if module.start(project._node.path())]
    
    record = True
    b_save_prop = False
    b_save = False
        
    if len(modules) == 0:
        log.error('No modules available to record')
        record = False
        exit = True

    if record:

        # Start recording
        log.debug('Recording index: {}'.format(data_increment))
        log.debug('Starting to record...')
        log.debug('Press '+term.BOLD+term.RED+'CTRL+C'+term.END+' in terminal to stop.')
                
        
        
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
        project._node.set('data_increment',data_increment)
        project._node.set('data_directories',data_directories)
        project._node.save()
        
        log.debug('Recording saved.')
        
        return entry.Entry(project,sub)
        
    else:
        for module in modules:
            module.delete()
        sub.unsafe_erase()
        
        log.debug('Recording not saved.')
        
        return None

