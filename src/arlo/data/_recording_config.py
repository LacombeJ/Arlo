
'''
_recording_config.py

This module contains private utility functions to handle recording configuration file

'''


import arlo.utils.config as au_config
import arlo.utils.term as term
import arlo.utils.ext as ext


_recording_config = 'recording.json'



'''
Attempts to read recording.json, if not found returns None
'''
def read_config():
    return au_config.read(_recording_config)

'''
Attempts to read recording.json, if not found creates one
'''
def read_or_create_config():
    result = au_config.read(_recording_config)
    if result == None:
        print "File '{}' not found".format(_recording_config)
        result = _create_config()
    else:
        print "File '{}' found".format(_recording_config)
        check_result = _check_config(result)
        if check_result is not None:
            result = check_result
            au_config.write(_recording_config,result) #write again if missing parameters
            print "Updated config at '{}'".format(_recording_config)
        else:
            print "Config not changed: '{}'".format(_recording_config)
    return result



'''
Edits recording.json, if it does not exist, creates it
'''
def edit_config():
    result = au_config.read(_recording_config)
    if result == None:
        print "File '{}' not found".format(_recording_config)
        result = _create_config()
    else:
        print "File '{}' found".format(_recording_config)
        edit_result = _edit_config(result)
        if edit_result is not None:
            result = edit_result
            au_config.write(_recording_config,result) #write again if missing parameters
            print "Updated config at '{}'".format(_recording_config)
        else:
            print "Config not changed: '{}'".format(_recording_config)
    return result





#Prints output in blue then value in white
def _output_value(output,value,white=True):
    if white:
        term.print_blue_white(output,value)
    else:
        term.print_blue_purple(output,value)
        

#Reads json config file and asks user to enter a value from the values in the config
def _read_config_enter_attrib(name, set_name, json_file, default):
    _config = au_config.read(json_file)    
    print "Read '{}'".format(json_file)
    config_set = [i for i in _config]
    _output_value("Available {}: ".format(set_name),ext.list_format(config_set))
    
    if default == None:
        default = config_set[0]
    attrib = _default("Enter {}: ".format(name), default)
    element = _config.get(attrib)
    while element==None:
        print term.RED + "{} not found in {}, try again.".format(attrib,json_file) + term.END
        attrib = raw_input(term.GREEN + "Enter {}: ".format(name) + term.END)
        element = _config.get(attrib)

    return attrib

#Outputs the selected attribute
def _output_attrib(name,attrib):
    _output_value("{} is: ".format(name.capitalize()), attrib, False)  

#Asks the user to enter attribute from a json string
def _enter_attrib(name,set_name,json_file,override_default=None):
    attrib = _read_config_enter_attrib(name, set_name, json_file, override_default)
    _output_attrib(name,attrib)
    return attrib

#Outputs the selected path
def _output_path(name,path):
    _output_value("{} path is: ".format(name.capitalize()), path, False)

#Asks the user to enter path and prints the path value, returns the path
def _enter_path(name,default):
    path = _default('Enter {} path: '.format(name), default)
    _output_path(name,path)
    return path

#Asks the user to enter a value, prints the default value for an empty string
#and returns the value the user enters (or the default if the user enters an empty string)
def _default(statement, default):
    result = default
    default_statement = term.BLUE + "[Default is: " + term.END + default + term.BLUE + "]" + term.END
    val = raw_input( term.GREEN + statement + default_statement + " ")
    if val != '':
        result = val
    return result




#Writes new a json config file
def _create_config():
    print "Attempting to create config file ..."
    print "Leave entries empty to set value to default" + term.END
    
    user = _enter_attrib('user','users','users.json')
    task = _enter_attrib('task','tasks','tasks.json')
    recording_path = _enter_path('recording','Recording/')
    raw_data_path = _enter_path('raw data','Raw_data/')
    data_path = _enter_path('data','Data/')

    rconfig = {
        'user' : user,
        'task' : task,
        'recording_path' : recording_path,
        'raw_data_path' : raw_data_path,
        'data_path' : data_path
    }
    
    au_config.write(_recording_config,rconfig)
    print "Wrote config to '{}'".format(_recording_config)



#Checks the json config file to see if it contains the necessary values,
#if it doesn't, it writes to the json config file
def _check_config(rconfig):
    print "Checking config file..."

    #User
    user = rconfig.get('user')
    rconfig['user'], v0 = _valid_attrib(user,'user','users','users.json')

    #Task
    task = rconfig.get('task')
    rconfig['task'], v1 = _valid_attrib(task,'task','tasks','tasks.json')

    #Recording path
    recording_path = rconfig.get('recording_path')
    rconfig['recording_path'], v2 = _valid_path(recording_path,'recording','Recording/')

    #Raw data path
    raw_data_path = rconfig.get('raw_data_path')
    rconfig['raw_data_path'], v3 = _valid_path(raw_data_path,'raw data','Raw_data/')

    #Data path
    data_path = rconfig.get('data_path')
    rconfig['data_path'], v4 = _valid_path(data_path,'data','Data/')

    if v0 and v1 and v2 and v3 and v4: #Valid, rconfig is not changed, no need to return rconfig
        return None

    return rconfig #Was invalid, rconfig is updated
    
#Checks to see if this attribute is valid, if it isn't it asks the user for a valid attribute
def _valid_attrib(attrib,name,set_name,json_file):
    valid = True
    if attrib == None:
        print term.RED + 'Error finding {} in config file'.format(attrib)
        attrib = _read_config_enter_attrib(name,set_name,json_file)
        valid = False
    _output_attrib(name,attrib)
    return attrib, valid

#Checks to see if this path is valid, if it isn't it asks the user for a valid path
def _valid_path(path,name,default):
    valid = True
    if path == None:
        print term.RED + 'Error finding {} path in config file'.format(name)
        path = _default('Enter {} path: '.format(name), default)
        valid = False
    _output_path(name,path)
    return path, valid




#Checks the json config file to see if it contains the necessary values,
#if it doesn't, it writes to the json config file
def _edit_config(rconfig):
    print "Reading config file..."
    print "Leave entries empty to set value to default" + term.END
    
    #User
    user = rconfig.get('user')
    if user == None:
        rconfig['user'], v0 = _valid_attrib(user,'user','users','users.json')
    else:
        rconfig['user'] = _enter_attrib('user','users','users.json',user)
        v0 = rconfig['user'] == user

    #Task
    task = rconfig.get('task')
    if task == None:
        rconfig['task'], v1 = _valid_attrib(task,'task','tasks','tasks.json')
    else:
        rconfig['task'] = _enter_attrib('task','tasks','tasks.json',task)
        v1 = rconfig['task'] == task

    #Recording path
    recording_path = rconfig.get('recording_path')
    if recording_path == None:
        rconfig['recording_path'], v2 = _valid_path(recording_path,'recording','Recording/')
    else:
        rconfig['recording_path'] = _enter_path('recording',recording_path)
        v2 = rconfig['recording_path'] == recording_path

    #Raw data path
    raw_data_path = rconfig.get('raw_data_path')
    if raw_data_path == None:
        rconfig['raw_data_path'], v3 = _valid_path(raw_data_path,'raw data','Raw_data/')
    else:
        rconfig['raw_data_path'] = _enter_path('raw data',raw_data_path)
        v3 = rconfig['raw_data_path'] == raw_data_path

    #Data path
    data_path = rconfig.get('data_path')
    if data_path == None:
        rconfig['data_path'], v4 = _valid_path(data_path,'data',data_path)
    else:
        rconfig['data_path'] = _enter_path('data','Data/')
        v4 = rconfig['data_path'] == data_path

    
    if v0 and v1 and v2 and v3 and v4: #Not edited, rconfig is not changed, no need to return rconfig
        return None

    return rconfig #Was edited, rconfig is updated






