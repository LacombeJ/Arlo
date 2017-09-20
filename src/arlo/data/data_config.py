
'''
data_config.py

This module contains private utility functions to handle reading and writing configuration file

'''


import arlo.utils.config as au_config
import arlo.utils.term as term
import arlo.utils.ext as ext




# ===============================================================================

config_file = 'config.json'

#Format of a sample config file
'''
{
    "recording_config": "recording.json", 
    "recording_path": "Recording/",
    "config_path": "Config/",
    "config_set": [
        ["user",    "users",    "users.json"],
        ["task",    "tasks",    "tasks.json"],
        ...
    ]
}
'''

def _prepare_config():
    
    config_json =  au_config.read(config_file)
    
    if config_json == None:
        print term.RED + "File '{}' not found".format(config_file) + term.END
        
    recording_config = config_json['recording_config']
    recording_path = config_json['recording_path']
    config_path = config_json['config_path']
    config_set = config_json['config_set']
    for config in config_set:
        config_set[config][1] = config_path + config_set[config][1]
    return recording_config, recording_path, config_path, config_set

_recording_config, _recording_path, _config_path, _config_set = _prepare_config()

# ===============================================================================


def recording_path():
    return _recording_path
    
def config_set():
    return _config_set

'''
Attempts to read recording.json, if not found returns None
'''
def read_config():
    return au_config.read(_recording_config)

'''
Attempts to read recording.json, if not found creates one
'''
def read_or_create_config(output=True):
    result = au_config.read(_recording_config)
    if result == None:
        print "File '{}' not found".format(_recording_config)
        result = _create_config()
    else:
        print "File '{}' found".format(_recording_config)
        check_result = _check_config(result, output)
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
        _pbw(output,value)
    else:
        _pbp(output,value)
        

#Reads json config file and asks user to enter a value from the values in the config
def _read_config_enter_attrib(name, set_name, json_file, default=None):
    _config = au_config.read(json_file)
    if _config == None:
        print term.RED + "Error reading '{}'".format(json_file) + term.END
    else:
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
    
    rconfig = {}
    
    for value in _config_set:
        rconfig[value] = _enter_attrib(value,_config_set[value][0],_config_set[value][1])
        
    au_config.write(_recording_config,rconfig)
    print "Wrote config to '{}'".format(_recording_config)
    
    return rconfig



#Checks the json config file to see if it contains the necessary values,
#if it doesn't, it writes to the json config file
def _check_config(rconfig, output=True):
    print "Checking config file..."

    changed = False
    for value in _config_set:
        prev_value = rconfig.get(value)
        rconfig[value], valid = _valid_attrib(prev_value,value,_config_set[value][0],_config_set[value][1],output)
        if not valid:
            changed = True
    
    if not changed: #Valid, rconfig is not changed, no need to return rconfig
        return None

    return rconfig #Was invalid, rconfig is updated
    
#Checks to see if this attribute is valid, if it isn't it asks the user for a valid attribute
def _valid_attrib(attrib,name,set_name,json_file,output=True):
    valid = True
    if attrib == None:
        print term.RED + 'Error finding {} in config file'.format(name)
        attrib = _read_config_enter_attrib(name,set_name,json_file)
        valid = False
    if output:
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
    
    changed = False
    for value in _config_set:
        prev_value = rconfig.get(value)
        if prev_value == None:
            rconfig[value], valid = _valid_attrib(prev_value,value,_config_set[value][0],_config_set[value][1])
        else:
            rconfig[value] = _enter_attrib(value,_config_set[value][0],_config_set[value][1],prev_value)
            valid = rconfig[value] == prev_value
        if not valid:
            changed = True
    
    if not changed: #Not edited, rconfig is not changed, no need to return rconfig
        return None

    return rconfig #Was edited, rconfig is updated





def _print2(SA,SB,a,b):
    print SA+a + SB+b + term.END

# Print blue then white text
def _pbw(a,b):
    _print2(term.BLUE,term.END,a,b)
    
# Print blue then purple text
def _pbp(a,b):
    _print2(term.BLUE,term.PURPLE,a,b)
    
    




