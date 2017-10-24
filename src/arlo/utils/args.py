
# args.py
# This module is used to handle system arguments
import sys


_sys_args = sys.argv[1:]

_checked = [False for a in _sys_args]


# Gets the name of the main script, sys.argv[0]
def getScript():
    return sys.argv[0]

# Gets the argument at the given index, sys.argv[index-1]
def get(index):
    return _sys_args[index]
    
# Gets the argument at the given index or return None
def get_arg(index):
    if index < len(_sys_args):
        return get(index)
    return None

# Checks if arguments meet the minimuim number and prints an error, quits if exit==True
def check_minimum(num_args, error=None, exit=True):
    if len(_sys_args) < num_args:
        if error == None:
            error='Invalid number of arguments - {} minimum, {} given'.format(num_args, len(_sys_args))
        print error
        if exit:
            quit()

# Checks if arguments meet the minimuim number and prints an error, quits if exit==True
def check_maximum(num_args, error=None, exit=True):
    if len(_sys_args) > num_args:
        if error == None:
            error='Invalid number of arguments - {} maximum, {} given'.format(num_args, len(_sys_args))
        print error
        if exit:
            quit()

# Checks if arguments contain the given argument and returns true if it does
def check(arg):
    result = False
    for i, a in enumerate(_sys_args):
        if a == arg:
            _checked[i] = True
            result = True
    return result

# Checks the argument
# Returns a tuple (check,index)
#   Check = None if the argument was found, but no argument after was found
#   Check = False if the argument was not found
#   Check = the argument after the given argument, if the given argument is found
#   Index = index of given argument if the argument was found
#   Index = -1 if the argument was not found
def check_after(arg):
    for i, a in enumerate(_sys_args):
        if a == arg:
            _checked[i] = True
            if i < len(_sys_args)-1:
                _checked[i+1] = True
                return _sys_args[i+1], i
            else:
                return None, i
    return False, -1

# Return true if argument is preceded by hyphen
def is_arg(arg):
    return arg[0]=='-'
    
# Return true if the argument at the given index was checked
def is_checked(index):
    return _checked[index]

# Prints an error saying that argument is invalid, quits if exit==True
def invalid_arg(arg_num, error=None, exit=True):
    if error == None:
        error = 'Invalid argument - #{}'.format(arg_num)
    print error
    if exit:
        quit()

# Looks to see if there is any argument not checked and prints an error string for each
# error_func(int, int) -> string
def check_invalid_args(error=None, exit=True):
    invalid = False
    if error == None:
        def _error(arg_num,arg):
            return "Invalid argument #{} - '{}'".format(arg_num, get(arg_num))
        error = _error
    for i, a in enumerate(_sys_args):
        if not _checked[i]:
            invalid = True
            print error(i, a)
    if exit and invalid:
        quit()
