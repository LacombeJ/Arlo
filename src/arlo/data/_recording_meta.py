
'''
_recording_meta.py

This module contains private utility functions to handle recording meta files

'''


import arlo.utils.config as au_config
import arlo.utils.term as term



'''
Attempts to read meta file, if not found returns None
'''
def read_meta(meta_path):
    return au_config.read(meta_path)


'''
Attempts to read meta file, if not found creates one
'''
def read_or_create_meta(meta,meta_path):
    result = au_config.read(meta_path)
    if result == None:
        result = meta
        au_config.write(meta_path,meta)
    else:
        result, changed = _recursive_fill(result,meta)
        if changed:
            au_config.write(meta_path,result)
    return result

'''
Writes meta file directly without checking if valid
'''
def write_meta_unsafe(meta,meta_path):
    au_config.write(meta_path,meta)


'''
Recursively fills the data dict to match the default structure but will
not override entries if different.
Returns data and true if data was changed, false otherwise
'''
def _recursive_fill(data,default):
    if not isinstance(data,dict):
        if isinstance(default,dict):
            return default, True
        return data, False
    changed = False
    for d in default:
        find = data.get(d)
        if find == None:
            data[d] = default[d]
            changed = True
        else:
            data[d], c = _recursive_fill(data[d],default[d])
            changed = c or changed
    return data, changed





