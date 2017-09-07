
'''
config.py

This module is used to handle reading and writing config files

Some examples in this module will use the data:

users = {
    'rick' : {
        'first' : 'Rick',
        'last' : 'Grimes',
        'id' : 10
    },
    'jon' : {
        'first' : 'Jon',
        'last' : 'Snow',
        'id' : 20
    },
    'michelle' : {
        'first' : 'Michelle',
        'last' : 'Cloverfield',
        'id' : 30
    }
}

'''


import json


'''
Returns a dict loaded from the given json file

Example:
data = config.read('config.json')
'''
def read(fname):
    try:
        with open(fname, 'r') as read_file:
            data = json.load(read_file)
            return data
    except IOError, err:
        return None


'''
Writes a dict to a file

Example:
config.write('config.json', data)
'''
def write(fname, data):
    with open(fname, 'w') as write_file:
        json.dump(data, write_file, indent=4)


def keylist(data):
    return [i for i in data]

def valuelist(data):
    return [data[i] for i in data]


def query(data, keys):
    if len(keys)==1:
        return data[keys[0]]
    return query(data[keys[0]],keys[1:])

'''

users['jon']['id'] = 20
query_all(users, (None,'id')) -> [ 10, 20, 30 ]
query_all(users, ('jon',None)) -> ['Jon', 'Snow', 20]
'''
def query_all(data, keys):
    if not isinstance(data, dict):
        return None
    if keys[0]==None:
        if len(keys)==1:
            return valuelist(data)
        else:
            result = []
            for e in data:
                sub_query = query_all(data[e],keys[1:])
                if sub_query != None:
                    for sub in sub_query:
                        result.append(sub)
            if len(result)>0:
                return result
    else:
        result = []
        for e in data:
            if e == keys[0]:
                if len(keys)==1:
                    result.append(data[e])
                else:
                    sub_query = query_all(data[e],keys[1:])
                    if sub_query != None:
                        for sub in sub_query:
                            result.append(sub)
        if len(result)>0:
            return result
    return None


'''
Searches for the value and returns a path to query

user = config.search(users,(None,'id'), 20') -> [ 'jon', 'id' ]
assert 20 == config.query(user)
'''
def search(data,keys,value):
    if not isinstance(data, dict):
        return None
    result = []
    for e in data:
        if len(keys) == 1: #Last search
            if keys[0] == None:
                if data[e] == value:
                    return [e]
            else:
                if e == keys[0] and data[e] == value:
                    return [e]
        else: #More to search
            if keys[0] == None or e == keys[0]:
                sub_search = search(data[e],keys[1:],value)
                if sub_search != None:
                    sub_search.insert(0,e)
                    return sub_search
    return None




