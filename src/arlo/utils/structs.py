# Python Structs
# Version: 1.00
# Jonathan Lacombe

import collections

def bimap():
    return _BijectiveMap()


class _BijectiveMap(collections.MutableMapping):
    
    def __init__(self, keyMap=None, valueMap=None):
        self._keyMap = keyMap  # map[key] = value
        self._valueMap = valueMap  # map[value] = key
        if self._keyMap is None: self._keyMap = {}
        if self._valueMap is None: self._valueMap = {}

    def __getitem__(self, key):
        return self._keyMap[key]

    def __setitem__(self, key, value):
        try:
            self._keyMap.pop(self._valueMap.pop(value))
        except KeyError, e:
            pass
        self._keyMap[key] = value
        self._valueMap[value] = key

    def __delitem__(self, key):
        value = self._keyMap[key]
        del self._keyMap[key]
        del self._valueMap[value]

    def __iter__(self):
        return self._keyMap.__iter__()

    def __len__(self):
        return self._keyMap.__len__()

    def flip(self):
        k = self._keyMap
        v = self._valueMap
        return _BijectiveMap(keyMap=v, valueMap=k)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return self._keyMap.__str__()
