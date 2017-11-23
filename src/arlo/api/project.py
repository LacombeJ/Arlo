# project.py
# This module contains functions for loading and managing projects
import module
import _util as util
import os
import cv2
import arlo.data.node as node
import arlo.utils.config as config
import arlo.utils.ext as ext
import arlo.utils.log as log

# Constants
META_LABEL = 'meta_type'
META = 'arlo_project'
VERSION_LABEL = 'version'
VERSION = '1.0.0'
ENTRY_LABEL = 'entry'
ENTRY = 'data'
INCREMENT = 'data_increment'
DIRECTORIES = 'data_directories'


def load(path, translate=None):
    root, new = node.load(path, util._addTranslator(translate))
    if new:
        root.set(META_LABEL,    META)
        root.set(VERSION_LABEL, VERSION)
        root.set(ENTRY_LABEL,   ENTRY)
        root.set(INCREMENT,     0)
        root.set(DIRECTORIES,   [])
        root.save()
    else:
        if root.get(META_LABEL) != META:
            print "Error loading project: Wrong meta type"
            return None
        if root.get(VERSION_LABEL) != VERSION:
            print "Error loading project: Wrong version"
            return None
    return Project(root)


class ProjectNode(object):

    def __init__(self, project, node):
        self._project = project
        self._node = node
        self._name = self._node.directory_name()

    def path(self):
        return self._node.absolute_path()

    def relative_path(self):
        return self._node.path()
        
    def fileExists(self, fname):
        return self._node.fileExists(fname)

    def name(self):
        return self._name

    def get(self, key, otype=None):
        return self._node.get(key, otype)

    def set(self, key, value):
        self._node.set(key, value)

    def save(self):
        self._node.save()

#TODO jonathan rename functions / refactor code
class Project(ProjectNode):

    # ----------------------------------------------- #

    def __init__(self, root):
        ProjectNode.__init__(self, self, root)
        self._refresh()
        self._logger = log.Logger()

    def _refresh(self):
        self._data_dir = self._node.get(DIRECTORIES)
        self._entry_count = len(self._data_dir)

    # ----------------------------------------------- #

    def entryCount(self):
        return self._entry_count

    def entry(self, index):
        sub_name = self._data_dir[index]
        sub, new = self._node.load(sub_name)
        if new:
            sub.unsafe_erase()
            return None
        return Entry(self, sub)

    def getEntry(self, name):
        sub, new = self._node.load(name)
        if new:
            sub.unsafe_erase()
            return None
        return Entry(self, sub)

    # def func(entry)
    def entries(self, func):
        for i in range(self._entry_count):
            e = self.entry(i)
            func(e)

    def getEntries(self, filter_func=None):
        entries = []
        for i in range(self._entry_count):
            e = self.entry(i)
            if filter_func is None:
                entries.append(e)
            elif filter_func(e):
                entries.append(e)
        return entries

    def deleteEntryAt(self, index):
        E = self.entry(index)
        if E is not None:
            E.delete()

    def deleteEntry(self, name):
        E = self.getEntry(name)
        if E is not None:
            E.delete()

    def getLogger(self):
        return self._logger

    def setLogger(self, logger):
        self._logger = logger

    # ----------------------------------------------- #

    def record(self, modules, require=True):
        return util.record(self, modules, require)

    def playback(self, modules, index=-1, require=True):
        return util.playback(self, modules, index, require)

    # ----------------------------------------------- #


class Entry(ProjectNode):

    def __init__(self, project, sub):
        ProjectNode.__init__(self, project, sub)
        
    def delete(self):
        self._node.delete()
        directories = self._project._node.get(DIRECTORIES)
        directories.remove(self.name()+'/')
        self._project._node.set(DIRECTORIES, directories)
        self._project._node.save()
        self._project._refresh()
        

