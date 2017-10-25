import numpy as np
import arlo.data.node as node
import arlo.utils.ext as ext


# Constants
META_LABEL = 'meta_type'
META = 'saver'

VERSION_LABEL = 'version'
VERSION = '1.0.0'

SUB_LABEL = 'sub'
SUB_SAVER = 'save'
SUB_NETWORK = 'model'

LABEL_LABEL = 'label'
LABEL_SAVER = 'saver'
LABEL_NETWORK = 'network'

INCREMENT = 'increment'
DIRECTORIES = 'directories'


class Saver(object):

    def __init__(self, path, label=LABEL_SAVER, sub=SUB_SAVER):

        self._label = label

        self._node, new = node.load(path, translate)
        if new:
            self._node.set(META_LABEL, META)
            self._node.set(VERSION_LABEL, VERSION)
            self._node.set(SUB_LABEL, SUB_SAVER)
            self._node.set(LABEL_LABEL, self._label)

            self._node.set(INCREMENT, 0)
            self._node.set(DIRECTORIES, [])

            self._node.save()
        else:
            if self._node.get(META_LABEL) != META:
                print "Error loading saver: Wrong meta type"
                self._node = None
            if self._node.get(VERSION_LABEL) != VERSION:
                print "Error loading saver: Wrong version"
                self._node = None
            if self._node.get(LABEL_LABEL) != self._label:
                print "Error loading saver: Wrong label"
                self._node = None

    def save(self, func):
        func(self._node.path())


# This class saves and loads neural network weights
class NetworkSaver(Saver):
    # net neural network
    # path save path
    # single if there should be one single saved instance on this model
    def __init__(self, path, net, single=False, label=LABEL_NETWORK,
                 sub=SUB_NETWORK):
        Saver.__init__(self, path, label, sub)
        self._net = net
        self._single = single

    def save(self,save_func=None):

        increment = self._node.get(INCREMENT)
        directories = self._node.get(DIRECTORIES)

        sub, new = self._node.load('{}{}'.format(SUB_NETWORK, increment))
        sub_path = sub.path()
        sub_data = {}
        
        self._net.save(sub_path)
        
        if save_func is not None:
            save_func(sub_path,sub_data)
            self._node.setValues(sub_data)
            self._node.save()
        
        if not self._single or new:
            increment += 1
            directories.append(sub.relative_path())
            self._node.set(INCREMENT, increment)
            self._node.set(DIRECTORIES, directories)
            self._node.save()

    def load(self):

        directories = self._node.get(DIRECTORIES)

        if len(directories) > 0:

            last_directory = directories[-1]

            sub, new = self._node.load('{}'.format(last_directory))
            sub_path = sub.path()

            if new:
                print "Load failed, network model does not exist."
                sub.unsafe_erase()
                return

            self._net.load(sub_path)


def translate(path, otype, value):
    if otype == 'np':
        return None  # TODO np memmap array?
    if otype == 'datetime':
        return ext.unpack_datetime(value)
    return None
    

