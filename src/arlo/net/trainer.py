import numpy as np

# This class trains a neural network by calling its partial_fit function
# This class handles running train in batches and epochs and shuffling data


class Trainer(object):

    def __init__(self, net, epochs=100, batches=100, seed=None):

        self._net = net
        self._epochs = epochs
        self._batches = batches
        self._intervals = []  # array of (func, interval)
        if seed is not None:
            np.random.seed(seed)

    def epochs(self):
        return self._epochs

    def batches(self):
        return self._batches

    def addInterval(self, func, interval):
        self._intervals.append((func, interval))

    def train(self, x):

        for i in range(self._epochs):

            for j in range(self._batches):
                train_result = self._net.train(x)

                for (func, interval) in self._intervals:

                    if j % interval == 0:
                        func(i, j, train_result)
