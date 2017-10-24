
import main

import arlo.net.saver as saver
import arlo.net.trainer as trainer
import arlo.net.vaegan as vaegan

import cv2
import sys
import numpy as np
import random as random
from PIL import Image
from scipy.misc import imresize

class VaeganTrainer(object):

    # Initializes network trainer and saver
    def __init__(self):
        # Contains raw data directory
        self._project = main.ConsoleProject().project()
        
        self._net = vaegan.VAEGAN()

        self._epoch = 50
        self._batch = 10
        self._batch_size = 20
        self._image_dim = (5,4)

        self._trainer = trainer.Trainer(self._net, seed=1241, epochs=self._epoch,
                                        batches=self._batch)
        self._trainer.addInterval(self._save_interval, self._batch)
        self._trainer.addInterval(self._image_interval, self._batch)
        self._trainer.addInterval(self._epoch_interval, self._batch)
        self._trainer.addInterval(self._batch_interval, 1)

        self._network_saver = saver.NetworkSaver('vaegan/models/',
                                                 net=self._net)
        self._image_saver = saver.Saver('vaegan/images/')

        self._network_saver.load()
        

    def batch_func(self,i,j):
    
        entry_count = self._project.entryCount()
        random_index = int(random.random()*entry_count)
        entry = self._project.entry(random_index)
        video = entry.get('video_file','video_numpy')
        
        # Get random sample from shuffled video images
        np.random.shuffle(video)
        sample = video[:self._batch_size]
        
        # Resize
        data = []
        for i in range(len(sample)):
            data.append(cv2.resize(sample[i],(128,128)))
        data = np.array(data, dtype='float32')
        data = data / 127.5 - 1;
        
        # Swap axes
        data = data.swapaxes(1,3)
        data = data.swapaxes(2,3)
        
        
        return data

    # Train method
    def train(self):
        self._trainer.train(self.batch_func)
        pass

    # Called when network model should save
    def _save_interval(self, epoch, batch, result):
        # self._network_saver.save()
        pass

    # Called when image should save
    def _image_interval(self, epoch, batch, result):
        
        data = self.batch_func(0,0)
        
        image = self._net.image(data, self._image_dim)
        
        def save_image(path):
            Image.fromarray(image).save(
                path + '{0:03d}_{1:07d}.png'.format(epoch, batch))

        self._image_saver.save(save_image)

    # Called every epoch
    def _epoch_interval(self, epoch, batch, result):
        print "\nEpoch: {} / {}".format(epoch+1, self._epoch)

    # Called every batch
    def _batch_interval(self, epoch, batch, result):
        loss_enc, loss_gen, loss_dis, loss_rec = result

        batch_size = self._trainer.batches()

        sys.stdout.write('\r'
                         'Batch: ' + str(batch+1) + ' / ' + str(self._batch) + ' -  errors: {0:0.4f} {1:0.4f} {2:0.8f} {3:0.4f}'
                         .format(loss_enc, loss_gen, loss_dis,loss_rec))
        sys.stdout.flush()
        

