import main
import arlo.net.saver as saver
import arlo.net.trainer as trainer
import arlo.net.vaegan as vaegan
import numpy as np
from PIL import Image


class VaeganTrainer(object):

    # Initializes network trainer and saver
    def __init__(self):
        # Contains raw data directory
        self._project = main.ConsoleProject().project()

        self._net = vaegan.VAEGAN()

        self._trainer = trainer.Trainer(self._net, seed=1241, epochs=10,
                                        batches=100)
        self._trainer.addInterval(self._save_interval, 100)
        self._trainer.addInterval(self._image_interval, 100)
        self._trainer.addInterval(self._epoch_interval, 100)
        self._trainer.addInterval(self._batch_interval, 1)

        self._network_saver = saver.NetworkSaver('vaegan/models/',
                                                 net=self._net)
        self._image_saver = saver.Saver('vaegan/images/')

        self._network_saver.load()

    # Train method
    def train(self):
        dim = 128  # size

        X = [np.zeros(3*dim*dim, dtype='float32').reshape((3, dim, dim)) for
             i in range(1000)]
        batch = X[0:100]
        c = 100
        rnn_in = np.zeros(c*20*5).reshape(c, 20, 5)
        rnn_out = np.zeros(c*20*5).reshape(c, 20, 5)

        self._trainer.train(x=(batch, rnn_in, rnn_out))

    # Called when network model should save
    def _save_interval(self, epoch, batch, result):
        # self._network_saver.save()
        pass

    # Called when image should save
    def _image_interval(self, epoch, batch, result):
        img_batch = []
        # image = self._net.image(img_batch)

        def save_image(path):
            Image.fromarray(image).save(
                path + '{1:03d}_{2:07d}.png'.format(epoch, batch))

        # self._image_saver.save(save_image)

    # Called every epoch
    def _epoch_interval(self, epoch, batch, result):
        print "Epoch: {} - Batch: {}".format(epoch, batch)

    # Called every batch
    def _batch_interval(self, epoch, batch, result):
        loss_enc, loss_gen, loss_dis, loss_rec, loss_rnn = result

        batch_size = self._trainer.batches()

        sys.stdout.write('\r' + str(i/batch_size) +
                         'errors: {0:0.4f} {1:0.4f} {2:0.8f} {3:0.4f} {4:0.4f}'
                         .format(loss_enc, loss_gen, loss_dis, loss_rnn,
                                 loss_rec))
        sys.stdout.flush()
