
import network
import models

import sys
import signal

import numpy as np
import cupy as cp

import chainer
import chainer.functions as F
import chainer.links as L
from chainer import cuda, Variable, optimizers, serializers

# TODO jonathan comment sections
# TODO jonathan add logger support

class VAEGAN(network.Network):

    batch_size = 100  # if you use something else, let me know, but batch size needs to be somewhere

    # Initializes the network
    # size image size, one of [48, 64, 80, 96, 112, 128]
    # I believe we should stick with 128
    def __init__(self, size=128):
        self._image_size = size

        self._init_parameters()
        self._init_model()

    # Initializes network parameters
    def _init_parameters(self):
        self._latent_size = 256
        self._gpu = 0
        
        self._train_dis = True

        self._normer = self._image_size * self._image_size * 3 * 60

        signal.signal(signal.SIGINT, self._signal_handler)
        # needed to signal for signal handler, limits time


    # Initializes the model for this network
    def _init_model(self):

        self._enc_model = models.Encoder(density=8, size=self._image_size,
                                         latent_size=self._latent_size)
        self._gen_model = models.Generator(density=8, size=self._image_size,
                                         latent_size=self._latent_size)
        self._dis_model = models.Discriminator(density=8, size=self._image_size)

        self._enc_dis_model = models.Encoder(density=8, size=self._image_size,
                                             latent_size=self._latent_size)
        self._gen_dis_model = models.Generator(density=8, size=self._image_size,
                                               latent_size=self._latent_size)

        self._optimizer_enc = optimizers.Adam(alpha=0.0001, beta1=0.5)
        self._optimizer_enc.setup(self._enc_model)
        self._optimizer_enc.add_hook(chainer.optimizer.WeightDecay(0.00001))
        self._optimizer_gen = optimizers.Adam(alpha=0.0001, beta1=0.5)
        self._optimizer_gen.setup(self._gen_model)
        self._optimizer_gen.add_hook(chainer.optimizer.WeightDecay(0.00001))

        self._optimizer_enc_dis = optimizers.Adam(alpha=0.0001, beta1=0.5)
        self._optimizer_enc_dis.setup(self._enc_dis_model)
        self._optimizer_enc_dis.add_hook(chainer.optimizer.WeightDecay(0.00001))
        self._optimizer_gen_dis = optimizers.Adam(alpha=0.0001, beta1=0.5)
        self._optimizer_gen_dis.setup(self._gen_dis_model)
        self._optimizer_gen_dis.add_hook(chainer.optimizer.WeightDecay(0.00001))

        self._optimizer_dis = optimizers.Adam(alpha=0.0001, beta1=0.5)
        self._optimizer_dis.setup(self._dis_model)
        self._optimizer_dis.add_hook(chainer.optimizer.WeightDecay(0.00001))

        self._enc_model.to_gpu(self._gpu)  # send to main GPU
        self._gen_model.to_gpu(self._gpu)
        self._dis_model.to_gpu(self._gpu)

    # Loads model weights into this network from the given path
    def load(self, path):
        serializers.load_hdf5(path + 'enc.model', self._enc_model)
        serializers.load_hdf5(path + 'enc.state', self._optimizer_enc)
        serializers.load_hdf5(path + 'gen.model', self._gen_model)
        serializers.load_hdf5(path + 'gen.state', self._optimizer_gen)

        serializers.load_hdf5(path + 'dis.model', self._dis_model)
        serializers.load_hdf5(path + 'dis.state', self._optimizer_dis)

    # Saves model weights from this network to the given path
    def save(self, path):

        # needs to follow the save interval as well!

        serializers.save_hdf5(path + 'enc.model', self._enc_model)
        serializers.save_hdf5(path + 'enc.state', self._optimizer_enc)
        serializers.save_hdf5(path + 'gen.model', self._gen_model)
        serializers.save_hdf5(path + 'gen.state', self._optimizer_gen)

        serializers.save_hdf5(path + 'dis.model', self._dis_model)
        serializers.save_hdf5(path + 'dis.state', self._optimizer_dis)


    # Trains this network given inputs (img_batch)
    # Returns (loss_enc loss_gen, loss_dis, loss_reconstruction)
    def train(self, x):
        img_batch = x

        loss_enc = 0
        
        # Train on GPU
        # ----------------------------------------------------------------------- #

        cuda.get_device(self._gpu).use()
        
        img_batch_for_gpu = img_batch[:]
        
        # TODO is this okay (check original train.py for reference)
        
        gpu_batch_size = len(img_batch_for_gpu)
        
        # encode
        x_in = cuda.to_gpu(img_batch_for_gpu, self._gpu)
        z0, mean, var = self._enc_model(Variable(x_in))
        x0 = self._gen_model(z0)
        loss_reconstruction = F.mean_squared_error(x0, x_in)
        y0, l0 = self._dis_model(x0)
        
        l_dis_rec = F.softmax_cross_entropy(y0,
            Variable(cuda.to_gpu(cp.zeros(gpu_batch_size).astype(np.int32),
                self._gpu))) / gpu_batch_size
        z1 = Variable(cuda.to_gpu(cp.random.normal(0, 1, (gpu_batch_size,
                self._latent_size), dtype=np.float32), self._gpu))
                
        x1 = self._gen_model(z1)
        y1, l1 = self._dis_model(x1)
        l_prior = F.gaussian_kl_divergence(mean, var) / (self._normer)
        
        l_dis_fake = F.softmax_cross_entropy(y1,
            Variable(cuda.to_gpu(cp.zeros(gpu_batch_size).astype(np.int32),
                self._gpu))) / gpu_batch_size

        # train discriminator
        y2, l2 = self._dis_model(Variable(x_in))
        l_dis_real = F.softmax_cross_entropy(y2,
            Variable(cuda.to_gpu(cp.ones(gpu_batch_size).astype(np.int32),
                self._gpu))) / gpu_batch_size
        l_feature_similarity = F.mean_squared_error(l0, l2)
        # * l2.data.shape[2] * l2.data.shape[3]
        
        l_dis_sum = (l_dis_real + l_dis_fake + l_dis_rec) / 3
        loss_enc = l_prior + l_feature_similarity
        loss_gen = l_feature_similarity - l_dis_sum
        loss_dis = l_dis_sum

        self._enc_model.cleargrads()
        loss_enc.backward()

        self._gen_model.cleargrads()
        loss_gen.backward()

        if self._train_dis:
            self._dis_model.cleargrads()
            loss_dis.backward()
        
        # ----------------------------------------------------------------------- #
        
        self._optimizer_enc.update()
        self._optimizer_gen.update()

        if self._train_dis:
            self._optimizer_dis.update()

        self._train_dis = float(loss_dis.data) > 0.0001

        return (float(loss_enc.data), float(loss_gen.data),
            float(loss_dis.data), float(loss_reconstruction.data))
            
    # System exit signal handler
    def _signal_handler(self, signal, frame):
        sys.exit(0)
        
    # Return latent size
    def get_size(self):
        return self._latent_size

    # Encodes image batch
    def encode(self,batch):
        with chainer.using_config('train', False), chainer.using_config('enable_backprop', False):
            cuda.get_device(self._gpu).use()
            x_in = cp.asarray(batch)
            z, mean, var = self._enc_model(Variable(x_in), train=False)
            return (cuda.to_cpu(z.data), cuda.to_cpu(mean.data), cuda.to_cpu(var.data))
    
    # Decodes z data into image batch
    def decode(self,z):
        with chainer.using_config('train', False), chainer.using_config('enable_backprop', False):
            cuda.get_device(self._gpu).use()
            z_in = cp.asarray(z)
            x = self._gen_model(Variable(z_in), train=False)
            return ((cuda.to_cpu(x.data) + 1) * 128).clip(0, 255).astype(np.uint8)

    # Encodes then decodes batch
    def reconstruct(self, batch):
        z,m,v = self.encode(batch)
        return self.decode(z)

    # Returns encoded image from the image batch ? TODO
    def image(self, image_batch):
        
        dim = len(image_batch)
        
        with chainer.using_config('train', False), chainer.using_config('enable_backprop', False):

            cuda.get_device(self._gpu).use()
            z, m, v = self._enc_model(Variable(cuda.to_gpu(image_batch, self._gpu)), train=False)
            z = m
            data = self._gen_model(z, train=False).data

            reconstructed_images = ((cuda.to_cpu(data) + 1) * 128).clip(0, 255).astype(np.uint8)
            reconstructed_images = reconstructed_images[:dim]
            
            original_images = ((image_batch+1)*128).clip(0,255).astype(np.uint8)
            image = np.concatenate((original_images,reconstructed_images))
            
            # Image shape = ( N, 3, _image_size, _image_size)
            # To switch from BGR to RGB color channel:
            image = image[ : , ::-1 , : , : ]
            
            # The row=2 in reshape represents the original and reconstruct coupled images
            row = 2
            image = image.reshape((row, dim, 3, self._image_size, self._image_size))
            image = image.transpose((0, 3, 1, 4, 2))
            image = image.reshape((row * self._image_size, dim * self._image_size, 3))

            return image
            
    
