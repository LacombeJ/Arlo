#!/usr/bin/env python

import theano
import numpy as np
import sys
import cv2
import h5py
from fuel.datasets import H5PYDataset

import os
import pandas as pd

from PIL import Image
#np.random.seed(0)

import arlo.net.vaegan as vaegan
import arlo.net.saver as saver

from arlo.train_config import config
locals().update(config)

def encode_image(net, images):

    z, mean, var = net.encode(images)
    
    return mean

def getConvFeatures(net, data_in, img_data):
    n_batch = 100
    code_size = net.get_size()
    img_features = np.empty((len(data_in), code_size), dtype=theano.config.floatX)
    for i in xrange(len(data_in)/n_batch+1):
        sys.stdout.write('\r' + str(i) + '/' + str(len(data_in)/n_batch))
        sys.stdout.flush()  # important
        start = i*n_batch
        end = min((i+1)*n_batch,len(data_in))
        if start >= end:
            break
        images = img_data[start:end]
        print images.shape
        img_features[start:end] = encode_image(net, images)
        
    data_in = np.column_stack((img_features, data_in))
    return data_in, data_in.shape[1]


def main():
    
    net = vaegan.VAEGAN()
    network_saver = saver.NetworkSaver('vaegan/models/', net=net)
    network_saver.load()
    
    # Configs ---------------------------------------------------------------------------------------- #
    
    input_columns = ['task0']
    output_columns = ['joint1', 'joint2', 'joint3', 'joint4','joint5','gripper','steps_to_goal']
    seq_length = 50  # number of chars in the sequence
    seq_redundancy = 50
    future_predictions = [1]
    hdf5_file = 'input.hdf5'
    
    # ------------------------------------------------------------------------------------------------ #
    
    img_data = np.load('img_data.npy', mmap_mode='r')
    data_out = np.load('data_out.npy', mmap_mode='r')
    data_in = np.load('data_in.npy', mmap_mode='r')
    
    data_in = np.array(data_in)
    data_out = np.array(data_out)
    
    print type(img_data), img_data.shape
    print type(data_in), data_in.shape
    print type(data_out), data_out.shape
    
    data_in, in_size = getConvFeatures(net, data_in, img_data)
    
    out_size = len(output_columns)
    max_prediction = max(future_predictions) + 1
    if len(data_in) % seq_length > 0:
        data_in = data_in[:len(data_in) - len(data_in) % seq_length + max_prediction]
    else:
        data_in = data_in[:len(data_in) - seq_length + max_prediction]
    nsamples = (len(data_in) / seq_redundancy)
    print 'Saving data to disc...'
    inputs = np.memmap('inputs.npy', dtype=theano.config.floatX, mode='w+', shape=(nsamples, seq_length, in_size))
    outputs = np.memmap('outputs.npy', dtype=theano.config.floatX, mode='w+',
                        shape=(nsamples, seq_length, len(future_predictions) * out_size))
    
    for i, p in enumerate(xrange(0, len(data_in) - max_prediction - seq_length, seq_redundancy)):
        inputs[i] = np.array([d for d in data_in[p:p + seq_length]])
        for j in xrange(len(future_predictions)):
            outputs[i, :, j * out_size:(j + 1) * out_size] = np.array(
                [d for d in data_out[p + future_predictions[j]:p + seq_length + future_predictions[j]]])

    nsamples = len(inputs)
    nsamples_train = train_data_size // seq_length

    print np.isnan(np.sum(inputs))
    print np.isnan(np.sum(outputs))

    f = h5py.File(hdf5_file, mode='w')
    features = f.create_dataset('features', inputs.shape, dtype=theano.config.floatX)
    targets = f.create_dataset('targets', outputs.shape, dtype=theano.config.floatX)

    features[...] = inputs
    targets[...] = outputs
    features.dims[0].label = 'batch'
    features.dims[1].label = 'sequence'
    features.dims[2].label = 'features'
    targets.dims[0].label = 'batch'
    targets.dims[1].label = 'sequence'
    targets.dims[2].label = 'outputs'
    split_dict = {
        'train': {'features': (0, nsamples_train), 'targets': (0, nsamples_train)},
        'test': {'features': (nsamples_train, nsamples), 'targets': (nsamples_train, nsamples)}}
    f.attrs['split'] = H5PYDataset.create_split_array(split_dict)
    f.flush()
    f.close()
    
    print nsamples_train, nsamples, train_data_size, seq_length
    print 'inputs shape:', inputs.shape
    print 'outputs shape:', outputs.shape
    print 'image inputs shape:', img_data.shape
    os.remove(inputs.filename)
    os.remove(outputs.filename)
    print 'Files saved on disc for Blocks!'

if __name__ == "__main__":
    main()
    



