
import sys
import operator

import numpy as np
import h5py

import theano
from theano import tensor as T
from theano.ifelse import ifelse
from theano.sandbox.rng_mrg import MRG_RandomStreams

from blocks.model import Model
from blocks.graph import ComputationGraph, apply_batch_normalization, get_batch_normalization_updates
from blocks.algorithms import StepClipping, GradientDescent, CompositeRule, RMSProp, Adam, AdaGrad, AdaDelta, VariableClipping
from blocks.filter import VariableFilter
from blocks.extensions import FinishAfter, Timing, Printing, saveload, ProgressBar
from blocks.extensions.training import SharedVariableModifier
from blocks.extensions.monitoring import DataStreamMonitoring, TrainingDataMonitoring
from blocks.monitoring import aggregation
from blocks.bricks.conv import ConvolutionalSequence, Convolutional
from blocks.bricks import MLP
from blocks.bricks.recurrent.base import BaseRecurrent
from blocks.roles import PARAMETER, FILTER, INPUT
from blocks import roles

from blocks.extensions import predicates, SimpleExtension
from blocks.extensions.training import TrackTheBest
from blocks import main_loop

import arlo.net.vaegan as vaegan
import arlo.net.saver as saver

from train_utils import get_stream, track_best, MainLoop, Dropout, apply_dropout, SetTrainFlag, load_encoder
from train_model import nn_fprop
from train_config import config
locals().update(config)

def run():

    # Load Model
    net_size = 128 #Hard-code instead of loading model (takes too long to set up network)
    #net = vaegan.VAEGAN()
    #network_saver = saver.NetworkSaver('vaegan/models/', net=net)
    #network_saver.load()
    
    with h5py.File(hdf5_file) as f:
        print f
        print dir(f)
        print "\n\n"
        print f.items()
        print f.keys()
        print f.values()
        print "\n\n"
        print f.attrs, dir(f.attrs)
        print "\n\n"
        print f.attrs.items()
        print f.attrs.keys()
        print f.attrs.values()
        print "\n\n"
    
    # Config
    cost_mode = 'RL-MDN'
    input_columns = ['task_dice']
    output_columns = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper', 'steps_to_goal']
    batch_size = 128  # number of samples taken per each update
    nepochs = 5 #50000  # number of full passes through the training data
    learning_rate = .005
    learning_rate_decay = 0.5  # set to 0 to not decay learning rate
    lr_decay_every_n_epochs = 100
    
    
    save_path = 'models/model_save.pkl' #TODO jonathan what is this?
    last_path = 'models/model_last.pkl' #TODO jonathan what is this?
    load_path = save_path
    
    # DATA
    train_stream = get_stream(hdf5_file, 'train', batch_size) #TODO jonathan ?
    test_stream = get_stream(hdf5_file, 'test', batch_size) #TODO jonathan ?

    # MODEL
    x = T.TensorType('floatX', [False] * 3)('features')
    y = T.tensor3('targets', dtype='floatX')
    train_flag = [theano.shared(0)]
    x = x.swapaxes(0,1)
    y = y.swapaxes(0,1)
    
    # More Config
    out_size = len(output_columns) - 1 # code_mode=RL-MDN
    hidden_size = 100
    latent_size = net_size
    in_size = latent_size + len(input_columns)
    layer_models = ['lstm','lstm','lstm']
    num_recurrent_layers = len(layer_models)
    
    
    # Network
    y_hat, cost, cells = nn_fprop(x, y, in_size, out_size, hidden_size, num_recurrent_layers, train_flag)

    # COST
    cg = ComputationGraph(cost)
    extra_updates = []
    
    # RMS Prop training optimizer
    step_rules = [RMSProp(learning_rate=learning_rate, decay_rate=decay_rate), StepClipping(step_clipping)]
    
    parameters_to_update = cg.parameters
    algorithm = GradientDescent(cost=cg.outputs[0], parameters=parameters_to_update,
                                step_rule=CompositeRule(step_rules))
    algorithm.add_updates(extra_updates) # TODO jonathan what is this, is this needed?

    # Extensions
    gradient_norm = aggregation.mean(algorithm.total_gradient_norm)
    step_norm = aggregation.mean(algorithm.total_step_norm)
    monitored_vars = [cost, step_rules[0].learning_rate, gradient_norm, step_norm]

    test_monitor = DataStreamMonitoring(variables=[cost], after_epoch=True,
                                        before_first_epoch=True, data_stream=test_stream, prefix="test")
    train_monitor = TrainingDataMonitoring(variables=monitored_vars, after_epoch=True,
                                           before_first_epoch=True, prefix='train')

    set_train_flag = SetTrainFlag(after_epoch=True, before_epoch=True, flag=train_flag)

    # plot = Plot('Plotting example', channels=[['cost']], after_batch=True, open_browser=True)
    extensions = [set_train_flag, test_monitor, train_monitor, Timing(), Printing(after_epoch=True),
                  FinishAfter(after_n_epochs=nepochs),
                  saveload.Load(load_path),
                  saveload.Checkpoint(last_path,every_n_epochs=10000),
                  ] + track_best('test_cost', save_path) #+ track_best('train_cost', last_path)


    if learning_rate_decay not in (0, 1):
        extensions.append(SharedVariableModifier(step_rules[0].learning_rate,
                                                 lambda n, lr: np.cast[theano.config.floatX](learning_rate_decay * lr),
                                                 after_epoch=False, every_n_epochs=lr_decay_every_n_epochs, after_batch=False))

    print 'number of parameters in the model: ' + str(T.sum([p.size for p in cg.parameters]).eval())
    # Finally build the main loop and train the model
    mainLoop = MainLoop(data_stream=train_stream, algorithm=algorithm,
                         model=Model(cost), extensions=extensions)
    mainLoop.run()
    

    


        

