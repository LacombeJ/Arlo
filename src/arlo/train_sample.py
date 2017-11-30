import numpy as np
import theano
import theano.d3viz as d3v
from theano import tensor
from blocks import roles
from blocks.roles import OUTPUT
from blocks.model import Model
from blocks.extensions import saveload
from blocks.filter import VariableFilter

import argparse
import sys
import os
import pandas as pd
import time
import signal

from pandas.parser import CParserError
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from numpy import dtype
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import PIL
import scipy

import arlo.net.vaegan as vaegan
import arlo.net.saver as saver
import arlo.data.node as node
import arlo.utils.ext as ext
import arlo.output.al5d as al5d

from train_utils import MainLoop
from train_config import config
from train_model import nn_fprop
from train_utils import pre_process_image #, load_encoder, encode_image, decode_image

locals().update(config)
sceneStateFile = os.path.abspath("predictions/sceneState")

def load_encoder(net):
    return None, net.get_size()
    

def encode_image(net, images):
    z, mean, var = net.encode(images)
    return mean

def decode_image(net, z):
    x = net.decode(z)
    return x

def arlo_crop(data):
    
    x = 75
    y = 25
    w = 400
    h = 400
    data = data[y:y+h, x:x+w]
    
    return data
    

def arlo_preprocess(data):

    data = np.array(data, dtype='float32')
    data = cv2.resize(data,(128,128))
    #data = cv2.flip(data, 1)
    data = data / 127.5 - 1;
    data = data.swapaxes(0,2)
    data = data.swapaxes(1,2)

    return data


class ImageFrame(object):

    # Returns image
    def get(self):
        return None
        
        
class WebcamFrame(object):

    def __init__(self):
        self._cap = cv2.VideoCapture(0)

        if self._cap is None:
            print "Webcam not found"

        if not self._cap.isOpened():
            print "Webcam not opened"
            
    def get(self):
        ret, frame = self._cap.read()
        if ret is False:
            return None
        return frame
        
    def finish(self):
        self._cap.release()
        
        
class VideoFrame(object):

    def __init__(self):
        self._cap = cv2.VideoCapture('video.avi')

        if self._cap is None:
            print "Video not found"

        if not self._cap.isOpened():
            print "Video not opened"
            
    def get(self):
        ret, frame = self._cap.read()
        if ret is False:
            return None
        return frame
        
    def finish(self):
        self._cap.release()


def load_models(net, model_path=save_path, in_size=len(input_columns),
                out_size=len(output_columns) - 1 if cost_mode == 'RL-MDN' else len(output_columns),
                hidden_size=hidden_size, num_recurrent_layers=num_recurrent_layers, model=layer_models[0]):
    
    initials = []
    if not os.path.isfile(model_path):
        print 'Could not find model file.'
        sys.exit(0)
    print 'Loading model from {0}...'.format(model_path)
    x = tensor.tensor3('features', dtype=theano.config.floatX)
    y = tensor.tensor3('targets', dtype='floatX')
    train_flag = [theano.shared(0)]
    
    latent_size = net.get_size() # latent_size
    
    in_size = latent_size + len(input_columns)
    y_hat, cost, cells = nn_fprop(x, y, in_size, out_size, hidden_size, num_recurrent_layers, train_flag)
    main_loop = MainLoop(algorithm=None, data_stream=None, model=Model(cost),
                         extensions=[saveload.Load(model_path)])
    for extension in main_loop.extensions:
        extension.main_loop = main_loop
    main_loop._run_extensions('before_training')
    bin_model = main_loop.model
    print 'Model loaded. Building prediction function...'
    hiddens = []
    for i in range(num_recurrent_layers):
        brick = [b for b in bin_model.get_top_bricks() if b.name == layer_models[i] + str(i)][0]
        hiddens.extend(VariableFilter(theano_name=brick.name + '_apply_states')(bin_model.variables))
        hiddens.extend(VariableFilter(theano_name=brick.name + '_apply_cells')(cells))
        initials.extend(VariableFilter(roles=[roles.INITIAL_STATE])(brick.parameters))
    predict_func = theano.function([x], hiddens + [y_hat])
    encoder, code_size = load_encoder(net)
    return predict_func, initials, encoder, code_size


def predict_one_timestep(net, imageFrame, predict_func, encoder, code_size, initials, x, out_size, iteration):
    
    image = imageFrame.get()
    
    #TODO remove this for webcam capture
    for i in range(4):
        imageFrame.get()
    
    image = arlo_crop(image)
    
    cv2.imshow('Input image', image)
    cv2.waitKey(10)
    
    current_scene_image = arlo_preprocess(image)
    
    current_scene_image = np.array(current_scene_image, dtype=np.float32)
    images = np.array([current_scene_image])
    encoded_images= encode_image(net, images)
    decoded_images = decode_image(net, encoded_images)
    cv2.imshow('Reconstructed image', cv2.resize(np.array(decoded_images[0].transpose((1, 2, 0)))[...,::-1], (0,0), fx=4, fy=4, interpolation=cv2.INTER_NEAREST ))
    cv2.waitKey(10)
    x = np.concatenate([encoded_images[0], x])
    newinitials = predict_func([[x]])
    raw_prediction = newinitials.pop().astype(theano.config.floatX)
    if single_dim_out:
        predicted_values = raw_prediction[:, -1, -1].astype(theano.config.floatX).reshape((len(raw_prediction),))
    else:
        predicted_values = raw_prediction[-1, -1, :].astype(theano.config.floatX)
    layer = 0
    for initial, newinitial in zip(initials, newinitials):
        if iteration % layer_resolutions[layer // 2] == 0:
            initial.set_value(newinitial[-1].flatten())
        layer += (2 if layer_models[layer // 2] == 'mt_rnn' else 1)
        layer = min([layer, len(layer_resolutions)])
    return predicted_values, newinitials


def set_task_column_to_one_hot(data):
    if config['multi_task_mode'] == 'ID':
        for i in config['game_tasks']:
            data['task' + str(i)] = 0
            data.loc[data['task'] == i, 'task' + str(i)] = 1
    return data

def plot_arrays(arrays, title='image'):
    images = []
    for i in range(int(len(arrays)/8)):
        images.append(np.hstack(arrays[i*8:(i+1)*8]))
    images = np.vstack(images)
    vis = cv2.cvtColor(np.array(images, np.float32), cv2.COLOR_GRAY2BGR)
    vis = cv2.resize(vis, (0,0), fx=5, fy=5, interpolation=cv2.INTER_NEAREST )
    cv2.imshow(title, vis)
    cv2.waitKey(10)

def toTrajectories(predicted, pos):
    array = [float(i) for i in predicted]
    for i in range(len(array)):
        if array[i] < 0.0:
            array[i] = 0.0
        if array[i] > 1.0:
            array[i] = 1.0
    array = np.array(array, np.float32)
    
    MIN = 600
    MAX = 2400
    array *= (MAX - MIN)
    array += MIN
    array = np.array(array, np.int32)
    array = [array[1], array[2], array[3], array[4], array[5], array[0]]
    traj = [pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]]
    traj[0] = array[0]
    traj[1] = array[1]
    traj[2] = array[2]
    traj[3] = array[3]
    traj[4] = array[4]
    traj[5] = array[5]
    return traj
    
def moveTowards(array, towards, max_step=100):
    move = [i for i in array]
    for i in range(len(array)):
        diff = towards[i] - array[i]
        if abs(diff) < max_step:
            move[i] = towards[i]
        else:
            s = 1 if diff > 0 else -1
            move[i] = array[i] + s * max_step
    return move

def sample():

    net = vaegan.VAEGAN()
    network_saver = saver.NetworkSaver('vaegan/models/', net=net)
    network_saver.load()

    #imageFrame = WebcamFrame()
    imageFrame = VideoFrame()

    arm = al5d.RobotArm()
    err = arm.create()
    if err is not None:
        print "Al5d arm failed to initialize"

    if plot_hidden_states:
        plt.ion()
        plt.ylim([-2, +4])
        plt.show()
    predict_func, initials, encoder, code_size = load_models(net)
    print("Generating trajectory...")
    last_time = 0
    counter = 0
    out_size = len(output_columns) - 1 if cost_mode == 'RL-MDN' else len(output_columns)
    last_speed_calc = time.time()
    predicted = np.array([0.749, 0.785, 0.613, 0.459, 0.679, 1., 0.])
    last_prediction = predicted.copy()
    hidden_states = np.empty((num_recurrent_layers, hidden_size), dtype='float32')
    active_hidden_states = np.empty((num_recurrent_layers, hidden_size), dtype='float32')
    for iteration in range(10000000):
        try:
            try:
                #command_msg = Float32MultiArray()
                #command_msg.data = predicted[0:out_size]
                print predicted
                pos = arm.get_pos()
                traj = toTrajectories(predicted, pos)
                print traj
                
                
                move = moveTowards(pos, traj)
                
                print pos, move
                
                #robot_command_pub.publish(command_msg)
                arm.move(move)
                #TODO jonathan process predicted
                
            except IOError:
                print 'could not open the prediction file.'
            prediction_diff = ((last_prediction[0:out_size] - predicted[0:out_size]) ** 2).mean()
            min_wait = np.clip(0.2 + prediction_diff * 80, .2, .5)
            time.sleep(min_wait)
            while True:
                new_state = pd.DataFrame({'task': [task_to_perform], 'time': [time.time()], 'gripper': [predicted[0]], 'joint1': [predicted[1]],
                    'joint2': [predicted[2]], 'joint3': [predicted[3]], 'joint4': [predicted[4]], 'joint5': [predicted[5]]})
                new_state = set_task_column_to_one_hot(new_state)
                print np.array(new_state[input_columns].iloc[0], dtype=theano.config.floatX)
                if last_time == new_state['time'][0]:
                    time.sleep(.005)
                    continue
                else:
                    break
            last_time = new_state['time'][0]
            x = np.array(new_state[input_columns].iloc[0], dtype=theano.config.floatX)
            predicted, newinitials = predict_one_timestep(net, imageFrame, predict_func, encoder, code_size, initials, x, out_size, iteration)
            last_prediction = predicted.copy()
            if plot_hidden_states:
                plot_arrays(newinitials)
        except(RuntimeError):
            print sys.exc_info()[0]

        counter += 1
        if (time.time() - last_speed_calc > 1):
            counter = 0
            last_speed_calc = time.time()
            
            
def main():

    if robot == 'al5d' or robot == 'mico':
        rospy.init_node('roboinstruct')
        def camera1_callback(msg):
            global camera1_msg
            camera1_msg = msg

        def move_callback(msg):
            global move_msg
            global last_move_msg_time
            move_msg = msg
            last_move_msg_time = time.time()

        #image_sub = rospy.Subscriber(camera1_image_topic, Image, camera1_callback)
        #image_sub = rospy.Subscriber("/move_info_for_test", Float32MultiArray, move_callback)
        #robot_command_pub = rospy.Publisher("/robot_command", Float32MultiArray, queue_size=100)

        def signal_handler(signal, frame):
            sys.exit(0)
        signal.signal(signal.SIGINT, signal_handler)

    locals().update(config)
    float_formatter = lambda x: "%.5f" % x
    np.set_printoptions(formatter={'float_kind': float_formatter})
    sample()


if __name__ == '__main__':
    main()
    
