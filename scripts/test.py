#!/usr/bin/env python

import arlo.api.project as project

import arlo.net.saver as saver
import arlo.net.trainer as trainer
import arlo.net.vaegan as vaegan

import arlo.data.node as node

import arlo.utils.ext as ext

import cv2
import sys
import numpy as np
import random as random
from PIL import Image
from scipy.misc import imresize

def main():

    #save_image()
    #encode()
    #read_data()
    #count_num()
    recVideo()

def recVideo():

    proj = project.load("Recording/")
    entry = proj.entry(304)
    
    data = entry.get("video_file", "video_numpy")
    print data.shape
    
    images = []
    for d in data:
        d = processImage(d)
        d = prepareData(d)
        images.append(d)
        
    images = np.array(images, dtype=np.float32)
    
    print images.shape
    print "Loading net..."
    net = vaegan.VAEGAN()
    network_saver = saver.NetworkSaver('vaegan/models/', net=net)
    network_saver.load()
    print "Net loaded"
    
    count = 0
    length = images.shape[0]
    i = 20
    
    total = []
    
    while True:
        if count >= length:
            break
        start = count
        end = min(count+i, length)
        print start, end
        
        imgBatch = images[start:end]
        rec = net.reconstruct(imgBatch)
        rec = rec.swapaxes(2,3)
        rec = rec.swapaxes(1,3)
        for r in rec:
            total.append(r)
        
        count += i
        
    total = np.array(total, np.float32)
    
    print total.shape
    print total
    total = np.array(total, np.uint8)
    
    # Output -------------------
    
    file = 'video_rec_new.avi'

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    fps = 30.0
    width = 128
    height = 128

    out = cv2.VideoWriter(file, fourcc, fps, (width, height))
    
    for f in total:
        out.write(f)
    
    out.release()
    

def count_num():

    proj = project.load("Recording/")
    
    count = 0
        
    num_entries = proj.entryCount()
    
    for i in range(num_entries):
        entry = proj.entry(i)
        duration = entry.get("control_duration")
        duration /= 1000.0
        c = duration * 30.0/8.0
        count += int(c)
        print count
        
    print count

def read_data():

    data_out = np.load('data_out.npy', mmap_mode='r')
    
    print data_out
    
    index = 0
    for i in range(len(data_out)):
        array = data_out[i]
        print array
            
    #18536
    
    print "======================================"
    
    
    
    print data_out.shape
    
    

def get_sample_image():

    proj = project.load("Recording/")
    # Look at 101
    entry = proj.entry(266)
    video = entry.get("video_file", "video_numpy")
    
    image = video[80]
    image = processImage(image)
    #image = cv2.flip(image, 1)
    
    return image


def encode():
    
    net = vaegan.VAEGAN()
    network_saver = saver.NetworkSaver('vaegan/models/', net=net)
    network_saver.load()

    data = get_sample_image()
    
    
    #cv2.imwrite('Images/imwrite.jpg', data)
    #data = cv2.imread('Images/imwrite.jpg')
    
    data = np.array(data)
    data = cv2.resize(data,(128,128))
    data = [data]
    data = np.array(data, dtype='float32')
    data = data / 127.5 - 1;
    data = np.array(data)
    data = data.swapaxes(1,3)
    data = data.swapaxes(2,3)
        
    '''
    image = net.image(data)
    Image.fromarray(image).save( 'Images/netimage.png' )
    '''
    
    image = net.reconstruct(data)
    print image.shape
    image = image[0]
    #image = image[ ::-1, :, : ]
    image = image.swapaxes(1,2)
    image = image.swapaxes(0,2)
    cv2.imwrite('Images/netimage2.jpeg', image)

def save_image():
    
    root, new = node.load("Images", translate=ext.translate)

    print root.path()+'image.jpeg'

    cv2.imwrite(root.path()+'image.jpeg', image)
    root.set('image', 'image.jpeg')
    root.save()

    print video.shape


    ext.display(image)


def processImage(image):

    # Resize
    x = 75
    y = 25
    w = 400
    h = 400
    image = image[y:y+h, x:x+w]
    #image = np.array(image, dtype='float32')
    #image = image / 127.5 - 1;
    #image = cv2.resize(image,(128,128))

    return image

def prepareData(data):

    data = np.array(data)
    data = cv2.resize(data,(128,128))
    data = np.array(data, dtype='float32')
    data = data / 127.5 - 1;
    data = np.array(data)
    data = data.swapaxes(0,2)
    data = data.swapaxes(1,2)
    
    return data

if __name__=='__main__':
    main()
    
    

