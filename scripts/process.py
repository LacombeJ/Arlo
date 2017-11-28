#!/usr/bin/env python

import arlo.api.project as project
import arlo.api.module as module
import arlo.input.ps4 as ps4

import arlo.utils.ext as ext
import arlo.utils.log as log
import arlo.utils.term as term

import arlo.data.node as node

import os
import numpy as np
import cv2

from arlo.train_config import config
locals().update(config)

def main():
    rec = project.load("Recording/")
    
    #TODO jonathan save values to a directory
    #root, new = node.load("Processed/")
    
    num_entries = rec.entryCount()
    
    index_frames = 0
    
    video_data = np.memmap('video_data.npy', dtype='float32', mode='w+',
                    shape=(max_frames, 3, 128, 128))
    control_data = np.memmap('control_data.npy', dtype='float32', mode='w+',
                    shape=(max_frames, 7))
    in_data = np.memmap('in_data.npy', dtype='float32', mode='w+',
                    shape=(max_frames, 1))
    
    for i in range(num_entries):
        entry = rec.entry(i)
        
        control_datetime = entry.get("control_datetime", "datetime")
        control_frames = entry.get("control_frame_times", "json_data")
        control = entry.get("control_file", "json_data")
        
        video_datetime = entry.get("video_datetime", "datetime")
        video_frames = entry.get("video_frame_times", "json_data")
        video = entry.get("video_file", "video_numpy")
        
        diff_ms = ext.delta_ms(control_datetime - video_datetime)
        video_start = 0
        control_start = diff_ms
        if diff_ms < 0:
            video_start = abs(diff_ms)
            control_start = 0
        
        timeRateS = 8.0 / 30.0 # seconds
        timeRate = timeRateS * 1000.0 # ms
        time = 0
        
        while True:
        
            if max_frames <= index_frames:
                break
        
            control_index = ext.timeIndex(time + control_start, control_frames)
            video_index = ext.timeIndex(time + video_start, video_frames)
            
            if control_index==None and video_index==None:
                break
                
            VIDEO = clampedIndex(video_index, video)
            CONTROL = clampedIndex(control_index, control)
            
            VIDEO = processImage(VIDEO)
            CONTROL = processControl(CONTROL)
            
            video_data[index_frames] = VIDEO
            control_data[index_frames] = CONTROL
            in_data[index_frames] = np.ones(1)
            
            time += timeRate
            index_frames += 1
        
        print "{} / {}".format(index_frames, max_frames)
        
        if max_frames <= index_frames:
            break
            
    np.save('img_data.npy', video_data)
    np.save('data_in.npy', in_data)
    np.save('data_out.npy', control_data)
    
    os.remove(video_data.filename)
    os.remove(in_data.filename)
    os.remove(control_data.filename)  
    
def processImage(image):

    # Resize
    x = 75
    y = 25
    w = 400
    h = 400
    image = image[y:y+h, x:x+w]
    image = np.array(image, dtype='float32')
    image = image / 127.5 - 1;
    image = cv2.resize(image,(128,128))
    
    # Swap axes
    image = image.swapaxes(0,2)
    image = image.swapaxes(1,2)

    return image
    
def processControl(control):

    control = alpha(np.array(control), 600.0, 2400.0) # Values from max-min of servos in leap_al5d
    
    steps_to_goal = 0.0
    
    control = np.array([control[5], control[0], control[1], control[2], control[3], control[4], steps_to_goal])
    
    return control


def alpha(value, MIN, MAX):
    V = value - MIN
    V /= (MAX - MIN)
    return V

def clampedIndex(index, array):
    if index == -1:
        return array[0]
    elif index == None:
        return array[-1]
    return array[index]

if __name__=="__main__":
    main()




