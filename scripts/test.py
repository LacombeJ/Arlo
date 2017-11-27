#!/usr/bin/env python

import arlo.api.project as project

import arlo.net.saver as saver
import arlo.net.trainer as trainer
import arlo.net.vaegan as vaegan

import cv2
import sys
import numpy as np
import random as random
from PIL import Image
from scipy.misc import imresize


proj = project.load("Recording/")

num_entries = proj.entryCount()

MIN = 50000
MAX = -50000

for i in range(num_entries):
    entry = proj.entry(i)
    print entry
    controls = entry.get('control_file','json_data')
    for c in controls:
        for e in c:
            MIN = min(e,MIN)
            MAX = max(e,MAX)

print "min: {}   max: {}".format(MIN,MAX)








