#!/usr/bin/env python

import arlo.api.project as project
import arlo.api.module as module
import arlo.input.ps4 as ps4
import arlo.utils.log as log
import arlo.utils.term as term


proj = project.load("Recording/")



toDelete = []

def clean(entry):

    shouldKeep = True
    shouldKeep &= entry.fileExists("control.json")
    shouldKeep &= entry.fileExists("control_frames.json")
    shouldKeep &= entry.fileExists("meta.json")
    shouldKeep &= entry.fileExists("video.avi")
    shouldKeep &= entry.fileExists("video_frames.json")
    
    if not shouldKeep:
    
        toDelete.append(entry.name())
    

proj.entries(clean)

nothingCleaned = True

for entryName in toDelete:

    print "Deleting... ", entryName

    proj.deleteEntry(entryName)
    
    nothingCleaned = False
    
if nothingCleaned:
    print "Nothing to clean"

    




