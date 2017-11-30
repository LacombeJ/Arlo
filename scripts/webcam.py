#!/usr/bin/env python

import arlo.api.project as project
import arlo.api.module as module
import arlo.input.ps4 as ps4
import arlo.utils.log as log
import arlo.utils.term as term

proj = project.load("Webcam/")

record_modules = [
    module.ModuleCameraExtendedcv
]

proj.record(record_modules)

