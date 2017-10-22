#!/usr/bin/env python
import arlo.main as main


proj = main.ConsoleProject()

project = proj.project()

print project.path()
