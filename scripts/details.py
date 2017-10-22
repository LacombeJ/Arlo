#!/usr/bin/env python
import arlo.main as main


cp = main.ConsoleProject()

project = cp.project()


def func(entry):
    print entry.path(), entry.get('user'), entry.get('task'),
    entry.get('camera'), entry.get('controller')

project.entries(func)
