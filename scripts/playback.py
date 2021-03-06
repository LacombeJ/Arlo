#!/usr/bin/env python
import arlo.main as main
import arlo.utils.args as args


args.check_maximum(1)

index = -1
arg0 = args.get_arg(0)
if arg0 is not None:
    index = int(arg0)

proj = main.ConsoleProject()

proj.playback(index)
