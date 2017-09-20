#!/usr/bin/env python

import arlo.data.playback as play
import arlo.utils.args as args


args.check_maximum(1)

index = -1
arg0 = args.get_arg(0)
if arg0 is not None:
    index = int(arg0)    


play.playback_session(index)




