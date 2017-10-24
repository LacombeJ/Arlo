#!/usr/bin/env python


import arlo.input.ps4 as ps4

import sys
import signal

pc = None

def main():
    global pc
    
    pc = ps4.PS4Controller()

    pc.create()


    while True:

        pc.poll()
        
        #print pc.x()


    pc.destroy()





if __name__=='__main__':
    #try:
    main()
    #except KeyboardInterrupt:
    #    pc.destroy()
