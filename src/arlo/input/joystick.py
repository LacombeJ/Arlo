# Released by rdb under the Unlicense (unlicense.org)
# Based on information from:
# https://www.kernel.org/doc/Documentation/input/joystick-api.txt

import os, struct, array
import fcntl
from fcntl import ioctl

import sys
import signal
import threading

# These constants were borrowed from linux/input.h
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'trottle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}


    

class Joystick(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        
    def create(self):
        
        self._lock = threading.Lock()
        
        self._axis = {}
        self._button = {}
        self._axis_map = []
        self._button_map = []
        self._run = True
        
        fn = '/dev/input/js0'
        self._jsdev = open(fn, 'rb')
        fd = self._jsdev.fileno()
        flag = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, flag | os.O_NONBLOCK )
        
        buf = array.array('c', ['\0'] * 64)
        ioctl(self._jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
        js_name = buf.tostring()
        
        buf = array.array('B', [0])
        ioctl(self._jsdev, 0x80016a11, buf) # JSIOCGAXES
        self._num_axes = buf[0]

        buf = array.array('B', [0])
        ioctl(self._jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
        self._num_buttons = buf[0]
        
        buf = array.array('B', [0] * 0x40)
        ioctl(self._jsdev, 0x80406a32, buf) # JSIOCGAXMAP

        for axis in buf[:self._num_axes]:
            axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self._axis_map.append(axis_name)
            self._axis[axis_name] = 0.0
        
        buf = array.array('H', [0] * 200)
        ioctl(self._jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

        for btn in buf[:self._num_buttons]:
            btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
            self._button_map.append(btn_name)
            self._button[btn_name] = 0

        print '%d axes found: %s' % (self._num_axes, ', '.join(self._axis_map))
        print '%d buttons found: %s' % (self._num_buttons, ', '.join(self._button_map))

        # Appends a keyboard interrupt signal handler that destroys this thread
        self._prev_signal = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self._signal_destroy)

        return True

    def _signal_destroy(self, signal, frame):
        self.destroy()
        if self._prev_signal != None:
            self._prev_signal(signal,frame)
    
    def destroy(self):
        self._run = False
        
    def run(self):
        while self._run:
        
            try:
                evbuf = self._jsdev.read(8)
                
                time, value, type, number = struct.unpack('IhBB', evbuf)

                if type & 0x80:
                    # print "(initial)",
                    pass

                if type & 0x01:
                    button = self._button_map[number]
                    if button:
                        with self._lock:
                            self._button[button] = value
                        if value:
                            print "%s pressed" % (button)
                        else:
                            print "%s released" % (button)
                
                if type & 0x02:
                    axis = self._axis_map[number]
                    #print number, value / 32767.0
                    if axis:
                        fvalue = value
                        with self._lock:
                            self._axis[axis] = fvalue / 32767.0
                        #print "{} axis with value {}".format(axis,fvalue)
            
            except IOError: # No events
                pass
                
            
        
    def getAxis(self):
        with self._lock:
            return self._axix
        
    def _setAxis(self,axis):
        with self._lock:
            self._axis = axis
        
    def getButtons(self):
        with self._lock:
            return self._button
        
    def _setButtons(self,buttons):
        with self._lock:
            self._button = buttons
            
'''
js = Joystick()
js.create()
js.start()

print "???"

time.sleep(3)
ps4.exit()
'''

