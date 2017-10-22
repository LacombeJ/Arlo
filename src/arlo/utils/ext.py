# ext.py
# This module contains miscellaneous utility functions as extensions to other modules

import os
import inspect
import time
import datetime as _datetime


def list_format(_list):
    result = ""
    for i, e in enumerate(_list):
        result = result + e
        if i == len(_list) - 1:
            break;
        result = result + ', '
    return result

datetime = _datetime.datetime

def pack_datetime(time):
    return [
        time.year,
        time.month,
        time.day,
        time.hour,
        time.minute,
        time.second,
        time.microsecond
    ]

def unpack_datetime(array):
    return _datetime.datetime(
        array[0],
        array[1],
        array[2],
        array[3],
        array[4],
        array[5],
        array[6]
    )

def delta_ms(timedelta):
    return timedelta.total_seconds() * 1000


class TimeStamper(object):
    
    def __init__(self):
        self._first_time = None
        self._times = []

    def stamp(self):
        if self._first_time == None:
            self._first_time = datetime.now()
            frame_diff = self._first_time - self._first_time  # First frame diff should always be 0
        else:
            frame_diff = datetime.now() - self._first_time
            
        frame_diff_ms = delta_ms(frame_diff)
        
        self._times.append(frame_diff_ms)
        
    def initial(self):
        return self._first_time
        
    def times_ms(self):
        return self._times
    
    def last_time_ms(self):
        return self._times[-1]


class TimeSync(object):

    # time_frames array of ms differences
    # time datetime
    # offset ms
    def __init__(self, time_frames, time=None, offset=0):
        self._time_frames = time_frames
        self._time = time
        self._offset = offset
        self._index = 0
        if self._time==None:
            self._time = datetime.now()

    # This class takes in an array of ms diff (for ex: [0, 1, 2, 3])
    # and returns (next,index,done) where next is True if index was changed
    # since last call and where index is equal to the current time frame
    # equivalent, and where done is True if all time frames have been passed
    # Ex:
    # Two methods that call sync [0, 1000, 2000, 3000, 4000] with the same
    # initial datetime, continously on separate threads
    # would return True every second at the same time
    # Ex:
    # Sync returns True once for every element in time_frames
    def sync(self):
        if self._index >= len(self._time_frames):
            return False, self._index, True
        time = datetime.now()
        delta = delta_ms(time - self._time)
        next = False
        index = self._index
        if delta >= self._time_frames[self._index]+self._offset:
            next = True
            self._index += 1
        return next, index, False


class TimeRate(object):
    
    def __init__(self):
        self._time = None

    # Return true if last call to rate or hold exceeded the given time (ms)
    def rate(self, ms):
        val = False
        if self._time == None:
            val = True
            self._time = datetime.now()
        else:
            diff_ms = delta_ms(datetime.now() - self._time)
            if diff_ms > ms:
                val = True
                self._time = datetime.now()
        return val

    # Holds thread if last call to rate or hold was within the given time (ms)
    def hold(self, ms):
        if self._time == None:
            self._time = datetime.now()
        else:
            diff_ms = delta_ms(datetime.now() - self._time)
            if diff_ms < ms:
                time.sleep((ms - diff_ms)/1000.0)
                self._time = datetime.now()


class Timer(object):

    def __init__(self):
        self._time = datetime.now()

    def within(self, ms):
        diff_ms = delta_ms(datetime.now() - self._time)
        if diff_ms < ms:
            return True
        return False
