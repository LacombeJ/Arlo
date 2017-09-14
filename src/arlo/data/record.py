
'''
record.py

This module contains functions for recording data

'''


import _recording_config as rc
import _recording_meta as rm

import arlo.utils.ext as ext
import arlo.utils.io as io
import arlo.utils.term as term

import cv2

def get_path():
    return rc.get_path()

def read_config():
    return rc.read_config()

def read_or_create_config():
    return rc.read_or_create_config()

def edit_config():
    return rc.edit_config()

def video_to_images(file_path):
    cap = cv2.VideoCapture(self._sub_video_path)
    images = []
    while (cap.isOpened()):
        ret, frame = cap.read()
        if ret == False:
            break
        images.append(frame)
    cap.release()
    
def images_to_video(file_path, images):
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    fps = 30.0
    width = images[0].shape[1]
    height = images[0].shape[2]
    video = cv2.VideoWriter(file_path,fourcc,fps,(width,height))
    for image in images:
        video.write(image)
    video.release()
    

class VideoIO(object):

    def __init__(self):
        pass

    def start(self):
        pass
        
        
class BasicVideoIO(VideoIO):
    
    def __init__(self):
        self._config = None
        self._meta = None
        pass
        
    def start(self):
        pass
        
    def _get_meta_path(self):
        return get_path() + 'meta.json'
    
    
    def _get_sub_path(self, index):
        return get_path() + self._meta['sub_name'] + str(index) +'/'
        
    def _get_next_path(self):
        return self._get_sub_path(self._meta['sub_count'])
        
        
    def _get_sub_meta_path(self, index):
        return self._get_sub_path(index) + 'meta.json'
        
    def _get_next_meta_path(self):
        return self._get_next_path() + 'meta.json'
        
        
    def _get_sub_video_name(self, index):
        return self._meta['file_name'] + str(index)
        
    def _get_next_video_name(self):
        return self._get_sub_video_name(self._meta['sub_count'])
        
    def _get_sub_video_path(self, index):
        return self._get_sub_path(index) + self._get_sub_video_name(index) + self._meta['file_ext']
        
    def _get_next_video_path(self):
        return self._get_sub_video_path(self._meta['sub_count'])
        
        

class BasicVideoReader(BasicVideoIO):
    
    def __init__(self,index=-1):
        self._index = index
        
    def start(self):
        self._config = read_config()
        
        self._meta_path = self._get_meta_path()
        self._meta = rm.read_meta(self._meta_path)
        if self._meta == None:
            print term.RED + "Meta data '{}' not found".format(self._meta_path)
            return
        
        index = range(self._meta['sub_count'])[self._index]
        self._sub_path = self._get_sub_path(index)
        self._sub_video_path = self._get_sub_video_path(index)
        self._sub_meta_path = self._get_sub_meta_path(index)
        self._sub_meta = rm.read_meta(self._sub_meta_path)
        
        print "Reading video meta file: '{}'".format(self._sub_meta_path)
        

        #Printing video meta data
        
        user = self._sub_meta['user']
        task = self._sub_meta['task']
        duration = self._sub_meta['duration']
        duration_s = duration / 1000
        vidfile = self._sub_meta['file_name'] + self._sub_meta['file_ext']
        frame_count = self._sub_meta['frame_count']
        datetime = ext.unpack_datetime(self._sub_meta['datetime'])
        
        term.print_blue_purple('File name: ', vidfile)
        term.print_blue_purple('Timestamp: ', str(datetime))
        term.print_blue_purple('Duration: ', '{} s'.format(duration_s))
        term.print_blue_purple('Number of frames: ', str(frame_count))
        term.print_blue_purple('User recorded: ', user)
        term.print_blue_purple('Task recorded: ', task)


        cap = cv2.VideoCapture(self._sub_video_path)
        
        frame_times = self._sub_meta['frame_times']
        
        self._prepare(self._sub_path)
        
        frame_index = 0
        next = True
        while (cap.isOpened()):
        
            next_index = frame_index
        
            if frame_index == len(frame_times):
                break
        
            if next:
                ret, frame = cap.read()
                next_index = frame_index + 1
            
            next, exit = self._read(ret,frame,frame_index,frame_times[frame_index])
            
            if exit:
                break
                
            frame_index = next_index
            
        
        cap.release()
        self._finish()
    
    # Called before video is read
    def _prepare(self,file_dir):
        pass
        
    # Called while video is read
    # ret value of cap.read()
    # frame image of cap.read()
    # index frame index in video
    # time  ms since first frame was recorded (in real video time)
    # return (next, exit)
    #   next if the next frame should be read on next call or false if same frame should be sent
    #   exit if no frame afterwards should be read
    def _read(self,ret,frame,index,time):
        return (True, False)
        
    # Called after video is read
    def _finish(self):
        pass
        
class BasicPlayback(BasicVideoReader):

    def _prepare(self,file_dir):
        self._frame_name = "Playback Window"
        print 'Playing video...'
        print 'Press '+term.BOLD+term.CYAN+'ESC'+term.END+' in Playback Window focus to exit'
        
        #For playing back video at the correct framerate
        self._first_time = None
        self._first_frame = True
        self._frame_time = 0
        
    def _read(self,ret,frame,index,time):
        
        next = False
        
        if self._first_frame:
            self._first_time = ext.datetime.now()
            self._first_frame = False
        else:
            self._frame_time = ext.datetime.now() - self._first_time
            if ext.delta_ms(self._frame_time) > time:
                next = True
        
        cv2.imshow(self._frame_name,frame)
                
        #if esc is pressed, return exit=True
        return next, cv2.waitKey(1) & 0xFF == 27
        
    def _finish(self):
        cv2.destroyAllWindows()
        print "Done playing video"
        
        

class BasicExtract(BasicVideoReader):
    
    def _prepare(self,file_dir):
        self._frame_name = "Playback Window"
        print 'Extracting images from video...'

        #For playing back video at the correct framerate
        self._base_name = 'img'
        self._base_ext = '.jpeg'
        self._extract_dir = file_dir + 'extract/'
        io.make_dir(self._extract_dir, True)

    def _read(self,ret,frame,index,time):
        cv2.imwrite(self._extract_dir + self._base_name + str(index) + self._base_ext,frame)
        return True, False
        
    def _finish(self):
        print "Done extracting images"
        
        

class BasicRecorder(BasicVideoIO):
    
    def __init__(self):
        pass

    def _init_meta(self):
        return {
            'meta_type' : 'meta',
            'type' : 'basic_video',
            'sub_name' : 'video',
            'sub_count' : 0,
            'file_name' : 'video',
            'file_ext' : '.avi',
            'record_type' : 'video'
        }
        
    def _init_sub_meta(self):
        return {
            'meta_type' : 'sub_meta',   #name of meta
            'type' : None,              #meta type
            'file_name' : None,         #file name
            'file_ext' : None,          #file ext
            'datetime' : None,          #datetime object of first frame
            'user' : None,              #user who recorded
            'task' : None,              #task recorded
            'width' : None,             #width of video frames
            'height' : None,            #height of video frames
            'frame_count' : None,       #number of video frames
            'frame_times' : None,       #array of ms time difference from first frame for each frame
            'duration' : None,          #duration of video = frame_times[-1]
            'annotation' : None,        #Can be anything, (ex: 'success','failure',array of data)
            'comments' : None           #Any extra comments for video
        }

        
    def start(self):
        self._config = read_or_create_config()
        
        io.make_dir(get_path())
        
        self._meta_path = self._get_meta_path()
        self._meta = rm.read_or_create_meta(self._init_meta(), self._meta_path)
        
        self._capture()
        
        
    def _capture(self):
        cap = cv2.VideoCapture(0)
        
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        fps = 30.0
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        sub_path = self._get_next_path()
        io.make_dir(sub_path, False)
        
        sub_meta_path = self._get_next_meta_path()
        sub_video_path = self._get_next_video_path()
        
        out = cv2.VideoWriter(sub_video_path,fourcc, fps, (width,height))

        frame_name = "Recording Window"
        
        first_time = None
        first_frame = True
        frame_count = 0
        
        frame_times = []
        
        # Video capture device found
        if cap.isOpened():
        
            print 'Recording video...'
        
            print 'Press '+term.BOLD+term.CYAN+'ESC'+term.END+' in Recording Window focus to exit',
            print 'and save video'
            
            print 'Press '+term.BOLD+term.CYAN+'CTRL+C'+term.END+' in terminal to discard video'
        
            while (cap.isOpened()):
                ret, frame = cap.read()
                
                if ret==True:
                
                    out.write(frame)
                    if first_frame:
                        first_time = ext.datetime.now()
                        first_frame = False
                        frame_time = first_time - first_time
                    else:
                        frame_time = ext.datetime.now() - first_time
                    
                    frame_count = frame_count + 1
                    frame_times.append(ext.delta_ms(frame_time))
                    cv2.imshow(frame_name,frame)
                    
                    #if esc is pressed
                    if cv2.waitKey(1) & 0xFF == 27:
                        break
                else:
                    break
                    
            sub_meta = self._init_sub_meta()

            sub_meta['type'] = self._meta['type']
            sub_meta['file_name'] = self._get_next_video_name()
            sub_meta['file_ext'] = self._meta['file_ext']
            sub_meta['datetime'] = ext.pack_datetime(first_time)
            sub_meta['user'] = self._config['user']
            sub_meta['task'] = self._config['task']
            sub_meta['width'] = width
            sub_meta['height'] = height
            sub_meta['frame_count'] = frame_count
            sub_meta['frame_times'] = frame_times
            sub_meta['duration'] = frame_times[-1]
            
            self._meta['sub_count'] = self._meta['sub_count'] + 1
            rm.write_meta_unsafe(self._meta, self._meta_path)
            
            rm.write_meta_unsafe(sub_meta, sub_meta_path)
            
            print "Created video: '{}'".format(sub_video_path)
              
        # No video capture device found    
        else:
        
            print term.RED + "Video capture failed - no video capture device found" + term.END
        
        cap.release()
        out.release()
        
        cv2.destroyAllWindows()
        
        



