
import cv2

def load(video_path):
    result = []
    
    cap = cv2.VideoCapture(video_path)
    
    while (cap.isOpened()):
    
        ret, frame = cap.read()
        
        if ret == False:
            break
            
        result.append(frame)
        
    cap.release()
    
    return result
    
def save(video_path,video):

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    fps = 30.0
    width = int(video[0].shape[1])
    height = int(video[0].shape[0])
    
    out = cv2.VideoWriter(video_path,fourcc, fps, (width,height))
    
    for frame in video:
        out.write(frame)
        
    out.release()
    
    
def play(video):

    frame_name = "Video"
    index = 0
    while (index < len(video)):
        
        cv2.imshow(frame_name,video[index])
        
        index = index + 1
        
        if cv2.waitKey(1) & 0xFF == 27:
            break
    
    cv2.destroyAllWindows()
    
'''
class BasicVideo(object):
    
    def __init__(self,path):
        self._path = path
        
    def play(self):
    
        cap = cv2.VideoCapture(self._path)
        
        frame_name = "Video"
        
        while (cap.isOpened()):
                
            ret, frame = cap.read()
                
            if ret == None:
                break
            
            cv2.imshow(frame_name,frame)
            
            if cv2.waitKey(1) & 0xFF == 27:
                break
        
        cap.release()
        cv2.destroyAllWindows()

class BasicVideo(object):
    
    def __init__(self,path):
        self._path = path
        
    def play(self):
    
        cap = cv2.VideoCapture(self._path)
        
        frame_name = "Video"
        
        while (cap.isOpened()):
                
            ret, frame = cap.read()
                
            if ret == None:
                break
            
            cv2.imshow(frame_name,frame)
            
            if cv2.waitKey(1) & 0xFF == 27:
                break
        
        cap.release()
        cv2.destroyAllWindows()
        
        
        
        
class Video(object):
    
    def __init__(self,path,frame_times=None):
        self._path = path
        self._frame_times = frame_times
        
    def play(self):
    
        cap = cv2.VideoCapture(self._path)
        
        frame_name = "Video"
        
        #For playing back video at the correct framerate
        first_time = None
        first_frame = True
        frame_time = 0
        
        frame_index = 0
        next = True
        while (cap.isOpened()):
        
            next_index = frame_index
        
            if frame_index == len(self._frame_times):
                break
        
            if next:
                ret, frame = cap.read()
                next_index = frame_index + 1
            
            next = False
        
            if first_frame:
                first_time = ext.datetime.now()
                first_frame = False
            else:
                frame_time = ext.datetime.now() - first_time
                if ext.delta_ms(frame_time) > time:
                    next = True
            
            cv2.imshow(frame_name,frame)
                    
            #if esc is pressed, return exit=True
            if cv2.waitKey(1) & 0xFF == 27:
                break
                
            frame_index = next_index
            
        
        cap.release()
        cv2.destroyAllWindows()
'''
        

