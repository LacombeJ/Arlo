
import os
import inspect
import sys

import cv2

from PyQt5 import QtCore, QtGui, QtWidgets, uic

gui_app_dir = os.path.dirname(os.path.abspath(inspect.stack()[0][1]))


def Window(app):
    window = uic.loadUi(gui_app_dir+'/forms/window.ui')

    # Set custom splitter sizes
    window.verSplitter.setSizes((300,50))
    window.horSplitter.setSizes((300,50))
    
    # Center and resize main window
    width = 1250
    height = 850
    
    window.resize(width, height)
    
    return window
    

def HomeForm():
    home = uic.loadUi(gui_app_dir+'/forms/home_form.ui')

    return home


class Recorder(object):

    def __init__(self, video_frame):
        self._video_frame = video_frame
        self._file_name = 'Recording/recording0/video.avi'
        
    def play(self):
        ret, frame = self._cap.read()
        
        if ret == False:
            self._timer.stop()
            self._cap.release()
            log("Playback finished")
            return
        
        #frame = cv2.cvtColor(frame, cv2.CV_BGR2RGB)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = QtGui.QImage(frame, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(img)
        self._video_frame.setPixmap(pix)

    def start(self):
        self._cap = cv2.VideoCapture(self._file_name)
        
        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self.play)
        self._timer.start(1000.0/300.0)
        log("Starting playback...")


def RecordForm():
    record = uic.loadUi(gui_app_dir+'/forms/record_form.ui')
    
    return record
    

outputLog = None

def start():
    global outputLog
    
    app = QtWidgets.QApplication(sys.argv)
    
    window = Window(app)
    
    home = HomeForm()
    record = RecordForm()
    
    window.tabWidget.addTab(home,'Home')
    window.tabWidget.addTab(record,'Record')
    
    outputLog = window.outputLog
    
    log("Started Qt Application")
    
    recorder = Recorder(record.videoFrame)
    
    # Add actions
    window.actionRecord.triggered.connect(recorder.start)
    
    window.show()
    
    sys.exit(app.exec_())
    
    
def log(text):
    outputLog.append(text)
    

