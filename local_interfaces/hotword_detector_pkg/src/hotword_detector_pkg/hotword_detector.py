import rospy
import snowboydecoder
import sys
import signal
from hotword_detector_pkg.srv import *

# Demo code for listening two hotwords at the same time
DETECTED = False


class HWDetector():
    def __init__(self, models):
        # detection service
        s = rospy.Service('/detect_hotword', 
                          DetectHotWord, 
                          self.detect_hotword)
        # capture SIGINT signal, e.g., Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        # init the detector
        sensitivity = [0.5]*len(models)
        self.detector = \
            snowboydecoder.HotwordDetector(models,sensitivity=sensitivity)
        self.callbacks = [self.notify]*len(models)
        # notify programmmer of init
        rospy.loginfo(rospy.get_name() + ': hotword detector initialized!')
    
    def detect_hotword(self,req):
        # start hotword not detected
        global DETECTED
        DETECTED = False
        # starts detector and waits indefinitely
        self.detector.start(detected_callback=self.callbacks, 
                            interrupt_check=self.interrupt_callback, 
                            sleep_time=0.03)
               
        if rospy.is_shutdown():
            # clean up detector and end
            self.detector.terminate()
        
        return DetectHotWordResponse(DETECTED)
        
    def notify(self): # sets global to return that hotword detected
        global DETECTED
        DETECTED = True
    
    
    def interrupt_callback(self): # notifies detector about interrupt or detection
        global DETECTED
        if rospy.is_shutdown(): # stop listening
            DETECTED = True
        return DETECTED
        
    def signal_handler(self, signal, frame): # notifies detector about interrupt
        global DETECTED # stop listening
        DETECTED = True
