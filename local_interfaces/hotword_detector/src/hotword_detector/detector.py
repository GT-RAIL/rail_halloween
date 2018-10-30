import rospy
import actionlib
import snowboydecoder
import sys
import signal
from hotword_detector.msg import DetectHotWordAction
from hotword_detector.srv import DetectHotWord as DetectHotWordSrv, DetectHotWordResponse


# Demo code for listening to two hotwords at the same time

class HWDetector():
    def __init__(self, models):
        # Flags
        self._detected = False
        self._stop = True

        # capture SIGINT signal, e.g., Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

        # A timer object as well incase we need to detect within a timeout
        self._timeout_timer = None

        # init the detector
        sensitivity = [0.465] * len(models)  # 0.465 is really well-tuned to Angel
        self.detector = \
            snowboydecoder.HotwordDetector(models, sensitivity=sensitivity)
        self.callbacks = [self.notify] * len(models)

        # detection service
        self._srv = rospy.Service('/detect_hotword',
                                  DetectHotWordSrv,
                                  self._srv_callback)

        # hotword detection action
        self._action = actionlib.SimpleActionServer('/detect_hotword',
                                                    DetectHotWordAction,
                                                    self._action_callback,
                                                    auto_start=False)
        self._action.register_preempt_callback(self._action_preempt)
        self._action.start()

        # notify programmmer of init
        rospy.loginfo(rospy.get_name() + ': hotword detector initialized!')

    def _action_callback(self, goal):
        result = self._action.get_default_result()
        result.reply = self.detect_hotword(goal.timeout)
        self._action.set_succeeded(result)

    def _action_preempt(self):
        self._stop = True
        result = self._action.get_default_result()
        result.reply = self._detected
        self._action.set_preempted(result)

    def _srv_callback(self, req):
        return DetectHotWordResponse(reply=self.detect_hotword(req.timeout))

    def detect_hotword(self, timeout=0.0):
        # start hotword not detected
        self._detected = False
        self._stop = False

        # Set a timeout timer if a timeout is specified
        if timeout > 0:
            if self._timeout_timer is not None:
                self._timeout_timer.shutdown()

            self._timeout_timer = rospy.Timer(rospy.Duration(timeout), self.timed_out, oneshot=True)

        # starts detector and waits indefinitely
        self.detector.start(detected_callback=self.callbacks,
                            interrupt_check=self.interrupt_callback,
                            sleep_time=0.03)

        if rospy.is_shutdown():
            # clean up detector and end
            self.detector.terminate()

        return self._detected

    def timed_out(self, evt):
        self._stop = True  # Notify the detector that it's time to stop
        self._timeout_timer = None

    def notify(self): # sets global to return that hotword detected
        self._detected = True
        self._stop = True

    def interrupt_callback(self): # notifies detector about interrupt or detection
        if rospy.is_shutdown(): # stop listening
            self._stop = True
        return self._stop

    def signal_handler(self, signal, frame): # notifies detector about interrupt
        self._stop = True  # stop listening
