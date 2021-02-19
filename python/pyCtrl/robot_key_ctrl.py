"""PYTHON module for robot driven by NEUI."""
import sys
import _thread as threading
import time

class ROBOT_KeyCtrl(object):
    def __init__(self, cb=None):
        self.cb = cb
        self.curKeyValue = 0
        self.curKeyCount = 0
        self.lastKeyCount = 0

        threading.start_new_thread(self.readInputKey, ())
        threading.start_new_thread(self.robotCtrlLoop, ())

    def readInputKey(self):
        while True:
            self.curKeyValue = sys.stdin.read(1).upper()
            self.curKeyCount = self.curKeyCount + 1
            time.sleep(0.1)

    def robotCtrlLoop(self):
        while True:
            if self.lastKeyCount != self.curKeyCount:
                if self.curKeyCount > 100:
                    self.curKeyCount = 0
                self.lastKeyCount = self.curKeyCount

                if self.cb is not None:
                    self.cb(key = self.curKeyValue)
