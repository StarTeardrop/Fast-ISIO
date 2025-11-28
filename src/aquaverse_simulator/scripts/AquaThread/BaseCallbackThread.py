import threading
import time
import numpy as np
import cv2

class BaseCallbackWorker(threading.Thread):
    def __init__(self, tag="WorkerThread", robot_type="BlueRov2", robot_id=0):
        super().__init__()
        self.tag = tag
        self.robot_type = robot_type
        self.robot_id = robot_id
        self._running = True
        self._lock = threading.Lock()
        self._data = None

    def update_data(self, data):
        with self._lock:
            self._data = data

    def stop(self):
        self._running = False

    def run(self):
        raise NotImplementedError("run() must be implemented in subclasses")


