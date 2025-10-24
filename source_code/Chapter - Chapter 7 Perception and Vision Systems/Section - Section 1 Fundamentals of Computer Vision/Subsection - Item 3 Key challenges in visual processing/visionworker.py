import cv2                # image ops
import numpy as np
import threading
# Frame source abstraction (camera, simulator)
class VisionWorker(threading.Thread):
    def __init__(self, frame_source, publish_fn):
        super().__init__(daemon=True)
        self.src = frame_source
        self.publish = publish_fn
        self.running = True
    def run(self):
        while self.running:
            color, depth, ts = self.src.read()                # blocking read
            # Exposure compensation (simple histogram matching) 
            color_eq = cv2.cvtColor(color, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(color_eq)
            l = cv2.equalizeHist(l)                          # reduce shadows/highlights
            color_eq = cv2.merge((l,a,b))
            color_out = cv2.cvtColor(color_eq, cv2.COLOR_LAB2BGR)
            # Depth confidence: check valid range and neighbor agreement
            valid = (depth > 0.1) & (depth < 5.0)             # meters
            median = cv2.medianBlur(depth, 5)
            conf = np.abs(depth - median) < 0.05              # rejection threshold
            depth_conf = valid & conf
            # Publish processed frames and confidence mask
            self.publish(color_out, depth, depth_conf, ts)
    def stop(self):
        self.running = False
# Usage: instantiate VisionWorker with simulator camera and ROS publish function.