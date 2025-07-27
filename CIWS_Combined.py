# !/usr/bin/python3
# CIWS_Combined.py created by Desktop at 7/27/2025

# Enter feature description here

# Python Libraries
# Import Libraries here

# Python Functions
# Define Functions Here
#!/usr/bin/env python3
# ciws_combined.py

"""
CIWS - Combined Script for Navigation and Person Detection
Runs both subsystems in parallel threads.
"""

import threading
import time
import signal
import sys
import cv2
import json

from navigation import NavigationSystem
from ultralytics import YOLO

# === Person Detection System ===
class PersonDetectorThread:
    def __init__(self, model_path="yolo11n.pt", camera_id=0):
        print("[Camera] Initializing YOLO model...")
        self.model = YOLO(model_path)
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise RuntimeError("Webcam could not be opened.")
        self.running = True
        self.PERSON_CLASS_ID = 0
        self.output_file = "largest_person_detection.json"

    def run(self):
        print("[Camera] Detection thread started.")
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                print("Frame grab failed.")
                break

            results = self.model(frame, stream=True, classes=self.PERSON_CLASS_ID)
            largest = None
            max_area = 0

            for r in results:
                frame = r.plot()
                if r.boxes:
                    for i, box in enumerate(r.boxes.xyxy.cpu().numpy()):
                        x1, y1, x2, y2 = box
                        area = (x2 - x1) * (y2 - y1)
                        if area > max_area:
                            max_area = area
                            largest = {
                                "center_x": int((x1 + x2) / 2),
                                "center_y": int((y1 + y2) / 2),
                                "width": int(x2 - x1),
                                "height": int(y2 - y1),
                                "confidence": float(r.boxes.conf[i]),
                                "class_id": int(r.boxes.cls[i]),
                                "label": self.model.names[int(r.boxes.cls[i])]
                            }

            if largest:
                print(f"[Camera] Person at ({largest['center_x']}, {largest['center_y']})")
                with open(self.output_file, 'w') as f:
                    json.dump(largest, f, indent=4)

            cv2.imshow("Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False

        self.cap.release()
        cv2.destroyAllWindows()
        print("[Camera] Detection stopped.")

    def stop(self):
        self.running = False


# === Combined Control ===
class CIWSControl:
    def __init__(self):
        print("[CIWS] Starting up subsystems...")

        self.detector = PersonDetectorThread()
        self.navigation_thread = threading.Thread(target=self.start_navigation, daemon=True)
        self.detector_thread = threading.Thread(target=self.detector.run, daemon=True)
        self.running = True

    def start_navigation(self):
        self.navigation = NavigationSystem()

    def run(self):
        print("[CIWS] Launching subsystems...")
        self.navigation_thread.start()
        self.detector_thread.start()

        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        print("[CIWS] Shutting down...")
        self.running = False
        self.detector.stop()
        sys.exit(0)


if __name__ == "__main__":
    controller = CIWSControl()
    signal.signal(signal.SIGINT, lambda *_: controller.stop())
    controller.run()

