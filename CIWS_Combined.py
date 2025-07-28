# !/usr/bin/python3
# CIWS_Combined.py created by Desktop at 7/27/2025

"""
CIWS - Combined Script for Navigation, Vision, and Firing Control
Controls:
- Navigation using onboard LiDAR
- Person detection using YOLOv11
- Auto/manual servo aiming and firing when size threshold is met
"""
# Python Libraries
import threading
import time
import signal
import sys
import cv2
import json
import os

# Runtime Environment Setup
os.environ["DISPLAY"] = ":0" # Forces the camera view to always open on local screen

# Subsystems
from Sensors.navigation import NavigationSystem
from Actuators.TriggerControl import TriggerControl
from KeyboardDispatcher import KeyboardDispatcher



# YOLO Config
import ultralytics
ultralytics.settings.verbose = False # turn off the log printout
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
        cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                print("Frame grab failed.")
                break

            results = self.model(frame, stream=True, classes=self.PERSON_CLASS_ID, verbose =False)
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
                # print(f"[Camera] Person at ({largest['center_x']}, {largest['center_y']})")
                with open(self.output_file, 'w') as f:
                    json.dump(largest, f, indent=4) # Used to share detection data

            #frame_resized = cv2.resize(frame, (800, 600))
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
        self.aim_thread = threading.Thread(target=self.aim_from_detection, daemon=True)

        self.running = True
        self.trigger = None  # Delay trigger initialization until after navigation
        self.keyboard = None
        self.trigger_ready = threading.Event()  # Event to wait for trigger setup

    def start_navigation(self):
        self.navigation = NavigationSystem()
        self.trigger = TriggerControl()  # Initialize trigger after navigation setup
        self.keyboard = KeyboardDispatcher(
            nav_handler=self.navigation.handle_key,
            trigger_handler=self.trigger.handle_key
        )
        self.keyboard.start()
        self.trigger_ready.set() # Signal that trigger is ready

    def aim_from_detection(self):
        """Reads detection data and uses it to control turret aim."""
        self.trigger_ready.wait()  # Wait until trigger is initialized
        print("[CIWS] Aiming thread started.")
        FIRE_AREA_THRESHOLD = 100_000  # Tune this value (e.g., 320x320 area)
        COOLDOWN_SECONDS = 3 # Cooldown period between shots
        last_shot_time = 0

        while self.running:
            try:
                if os.path.exists(self.detector.output_file):
                    with open(self.detector.output_file, 'r') as f:
                        data = json.load(f)
                        if data and isinstance(data, dict):
                            cx = data.get("center_x")
                            cy = data.get("center_y")
                            width = data.get("width")
                            height = data.get("height")

                            if cx is not None and cy is not None:
                                self.trigger.aim(cx, cy)

                            if width and height:
                                area = width * height
                                now = time.time()
                                if area >= FIRE_AREA_THRESHOLD and now - last_shot_time >= COOLDOWN_SECONDS:
                                    if getattr(self.trigger, 'mode', 'auto') == 'auto':
                                        print(f"[CIWS] Auto-firing at target (area={area}).")
                                        self.trigger.shoot()
                                        last_shot_time = now


            except Exception as e:
                print(f"[CIWS] Aim error: {e}")
            time.sleep(0.05)  # Fast enough for responsiveness but not CPU intensive

    def run(self):
        print("[CIWS] Launching subsystems...")
        self.navigation_thread.start()
        self.detector_thread.start()
        self.aim_thread.start()

        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        print("[CIWS] Shutting down...")
        self.running = False
        self.detector.stop()
        try:
            self.detector.stop()
        except Exception:
            pass

        try:
            if self.trigger:
                self.trigger.cleanup()
        except Exception:
            pass

        try:
            if self.keyboard:
                self.keyboard.stop()
        except Exception:
            pass

        print("[CIWS] All systems shutdown.")
        sys.exit(0)


if __name__ == "__main__":
    controller = CIWSControl()
    signal.signal(signal.SIGINT, lambda *_: controller.stop())
    controller.run()



