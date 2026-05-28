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
import os

# Runtime Environment Setup
os.environ["DISPLAY"] = ":0" # Forces the camera view to always open on local screen
os.environ["QT_LOGGING_RULES"] = "*=false" # stops the Qt message from displaying. cosmetic only

# Subsystems
from Sensors.navigation import NavigationSystem
from Sensors.lidar_visualizer import LidarVisualizer
from Actuators.TriggerControl import TriggerControl
from KeyboardDispatcher import KeyboardDispatcher



# YOLO Config
import ultralytics
ultralytics.settings.verbose = False # turn off the log printout
from ultralytics import YOLO

# === Watchdog ===
class Watchdog:
    """Tracks communication with each subsystem. Triggers safe state if any go stale."""
    def __init__(self, stale_threshold=2.0):
        self._lock = threading.Lock()
        self._heartbeats = {}
        self.stale_threshold = stale_threshold

    def beat(self, name):
        with self._lock:
            self._heartbeats[name] = time.time()

    def check(self):
        """Returns list of subsystem names that have gone stale."""
        now = time.time()
        stale = []
        with self._lock:
            for name, last in self._heartbeats.items():
                if now - last > self.stale_threshold:
                    stale.append(name)
        return stale

    def age(self, name):
        """Returns seconds since last heartbeat, or None if never beat."""
        with self._lock:
            last = self._heartbeats.get(name)
        return (time.time() - last) if last else None

# === Person Detection System ===
class DetectionBuffer:
    """Thread-safe holder for the latest person detection."""
    def __init__(self):
        self._lock = threading.Lock()
        self._data = None  # None = no current target

    def update(self, detection):
        """detection is a dict or None"""
        with self._lock:
            self._data = detection

    def read(self):
        """Returns the latest detection dict, or None if no target."""
        with self._lock:
            return self._data

class PersonDetectorThread:
    def __init__(self, model_path="yolov8n.pt", camera_id=0, lidar_vis=None,
             quit_handler=None, detection_buffer=None, watchdog=None):
        print("[Camera] Initializing YOLO model...")
        self.lidar_vis = lidar_vis
        self.quit_handler = quit_handler
        self.detection_buffer = detection_buffer
        self.watchdog = watchdog
        self.model = YOLO(model_path)
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise RuntimeError("Camera could not be opened.")
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"[Camera] Resolution: {self.frame_width}x{self.frame_height}")    
        self.running = True
        self.PERSON_CLASS_ID = 0

    def run(self):
        print("[Camera] Detection thread started.")
        cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Camera Feed", 480, 360)   # standard small

        if self.lidar_vis is not None:
            cv2.namedWindow("LiDAR", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("LiDAR", self.lidar_vis.window_size, self.lidar_vis.window_size)

        while self.running:
            if self.watchdog:
                self.watchdog.beat("camera")
            ret, frame = self.cap.read()
            if not ret:
                print("Frame grab failed.")
                break

            results = self.model(frame, stream=True, classes=self.PERSON_CLASS_ID, verbose=False)
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

            if self.detection_buffer is not None:
                self.detection_buffer.update(largest)  # largest is None if no target

            cv2.imshow("Camera Feed", frame)

            # Also draw the LiDAR window from this thread (GTK requires single-thread GUI)
            if self.lidar_vis is not None:
                try:
                    lidar_frame = self.lidar_vis._draw_frame()
                    cv2.imshow("LiDAR", lidar_frame)
                except Exception as e:
                    print(f"[Camera] LiDAR draw error: {e}")

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("[Camera] 'q' pressed — shutting down CIWS.")
                self.running = False
                if self.quit_handler:
                    self.quit_handler()
            elif key == ord('f'):
                current = cv2.getWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN)
                if current == cv2.WINDOW_FULLSCREEN:
                    cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
                else:
                    cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        self.cap.release()
        try:
            cv2.destroyWindow("Camera Feed")
        except Exception:
            pass
        try:
            cv2.destroyWindow("LiDAR")
        except Exception:
            pass
        # Pump the event loop a few times so Qt processes the destroy events
        for _ in range(5):
            cv2.waitKey(1)
        print("[Camera] Detection stopped.")

    def stop(self):
        self.running = False


# === Combined Control ===
class CIWSControl:
    def __init__(self):
        print("[CIWS] Starting up subsystems...")
        self.lidar_vis = LidarVisualizer()
        self.detection_buffer = DetectionBuffer()
        self.watchdog = Watchdog(stale_threshold=3.0)
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.detector = PersonDetectorThread(lidar_vis=self.lidar_vis, quit_handler=self.stop, detection_buffer=self.detection_buffer, watchdog=self.watchdog)
        self.navigation_thread = threading.Thread(target=self.init_sys, daemon=True)
        self.detector_thread = threading.Thread(target=self.detector.run, daemon=True)
        self.aim_thread = None
        self.running = True
        self.trigger = None  # Delay trigger initialization until after navigation
        self.keyboard = None
        self.navigation = None
        self.trigger_ready = threading.Event()  # Event to wait for trigger setup

    def init_sys(self):
        self.navigation = NavigationSystem(watchdog=self.watchdog)
        self.trigger = TriggerControl(frame_width=self.detector.frame_width, frame_height=self.detector.frame_height)  # Initialize trigger after navigation setup
        self.keyboard = KeyboardDispatcher(
            nav_handler=self.navigation.handle_key,
            trigger_handler=self.trigger.handle_key,
            quit_handler=self.stop
        )
        self.keyboard.start()
        self.trigger_ready.set() # Signal that trigger is ready

        # Launch aiming only if in auto mode
        if self.trigger.mode == "auto":
            self.aim_thread = threading.Thread(target=self.aim_from_detection, daemon=True)
            self.aim_thread.start()

    def _watchdog_loop(self):
        """Monitor subsystem heartbeats. Force safe state if anything goes stale."""
        print("[Watchdog] Started.")
        while self.running:
            time.sleep(0.5)
            stale = self.watchdog.check()
            if stale:
                # Stop motors
                try:
                    if self.navigation and self.navigation.motor:
                        self.navigation.motor.stop()
                except Exception:
                    pass
                # Engage trigger safe state (blocks aim + firing, centers servos)
                try:
                    if self.trigger:
                        self.trigger.enter_safe_state(reason=f"stale: {stale}")
                except Exception:
                    pass
            else:
                # All heartbeats fresh -- clear safe state if it was set
                try:
                    if self.trigger and self.trigger.safe_state:
                        self.trigger.exit_safe_state()
                except Exception:
                    pass
        print("[Watchdog] Stopped.")

    def aim_from_detection(self):
        """Reads detection data from shared buffer and controls turret aim."""
        self.trigger_ready.wait()
        print("[CIWS] Aiming thread started.")
        FIRE_AREA_THRESHOLD = 100_000
        COOLDOWN_SECONDS = 3
        last_shot_time = 0
        target_visible = False

        while self.running:
            if hasattr(self, 'watchdog') and self.watchdog:
                self.watchdog.beat("aim")            
            try:
                data = self.detection_buffer.read()
                if data and data.get("center_x") is not None:
                    cx = data.get("center_x")
                    cy = data.get("center_y")
                    width = data.get("width")
                    height = data.get("height")

                    self.trigger.aim(cx, cy)
                    target_visible = True

                    if width and height:
                        area = width * height
                        now = time.time()
                        if area >= FIRE_AREA_THRESHOLD and now - last_shot_time >= COOLDOWN_SECONDS:
                            if getattr(self.trigger, 'mode', 'auto') == 'auto':
                                print(f"[CIWS] Auto-firing at target (area={area}).")
                                self.trigger.shoot_auto()
                                last_shot_time = now
                else:
                    if target_visible:
                        print("[CIWS] No target detected. Resetting aim to center.")
                        self.trigger.center()
                        target_visible = False

            except Exception as e:
                print(f"[CIWS] Aim error: {e}")

            time.sleep(0.05)

    def run(self):
        print("[CIWS] Launching subsystems...")
        self.navigation_thread.start()
        self.detector_thread.start()
        self.watchdog_thread.start()

        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        print("[CIWS] Shutting down...")
        self.running = False

        # 1. Stop the detector first (closes OpenCV windows on its own thread)
        try:
            self.detector.stop()
        except Exception as e:
            print(f"[CIWS] Detector stop error: {e}")

        # 2. Give the detector thread a moment to close its windows cleanly
        if self.detector_thread.is_alive():
            self.detector_thread.join(timeout=2.0)

        # 3. Shutdown navigation (stops motors AND LiDAR motor, closes serial port)
        try:
            if hasattr(self, 'navigation') and self.navigation:
                self.navigation.shutdown()
        except Exception as e:
            print(f"[CIWS] Navigation shutdown error: {e}")

        # 4. Cleanup trigger (deinit servos and trigger relay)
        try:
            if self.trigger:
                self.trigger.cleanup()
        except Exception as e:
            print(f"[CIWS] Trigger cleanup error: {e}")

        # 5. Stop keyboard dispatcher (restores terminal settings)
        try:
            if self.keyboard:
                self.keyboard.stop()
        except Exception as e:
            print(f"[CIWS] Keyboard stop error: {e}")

        print("[CIWS] All systems shutdown.")
        time.sleep(0.3)
        sys.exit(0)


if __name__ == "__main__":
    controller = CIWSControl()
    signal.signal(signal.SIGINT, lambda *_: controller.stop())
    controller.run()



