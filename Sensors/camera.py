# !/usr/bin/python3
# camera.py created by alfon at 5/23/2025

"""
Module: camera.py
Subsystem: Vision / Target Detection

Description:
    Provides an interface for the onboard camera system, responsible for capturing images
    and performing target detection and classification using computer vision techniques.

    The VisionSystem class encapsulates all logic for frame capture and detection, enabling
    external scripts (e.g., main.py) to call its methods without worrying about the underlying
    vision model or camera API.

Usage:
    from Sensors.camera import Vision
    cam = VisionSystem()
    frame = cam.get_frame()
    targets = cam.detect_and_classify_targets(frame)
"""
# person_detector.py
import torch
import cv2
import numpy as np
import pandas as pd
from ultralytics import YOLO
import multiprocessing
import time
from pathlib import Path # Still useful for model paths if needed

# Import the receiver functions

class PersonDetector:
    def __init__(self, model_path="yolo11n.pt", camera_id=0):
        """
        Initializes the PersonDetector.

        Args:
            model_path (str): Path to the YOLO model file (e.g., "yolo11n.pt").
            camera_id (int): The ID of the camera to use (e.g., 0 for default webcam).
        """
        self.model_path = model_path
        self.camera_id = camera_id
        self.model = None
        self.cap = None
        self.parent_conn = None
        self.child_conn = None
        self.receiver_proc = None
        self.running = False
        self.last_sent_detection_data = None # To avoid sending redundant data

        # Define detection parameters
        self.PERSON_CLASS_ID = 0 # For COCO dataset

        # Define drawing parameters (can be made configurable if needed)
        self.crosshair_size = 10
        self.crosshair_color = (0, 255, 255) # Yellow for largest person
        self.crosshair_thickness = 3


    def _load_model(self):
        """Loads the YOLO model."""
        try:
            self.model = YOLO(self.model_path)
            print(f"[{self.__class__.__name__}] YOLO model '{self.model_path}' loaded successfully.")
        except Exception as e:
            print(f"[{self.__class__.__name__}] Error loading YOLO model: {e}")
            print(f"[{self.__class__.__name__}] Please ensure '{self.model_path}' is in the correct directory or is a valid model name.")
            self.model = None
            raise RuntimeError(f"Failed to load YOLO model: {e}")

    def _open_camera(self):
        """Opens the webcam."""
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            print(f"[{self.__class__.__name__}] Error: Could not open webcam (ID: {self.camera_id}).")
            print(f"[{self.__class__.__name__}] Please ensure your webcam is connected and not in use by another application.")
            self.cap = None
            raise RuntimeError(f"Failed to open camera ID: {self.camera_id}")
        print(f"[{self.__class__.__name__}] Webcam (ID: {self.camera_id}) opened successfully.")
        cv2.namedWindow("YOLO Live Feed", cv2.WINDOW_NORMAL) # Create window once

    def start(self):
        """
        Starts the detection process: loads model, opens camera, and spawns the receiver process.
        """
        if self.running:
            print(f"[{self.__class__.__name__}] Detector is already running.")
            return

        try:
            self._load_model()
            self._open_camera()

            # Setup Multiprocessing Pipe
            self.parent_conn, self.child_conn = multiprocessing.Pipe()

            # Start the receiver process
            self.receiver_proc = multiprocessing.Process(target=receiver_process, args=(self.child_conn,))
            self.receiver_proc.start()
            print(f"[{self.__class__.__name__}] Started receiver process with PID: {self.receiver_proc.pid}")

            self.running = True
            print(f"[{self.__class__.__name__}] Detector started. Press 'q' in the video window to stop.")

        except RuntimeError as e:
            print(f"[{self.__class__.__name__}] Initialization failed: {e}")
            self.stop() # Ensure cleanup if start fails
            return False
        return True

    def _process_frame(self, frame):
        """
        Internal method to perform detection on a single frame,
        identify the largest person, and return its data.
        """
        results_generator = self.model(frame, stream=True, classes=self.PERSON_CLASS_ID)

        largest_person_data = None
        max_area = 0

        annotated_frame = frame.copy() # Start with a copy of the frame to draw on

        for r in results_generator:
            # r.plot() returns the annotated frame, so we update it here
            annotated_frame = r.plot()

            if r.boxes: # Check if any bounding boxes (persons) were detected
                boxes_xyxy = r.boxes.xyxy.cpu().numpy()
                confidences = r.boxes.conf.cpu().numpy()
                class_ids = r.boxes.cls.cpu().numpy()

                for i, bbox in enumerate(boxes_xyxy):
                    x1, y1, x2, y2 = bbox
                    width = x2 - x1
                    height = y2 - y1
                    area = width * height

                    if area > max_area:
                        max_area = area
                        # TODO::Have the center_x and center_y be the inputs to control the x and y actuators for CIWS control
                        largest_person_data = {
                            "center_x": int((x1 + x2) / 2),
                            "center_y": int((y1 + y2) / 2),
                            "width": int(width),
                            "height": int(height),
                            "confidence": float(confidences[i]),
                            "class_id": int(class_ids[i]),
                            "label": self.model.names[int(class_ids[i])]
                        }
        return annotated_frame, largest_person_data

    def run(self):
        """
        The main loop for capturing frames, processing, and sending data.
        This method will block until stopped.
        """
        if not self.running:
            print(f"[{self.__class__.__name__}] Detector is not started. Call .start() first.")
            return

        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                print(f"[{self.__class__.__name__}] Error: Could not read frame. Stopping detector.")
                self.stop()
                break

            annotated_frame, current_frame_largest_person = self._process_frame(frame)

            # Highlight the largest person on the annotated frame
            if current_frame_largest_person:
                center_x_largest = current_frame_largest_person['center_x']
                center_y_largest = current_frame_largest_person['center_y']

                # Draw crosshair
                cv2.line(annotated_frame, (center_x_largest - self.crosshair_size, center_y_largest),
                         (center_x_largest + self.crosshair_size, center_y_largest),
                         self.crosshair_color, self.crosshair_thickness)
                cv2.line(annotated_frame, (center_x_largest, center_y_largest - self.crosshair_size),
                         (center_x_largest, center_y_largest + self.crosshair_size),
                         self.crosshair_color, self.crosshair_thickness)

                # Add text for coordinates
                cv2.putText(annotated_frame,
                            f"Lg:({center_x_largest}, {center_y_largest})",
                            (center_x_largest + self.crosshair_size + 5, center_y_largest - self.crosshair_size - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.crosshair_color, 2, cv2.LINE_AA)

                # Send data only if it has changed
                if current_frame_largest_person != self.last_sent_detection_data:
                    try:
                        self.parent_conn.send(current_frame_largest_person)
                        self.last_sent_detection_data = current_frame_largest_person
                        # print(f"[{self.__class__.__name__}] Data sent: {current_frame_largest_person['center_x']}, {current_frame_largest_person['center_y']}")
                    except Exception as e:
                        print(f"[{self.__class__.__name__}] Error sending data through pipe: {e}")
            else:
                # If no persons detected, send a 'None' signal if the last frame had a detection
                if self.last_sent_detection_data is not None:
                    try:
                        self.parent_conn.send(None)
                        self.last_sent_detection_data = None
                        # print(f"[{self.__class__.__name__}] Sent 'no detection' signal.")
                    except Exception as e:
                        print(f"[{self.__class__.__name__}] Error sending 'no detection' signal: {e}")

            cv2.imshow("YOLO Live Feed", annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print(f"[{self.__class__.__name__}] 'q' pressed. Stopping detector.")
                self.stop()
                break

    def stop(self):
        """
        Stops the detector, releases resources, and terminates the receiver process.
        """
        if not self.running:
            print(f"[{self.__class__.__name__}] Detector is not running.")
            return

        print(f"[{self.__class__.__name__}] Stopping detector...")
        self.running = False # Signal the run loop to stop

        if self.cap:
            self.cap.release()
            self.cap = None
        cv2.destroyAllWindows()

        # Close pipe connections and terminate the child process
        if self.parent_conn:
            self.parent_conn.close()
            self.parent_conn = None
        if self.child_conn: # Ensure child_conn is closed even if parent didn't get to use it
            self.child_conn.close()
            self.child_conn = None
        if self.receiver_proc and self.receiver_proc.is_alive():
            self.receiver_proc.join(timeout=2) # Give it a moment to clean up
            if self.receiver_proc.is_alive():
                print(f"[{self.__class__.__name__}] Receiver process did not terminate gracefully, forcing termination.")
                self.receiver_proc.terminate()
            self.receiver_proc = None

        print(f"[{self.__class__.__name__}] Detector stopped successfully.")

# This block is crucial for multiprocessing on Windows
if __name__ == "__main__":
    multiprocessing.freeze_support() # Essential for Windows when creating executables

    # --- Example Usage ---
    detector = PersonDetector(model_path="yolov11n.pt", camera_id=0) # Use yolov8n.pt as it's common

    if detector.start():
        detector.run()
    # No need for explicit detector.stop() here as .run() calls it upon exit
