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
# Python Libraries
# import cv2

# Python Functions
# Define Functions Here


class VisionSystem:
    def __init__(self, frame_width=640, h_fov_deg=60):
        """
        Initializes the camera feed and any associated detection models.
         Args: frame_width (int). width of camera frame in pixels
               h_fov_deg (float). horizontal field of view of the camera in degrees
        """

        # Example for OpenCV camera:
        # self.cap = cv2.VideoCapture(0)
        self.frame_width = frame_width
        self.h_fov = h_fov_deg
        print(f"Vision System - Initialized with HFOV {self.h_fov}° and frame width {self.frame_width}px.")

    # TODO: Replace with OpenCV code
    def get_frame(self):
        """
        Captures a frame from the camera.
        Returns: frame. An image (as a NumPy array or other) or None if not implemented
        """
        # Example for OpenCV:
        # ret, frame = self.cap.read()
        # return frame if ret else None

        return None  # Replace with camera feed logic

    def get_targets(self, frame):
        """
        Detects and classifies targets from the input frame.
        Args: frame. an image frame from the camera
        Returns: list. each target is represented as
                {
                    "label": str – class of the detected object ("friendly" or "enemy"),
                    "bbox": (x, y, w, h) – bounding box coordinates
                }
        """
        # TODO: Plug in object detection logic (YOLO))
        return [{
            "label": "enemy",
            "bbox": (100, 100, 50, 50)
        }]

    def get_relative_azimuth(self, bbox):
        """
        Calculates the relative azimuth angle to the target in degrees based on its pixel position.
        Args: bbox (tuple) - bounding box (x, y, w, h)
        Returns: float. relative angle from center (left negative, right positive)
        """
        x, _, w, _ = bbox
        target_center_x = x + w / 2
        pixel_offset = target_center_x - (self.frame_width / 2)
        angle_per_pixel = self.h_fov / self.frame_width
        return pixel_offset * angle_per_pixel
