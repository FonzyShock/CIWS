import torch
from pathlib import Path
import cv2
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from ultralytics import YOLO
import json # Import for JSON handling
import os   # Import for file path operations (if saving to file)

# Load YOLOv11 model from ultralytics
try:
    model = YOLO("yolo11n.pt") # Or 'yolov8n.pt', 'yolov5s.pt', etc.
except Exception as e:
    print(f"Error loading YOLO model: {e}")
    print("Please ensure 'yolo11n.pt' is in the correct directory or replace it with a valid model name (e.g., 'yolov8n.pt').")
    exit()

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        print("Please ensure your webcam is connected and not in use by another application.")
        return

    cv2.namedWindow("YOLO Live Feed", cv2.WINDOW_NORMAL)

    # Define crosshair parameters
    crosshair_size = 10
    crosshair_color = (255, 0, 0) # Blue color (B, G, R)
    crosshair_thickness = 2

    # Define the class ID for "person" (COCO dataset)
    PERSON_CLASS_ID = 0

    # Define the output file path for the largest detection data
    OUTPUT_DATA_FILE = "largest_person_detection.json"

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame. Exiting...")
            break

        # Perform object detection, filtering only for the 'person' class
        # (This is the most efficient way to only get 'person' detections)
        results_generator = model(frame, stream=True, classes=PERSON_CLASS_ID)

        # Initialize variables to track the largest person detection
        largest_person_detection = None
        max_area = 0

        for r in results_generator:
            # r.plot() draws all detections in 'r', even if we later filter one out for output.
            # If you only want to draw the largest, you'd plot manually.
            annotated_frame = r.plot()

            if r.boxes: # Check if any bounding boxes (persons) were detected
                # Get all boxes in xyxy format to calculate area easily
                boxes_xyxy = r.boxes.xyxy.cpu().numpy()
                confidences = r.boxes.conf.cpu().numpy()
                class_ids = r.boxes.cls.cpu().numpy()

                for i, bbox in enumerate(boxes_xyxy):
                    x1, y1, x2, y2 = bbox
                    width = x2 - x1
                    height = y2 - y1
                    area = width * height

                    # Since we filtered by classes=PERSON_CLASS_ID, all detections here are persons
                    # We just need to find the largest one.
                    if area > max_area:
                        max_area = area
                        # Store the data for the largest person
                        largest_person_detection = {
                            "center_x": int((x1 + x2) / 2),
                            "center_y": int((y1 + y2) / 2),
                            "width": int(width),
                            "height": int(height),
                            "confidence": float(confidences[i]), # Convert to float for JSON compatibility
                            "class_id": int(class_ids[i]),
                            "label": model.names[int(class_ids[i])]
                        }
                        # Optional: Draw a distinct marker for the largest person
                        # For demonstration, we'll draw for all, then highlight the largest.
                        # If you want to draw *only* the largest, move drawing logic here.

            # After processing all detections for the current frame,
            # If a largest person was found, print its data and send it.
            if largest_person_detection:
                print("\n--- Largest Person Detected ---")
                print(f"Label: {largest_person_detection['label']}")
                print(f"Center: ({largest_person_detection['center_x']}, {largest_person_detection['center_y']})")
                print(f"Size (WxH): ({largest_person_detection['width']}, {largest_person_detection['height']})")
                print(f"Confidence: {largest_person_detection['confidence']:.2f}")
                print("-------------------------------\n")

                # --- Draw distinct crosshair for the largest person ---
                center_x_largest = largest_person_detection['center_x']
                center_y_largest = largest_person_detection['center_y']
                largest_color = (0, 0, 255) # Red color for largest

                # Horizontal line
                cv2.line(annotated_frame, (center_x_largest - crosshair_size, center_y_largest),
                         (center_x_largest + crosshair_size, center_y_largest),
                         largest_color, crosshair_thickness + 2) # Thicker for emphasis
                # Vertical line
                cv2.line(annotated_frame, (center_x_largest, center_y_largest - crosshair_size),
                         (center_x_largest, center_y_largest + crosshair_size),
                         largest_color, crosshair_thickness + 2)

                # Add text for coordinates
                cv2.putText(annotated_frame,
                            f"Largest: ({center_x_largest}, {center_y_largest})",
                            (center_x_largest + crosshair_size + 5, center_y_largest - crosshair_size - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, largest_color, 2, cv2.LINE_AA)

                # --- Send Data to Another Script (Method: JSON File) ---
                try:
                    with open(OUTPUT_DATA_FILE, 'w') as f:
                        json.dump(largest_person_detection, f, indent=4)
                    # print(f"Largest person data saved to {OUTPUT_DATA_FILE}")
                except Exception as e:
                    print(f"Error saving data to JSON file: {e}")

                # --- Alternative for IPC (e.g., Sockets - conceptual) ---
                # This part is commented out, but shows how you'd prepare data for IPC
                # import socket # Would need to import at the top
                # try:
                #     # Example: Send via UDP socket (fire and forget)
                #     # You'd need to set up a UDP server in the other script
                #     UDP_IP = "127.0.0.1"
                #     UDP_PORT = 5005
                #     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                #     message = json.dumps(largest_person_detection).encode('utf-8')
                #     sock.sendto(message, (UDP_IP, UDP_PORT))
                #     sock.close()
                # except Exception as e:
                #     print(f"Error sending data via socket: {e}")
                # --------------------------------------------------------

            else:
                # If no persons were detected, you might want to clear the file or send a "no detection" signal
                # For this example, we'll just print a message.
                print("No persons detected in this frame.")
                # Optional: Clear the file if no person is detected to signal nothing is there.
                # with open(OUTPUT_DATA_FILE, 'w') as f:
                #     f.write("{}") # Write an empty JSON object or specific "no detection" flag

            cv2.imshow("YOLO Live Feed", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Exiting program.")
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()