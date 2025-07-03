"""Example of a camera service client."""
# Copyright (c) farm-ng, inc.
#
# Licensed under the Amiga Development Kit License (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://github.com/farm-ng/amiga-dev-kit/blob/main/LICENSE
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import annotations

import argparse
import asyncio
from pathlib import Path
import socket
import json


import cv2
import numpy as np
import mediapipe as mp

from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.core.stamp import get_stamp_by_semantics_and_clock_type
from farm_ng.core.stamp import StampSemantics


async def main(service_config_path: Path) -> None:
    """
    Main async function to run camera client with pose detection and UDP tracking data sending.

    Args:
        service_config_path (Path): Path to camera service config JSON.
    """
    # Load the camera service configuration from JSON file
    config: EventServiceConfig = proto_from_json_file(service_config_path, EventServiceConfig())

    # Initialize MediaPipe pose detector with low model complexity for speed
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils
    pose = mp_pose.Pose(
        static_image_mode=False,        # Video stream mode
        model_complexity=0,             # Fast, less accurate model
        min_detection_confidence=0.5,  # Confidence thresholds
        min_tracking_confidence=0.5,
    )

    # Setup UDP socket to send tracking data (bounding box info)
    UDP_IP = "127.0.0.1"  # localhost
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Subscribe asynchronously to the camera event stream
    async for event, message in EventClient(config).subscribe(config.subscriptions[0], decode=True):
        # Extract a reliable timestamp (for debugging)
        stamp = (
            get_stamp_by_semantics_and_clock_type(event, StampSemantics.DRIVER_RECEIVE, "monotonic")
            or event.timestamps[0].stamp
        )
        print(f"Timestamp: {stamp}\n")
        print(f"Meta: {message.meta}")
        print("###################\n")

        # Decode image data bytes to OpenCV numpy array
        image = cv2.imdecode(np.frombuffer(message.image_data, dtype="uint8"), cv2.IMREAD_UNCHANGED)

        # Optional: if this is a disparity image, apply a color map for visualization
        if event.uri.path == "/disparity":
            image = cv2.applyColorMap(image * 3, cv2.COLORMAP_JET)

        # Convert BGR OpenCV image to RGB for MediaPipe processing
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Run pose detection on the RGB image
        results = pose.process(image_rgb)

        if results.pose_landmarks:
            # Draw pose landmarks and connections on original BGR image
            mp_drawing.draw_landmarks(
                image,
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS,
                mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2),
            )

            # Get image dimensions for converting normalized coordinates
            h, w, _ = image.shape

            # Convert normalized landmarks (x,y) to pixel coordinates (numpy array)
            landmark_array = np.array([(lm.x * w, lm.y * h) for lm in results.pose_landmarks.landmark])

            # Calculate bounding box corners from landmarks
            x_min, y_min = np.min(landmark_array, axis=0).astype(int)
            x_max, y_max = np.max(landmark_array, axis=0).astype(int)

            # Add padding and clamp to image size
            padding = 10
            x_min = max(x_min - padding, 0)
            y_min = max(y_min - padding, 0)
            x_max = min(x_max + padding, w)
            y_max = min(y_max + padding, h)

            # Draw a bounding box rectangle (blue)
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)

            # Prepare bounding box center and width for sending
            bbox_center_x = (x_min + x_max) / 2
            bbox_width = x_max - x_min

            # Convert values to native Python types for JSON serialization
            data_dict = {
                "bbox_center_x": float(bbox_center_x),  # Convert numpy float to Python float
                "bbox_width": float(bbox_width),
                "frame_width": int(w),                  # Convert numpy int to Python int
            }

            # Serialize dict to JSON string
            data_json = json.dumps(data_dict)

            # Send JSON data via UDP socket to vehicle twist listener
            sock.sendto(data_json.encode("utf-8"), (UDP_IP, UDP_PORT))

        # Display the image frame with pose and bounding box overlays
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        cv2.imshow("image", image)
        cv2.waitKey(1)

    # Clean up MediaPipe resources if loop exits
    pose.close()




if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="python main.py",
        description="Camera client with pose detection and UDP bounding box sender"
    )
    parser.add_argument("--service-config", type=Path, required=True, help="Path to camera service config JSON")
    args = parser.parse_args()

    asyncio.run(main(args.service_config))
