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
import json
import socket
from pathlib import Path

import cv2
from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file

# Set your base speeds
SET_LINEAR_SPEED_MPS = 0.5
SET_ANGULAR_SPEED_RPS = 0.5

# UDP config (must match camera client)
UDP_IP = "127.0.0.1"
UDP_PORT = 5005


def update_twist_with_key_press(twist: Twist2d, key: int) -> Twist2d:
    """Update robot motion based on keyboard (WASD) input."""
    twist.linear_velocity_x = 0.0
    twist.angular_velocity = 0.0

    if key == ord("w"):
        twist.linear_velocity_x = SET_LINEAR_SPEED_MPS
    elif key == ord("s"):
        twist.linear_velocity_x = -SET_LINEAR_SPEED_MPS
    elif key == ord("a"):
        twist.angular_velocity = SET_ANGULAR_SPEED_RPS
    elif key == ord("d"):
        twist.angular_velocity = -SET_ANGULAR_SPEED_RPS
    elif key == ord(" "):
        pass  # stop motion

    return twist


def update_twist_with_tracking(twist: Twist2d, bbox_center_x: float, frame_width: int) -> Twist2d:
    """Update twist to follow the person based on bounding box center."""
    center_frame = frame_width / 2
    offset = bbox_center_x - center_frame

    # Adjust turn sensitivity
    tolerance = frame_width * 0.05  # only turn if offset is more than 5% of frame width
    angular_scale = 0.003

    twist.linear_velocity_x = SET_LINEAR_SPEED_MPS  # always move forward

    if abs(offset) > tolerance:
        twist.angular_velocity = -angular_scale * offset  # turn to re-center target
    else:
        twist.angular_velocity = 0.0  # go straight

    return twist


async def main(service_config_path: Path) -> None:
    twist = Twist2d()
    follow_mode = False  # Start in manual mode

    # Set up OpenCV window to capture keyboard input
    cv2.namedWindow('Virtual Keyboard')

    # Load canbus config
    config: EventServiceConfig = proto_from_json_file(service_config_path, EventServiceConfig())
    client: EventClient = EventClient(config)

    # Set up UDP socket to receive tracking data from camera client
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)

    print("WASD = move | Space = stop | F = toggle follow mode | Q = quit")
    print("Starting in manual mode.")

    while True:
        key = cv2.waitKey(50) & 0xFF

        # Quit
        if key == ord("q"):
            print("Quitting...")
            break

        # Toggle follow mode
        if key == ord("f"):
            follow_mode = not follow_mode
            print("Follow mode:", "ON" if follow_mode else "OFF")

        # Person-following logic
        if follow_mode:
            try:
                data, _ = sock.recvfrom(1024)
                tracking = json.loads(data.decode("utf-8"))
                bbox_center_x = tracking["bbox_center_x"]
                frame_width = tracking["frame_width"]
                twist = update_twist_with_tracking(twist, bbox_center_x, frame_width)
            except BlockingIOError:
                # No data received — stay in last state
                twist.linear_velocity_x = 0.0
                twist.angular_velocity = 0.0
        else:
            # Manual keyboard movement
            twist = update_twist_with_key_press(twist, key)

        # Send command to Amiga
        response = await client.request_reply("/twist", twist)
        print(f"Velocity: linear={twist.linear_velocity_x:.3f}, angular={twist.angular_velocity:.3f}")

        await asyncio.sleep(0.05)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="python main.py",
        description="Twist controller for Amiga with keyboard + tracking toggle.",
    )
    parser.add_argument("--service-config", type=Path, required=True, help="The canbus service config.")
    args = parser.parse_args()

    asyncio.run(main(args.service_config))