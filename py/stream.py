from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
import uvicorn
import cv2
import numpy as np
import subprocess
import json
import pynmea2
from fastapi.middleware.cors import CORSMiddleware
import asyncio
from pathlib import Path

# farm-ng imports
from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng_core_pybind import Pose3F64, Isometry3F64
from examples.track_planner.track_planner import TrackBuilder
from farm_ng.track.track_pb2 import Track, TrackFollowRequest
from google.protobuf.empty_pb2 import Empty

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], allow_credentials=True,
    allow_methods=["*"], allow_headers=["*"],
)

# global coordinate storage
latest_coords = {"lat": None, "lng": None, "precision": None}
last_coordinates = []

# ------------- live GPS directly from Emlid (over TCP) -------------

async def gps_listener():
    """
    Continuously listen to NMEA stream from Emlid Reach RS+
    and update latest_coords.
    """
    # adjust Emlid IP and port if needed
    reach_host = "192.168.2.15"
    reach_port = 9000

    while True:
        try:
            reader, _ = await asyncio.open_connection(reach_host, reach_port)
            print(f"[GPS] Connected to Emlid at {reach_host}:{reach_port}")
            while True:
                line = await reader.readline()
                line = line.decode().strip()
                try:
                    msg = pynmea2.parse(line)
                    if isinstance(msg, pynmea2.types.talker.GGA):
                        latest_coords["lat"] = msg.latitude
                        latest_coords["lng"] = msg.longitude
                        latest_coords["precision"] = msg.horizontal_dil #HDOP
                        print(f"[RTP] updated: lat={latest_coords['lat']}, lon={latest_coords['lng']}; with precision of {latest_coords['precision']}")
                except pynmea2.nmea.ParseError:
                    continue
        except Exception as e:
            print(f"[GPS] connection error: {e}, retrying in 5s...")
            await asyncio.sleep(5)

# start background GPS listener
@app.on_event("startup")
async def startup_event():
    asyncio.create_task(gps_listener())

# ------------- serve current robot coordinates -------------
@app.get("/robot_gps")
async def get_latest():
    print(f"Latest coordinates: {latest_coords}")
    return latest_coords

# ------------- WebSocket camera stream -------------
@app.websocket("/ws/camera/{camera_id}")
async def camera_ws(websocket: WebSocket, camera_id: int):
    await websocket.accept()
    if camera_id == 1:
        config_path = "~/py/examples/camera_client/service_config1.json"
    elif camera_id == 2:
        config_path = "~/py/examples/camera_client/service_config.json"
    else:
        await websocket.close()
        return
    config = proto_from_json_file(config_path, EventServiceConfig())
    try:
        while True:
            async for _, message in EventClient(config).subscribe(config.subscriptions[0], decode=True):
                image = cv2.imdecode(np.frombuffer(message.image_data, dtype="uint8"), cv2.IMREAD_UNCHANGED)
                image = cv2.resize(image, (600, 460))
                _, jpeg = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                await websocket.send_bytes(jpeg.tobytes())
    except WebSocketDisconnect:
        print("WebSocket disconnected")
    except Exception as e:
        print(f"WebSocket error: {e}")
    finally:
        await websocket.close()

# ------------- coordinates from NextJS -------------
@app.post("/coordinates")
async def receive_coordinates(request: Request):
    global last_coordinates
    data = await request.json()
    last_coordinates = data
    print(f"Received coordinates: {data}")
    return {"status": "ok"}

# ------------- driving logic -------------
@app.post("/run")
async def run_farmng(request: Request):
    if not last_coordinates:
        return {"status": "no coordinates"}

    try:
        # convert last_coordinates [{lat, lng}, ...] to farm-ng Poses
        poses = []
        for pt in last_coordinates:
            # You *must* convert lat/lng to local map coordinates
            # Here is a placeholder, you should substitute your
            # known reference or use a transform if you have one:
            pose = Pose3F64(
                x=pt["lng"],  # placeholder: replace with local-X
                y=pt["lat"],  # placeholder: replace with local-Y
                z=0,
                orientation=Isometry3F64.identity().so3()
            )
            poses.append(pose)

        # build a Track protobuf
        from farm_ng.track.track_pb2 import Track, TrackFollowRequest
        track = Track(waypoints=poses)

        config = proto_from_json_file(
            "~/py/examples/track_planner/service_config.json",
            EventServiceConfig()
        )
        client = EventClient(config)

        # send track to follower
        await client.request_reply("/set_track", TrackFollowRequest(track=track))

        # then start
        from google.protobuf.empty_pb2 import Empty
        await client.request_reply("/start", Empty())

        return {"status": "driving to coordinates"}

    except Exception as e:
        print(f"run_farmng error: {e}")
        return {"status": "error", "detail": str(e)}


@app.post("/set_track")
async def set_track(request: Request):
    coords = await request.json()  # list of { lat, lng }
    poses = []
    for pt in coords:
        # Convert lat/lng → Pose3F64 (local robot frame)
        pose = Pose3F64(...)  # implement conversion
        poses.append(pose)
    track = Track(waypoints=poses)
    config = proto_from_json_file("track_follower_config.json", EventServiceConfig())
    client = EventClient(config)
    await client.request_reply("/set_track", TrackFollowRequest(track=track))
    await client.request_reply("/start", Empty())
    return {"status": "track started"}

@app.post("/stop_track")
async def stop_track():
    config = proto_from_json_file("track_follower_config.json", EventServiceConfig())
    client = EventClient(config)
    await client.request_reply("/cancel", Empty())
    return {"status": "track stopped"}

if __name__ == "__main__":
    uvicorn.run("stream:app", host="0.0.0.0", port=8000, reload=False)