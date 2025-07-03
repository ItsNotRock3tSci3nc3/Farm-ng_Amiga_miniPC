import asyncio
from pathlib import Path
from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file

async def send_test_twist():
    config = proto_from_json_file(Path("service_config.json"), EventServiceConfig())
    client = EventClient(config)

    twist = Twist2d()
    twist.linear_velocity_x = 0.2  # move forward at 0.2 m/s
    twist.angular_velocity = 0.0

    # Try "/twist" first, change to "/cmd_vel" if no response
    response = await client.request_reply("/twist", twist)
    print("Response for /twist:", response)

    response2 = await client.request_reply("/cmd_vel", twist)
    print("Response for /cmd_vel:", response2)

asyncio.run(send_test_twist())