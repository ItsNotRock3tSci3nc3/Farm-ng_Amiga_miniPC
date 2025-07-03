import subprocess

'''
Program to launch multiple camera, live stream, and vehicle_twist in parallel.
'''


processes = [
    ["python", "/home/paalab/Desktop/farm-ng-amiga/py/examples/camera_client/main.py", "--service-config", "/home/paalab/Desktop/farm-ng-amiga/py/examples/camera_client/service_config1.json"],
    ["python", "/home/paalab/Desktop/farm-ng-amiga/py/stream.py"],
    ["python", "/home/paalab/Desktop/farm-ng-amiga/py/examples/vehicle_twist/main.py", "--service-config", "/home/paalab/Desktop/farm-ng-amiga/py/examples/vehicle_twist/service_config.json"],
]

procs = []
try:
    for cmd in processes:
        procs.append(subprocess.Popen(cmd))
    for p in procs:
        p.wait()
except KeyboardInterrupt:
    print("Shutting down...")
    for p in procs:
        p.terminate()