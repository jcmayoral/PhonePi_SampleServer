#!/usr/bin/env python

import asyncio
import websockets
import socket
import json
from sensor_msgs.msg import Imu
import rospy

rospy.init_node('android_Sensor')
publisher = rospy.Publisher("android_imu", Imu, 1)
msg = Imu()

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP


hostname = socket.gethostname()
IPAddr = get_ip()
print("Your Computer Name is: " + hostname)
print("Your Computer IP Address is: " + IPAddr)
print("Enter {0}:5000 in the app [PhonePi] and select the sensors to stream. For PhonePi+ just enter {0}, without the port".format(IPAddr))


async def echo(websocket, path):
    async for message in websocket:
        if path == '//accelerometer':
            data = await websocket.recv()
            djson = json.loads(data)
            print(data)

        if path == '//gyroscope':
            data = await websocket.recv()
            djson = json.loads(data)
            print(data)
      
        if path == '//orientation':
            data = await websocket.recv()
            print("A")
            djson = json.loads(data)
            print(data)
            print(djson['roll'])

asyncio.get_event_loop().run_until_complete(
    websockets.serve(echo, '0.0.0.0', 5000))
asyncio.get_event_loop().run_forever()
