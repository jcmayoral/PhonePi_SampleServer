#!/usr/bin/env python

import asyncio
import websockets
import socket
import json
from sensor_msgs.msg import Imu
import rospy
from tf.transformations import quaternion_from_euler

rospy.init_node('android_Sensor')
publisher = rospy.Publisher("android/imu", Imu, queue_size=1)
msg = Imu()
msg.header.frame_id = "imu"

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
        msg.header.stamp = rospy.Time.now()
        if path == '//accelerometer':
            data = await websocket.recv()
            djson = json.loads(data)
            msg.linear_acceleration.x = float(djson['dataX'])
            msg.linear_acceleration.y = float(djson['dataY'])
            msg.linear_acceleration.z = float(djson['dataZ'])

        if path == '//gyroscope':
            data = await websocket.recv()
            djson = json.loads(data)
            msg.angular_velocity.x = float(djson['dataX'])
            msg.angular_velocity.y = float(djson['dataY'])
            msg.angular_velocity.z = float(djson['dataZ'])
      
        if path == '//orientation':
            data = await websocket.recv()
            djson = json.loads(data)
            q = quaternion_from_euler(float(djson['roll']), 
                                float(djson['pitch']), 
                                float(djson['azimuth']))
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
            #print(djson['roll'])
        publisher.publish(msg)

asyncio.get_event_loop().run_until_complete(
    websockets.serve(echo, '0.0.0.0', 5000))
asyncio.get_event_loop().run_forever()
