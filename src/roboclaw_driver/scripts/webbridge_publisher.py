#!/usr/bin/python3

## script to connect to ros instance via rosbridge websocket

import roslibpy

topic = '/motors'
msg_type = 'std_msgs/Int32MultiArray'

client = roslibpy.Ros(host='localhost', port=9090)
client.run()
talker = roslibpy.Topic(client, topic, msg_type)

while(client.is_connected):
    val = input("Enter val to publish: ")
    talker.publish(roslibpy.Message({'data': [0, 0, int(val)]}))

print("Client Disconnected... Exiting")
