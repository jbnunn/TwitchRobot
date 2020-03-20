#!/usr/bin/python3

import rospy
import boto3
import sys
import signal
import time
import json
import os
from io import BytesIO

from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient

from Drive import *

endpoint = os.environ['AWS_IOT_ENDPOINT'] # Set this up in your bash profile

# See https://github.com/aws/aws-iot-device-sdk-python/blob/master/samples/basicPubSub/basicPubSub.py
createMQTTClient = AWSIoTMQTTClient("TwitchRobotRaspberryPi")
createMQTTClient.configureEndpoint(endpoint, 443)

createMQTTClient.configureCredentials("/home/pi/TwitchRobot/certs/AmazonRootCA1.crt", "/home/pi/TwitchRobot/certs/TwitchRobot.private.key", "/home/pi/TwitchRobot/certs/TwitchRobot.cert.pem")
createMQTTClient.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
createMQTTClient.configureDrainingFrequency(2)  # Draining: 2 Hz
createMQTTClient.configureConnectDisconnectTimeout(10)  # 10 sec
createMQTTClient.configureMQTTOperationTimeout(5)  # 5 sec

createMQTTClient.connect()

# Initialize the node
rospy.init_node('alexa', anonymous=True)
drive = Drive()

def unsubscribe_topics():
    """Unbsubscribes from AWS IoT topics before exiting
    """

    topics = [ '/voice/drive' ]

    for topic in topics:
        createMQTTClient.unsubscribe(topic)

# Interrupt Handler useful to break out of the script
def interrupt_handler(signum, frame):
    unsubscribe_topics()
    sys.exit("Exited and unsubscribed")

# Custom MQTT message callbacks
def driveCallback(client, userdata, message):
    print(f"Received {message.payload} from {message.topic}")
    payload = json.loads(message.payload)
    command = payload['data']
    print(f"Processing command: {command}")
    
    if command == "forward":
        drive.forward()
        drive.stop()
    elif command == "spin":
        drive.spin()

# Subscribe to topics
createMQTTClient.subscribe("/voice/drive", 1, driveCallback)
print("Listening on /voice/drive")

while True:
    signal.signal(signal.SIGINT, interrupt_handler)
    time.sleep(1)

unsubscribe_topics()
