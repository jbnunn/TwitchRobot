#!/usr/bin/python3

import boto3
import json
import os
import rospy
import signal
import sys
import time

from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient

from CameraImage import *
from Drive import *

AWS_IOT_ENDPOINT = os.environ['AWS_IOT_ENDPOINT'] # Set this environment variable up in your bash/zsh profile, eg `EXPORT AWS_IOT_ENDPOINT=a2xxxxxxxxxxx-ats.iot.us-east-1.amazonaws.com`

# See https://github.com/aws/aws-iot-device-sdk-python/blob/master/samples/basicPubSub/basicPubSub.py
createMQTTClient = AWSIoTMQTTClient("TwitchRobotRaspberryPi")
createMQTTClient.configureEndpoint(AWS_IOT_ENDPOINT, 443)

# Change these certificate names
createMQTTClient.configureCredentials("/home/pi/TwitchRobot/certs/AmazonRootCA1.crt", "/home/pi/TwitchRobot/certs/TwitchRobot.pem.key", "/home/pi/TwitchRobot/certs/TwitchRobot.pem.crt")
createMQTTClient.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
createMQTTClient.configureDrainingFrequency(2)  # Draining: 2 Hz
createMQTTClient.configureConnectDisconnectTimeout(10)  # 10 sec
createMQTTClient.configureMQTTOperationTimeout(5)  # 5 sec

createMQTTClient.connect()

# Initialize the node and classes
rospy.init_node('alexa', anonymous=True)
drive = Drive()

def unsubscribe_topics():
    """Unbsubscribes from AWS IoT topics before exiting
    """

    topics = [
        '/voice/drive',
        '/camera'
    ]

    for topic in topics:
        createMQTTClient.unsubscribe(topic)

# Interrupt Handler to break out of the script after unsubscribing from topics
def interrupt_handler(signum, frame):
    unsubscribe_topics()
    sys.exit("Exited and unsubscribed")

# Custom MQTT message callbacks
def driveCallback(client, userdata, message):
    print(f"Received {message.payload} from {message.topic}")
    payload = json.loads(message.payload)
    directive = payload['directive']
    print(f"Processing directive: {directive}")
    
    if directive == "forward":
        drive.forward()
        drive.stop()
    elif directive == "spin":
        drive.spin()

def cameraImageCallback(client, userdata, message):
    """Takes a picture, sends to Amazon Rekognition, updates the AWS IoT shadow so Alexa can describe the picture
    """
    print(f"Received {message.payload} from {message.topic}")
    payload = json.loads(message.payload)
    
    directive = payload['directive']

    # Grab (or create) session_id to store Rekognition response into DDB so ASK can can give response
    if 'data' in payload:
        if 'session_id' in payload['data']:
            session_id = payload['data']['session_id']
        else:
            session_id = "LOCAL"

    print(f"Processing directive: {directive}")
    
    if directive == "take picture" or directive == "take a picture" or directive == "picture":
        filename = f"/tmp/{time.time()}.jpg"
        cameraImage = CameraImage(filename) 
  
        # Look for known faces/people first
        cameraImage.recognize_people()
        
        # Look for objects to label
        cameraImage.detect_labels()
        
        # Save this scene so Alexa can return the results
        result = cameraImage.save_scene(session_id)

        print(result)

    else:
        print("Unrecognized command")

# Subscribe to topics
createMQTTClient.subscribe("/voice/drive", 1, driveCallback)
print("Listening on /voice/drive")

createMQTTClient.subscribe("/camera", 1, cameraImageCallback)
print("Listening on /camera")

while True:
    signal.signal(signal.SIGINT, interrupt_handler)
    time.sleep(1)

unsubscribe_topics()