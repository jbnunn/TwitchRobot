#!/usr/bin/python3

import rospy
import boto3
import time
import sys
import signal
import json
import os

from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from geometry_msgs.msg import Twist

endpoint = os.environ['AWS_IOT_ENDPOINT'] # Set this up in your bash profile

# See https://github.com/aws/aws-iot-device-sdk-python/blob/master/samples/basicPubSub/basicPubSub.py for example code for the following:

createMQTTClient = AWSIoTMQTTClient("TwitchRobotRaspberryPi")
createMQTTClient.configureEndpoint(endpoint, 443)
# Change these certificate names
createMQTTClient.configureCredentials("/home/pi/TwitchRobot/certs/AmazonRootCA1.crt", "/home/pi/TwitchRobot/certs/TwitchRobot.pem.key", "/home/pi/TwitchRobot/certs/TwitchRobot.pem.cert")

createMQTTClient.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
createMQTTClient.configureDrainingFrequency(2)  # Draining: 2 Hz
createMQTTClient.configureConnectDisconnectTimeout(10)  # 10 sec
createMQTTClient.configureMQTTOperationTimeout(5)  # 5 sec

createMQTTClient.connect()

rospy.init_node('alexa', anonymous=True)
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Move Operations
def forward():
    print("moving")
    vel_msg = Twist()
    vel_msg.linear.x = abs(0.5)
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    time.sleep(1)

def spin():
    print("spinning")
    vel_msg = Twist()
    speed = 60
    angle = 360
    PI = 3.14159
    angular_speed = speed * 2 * PI/360
    relative_angle = angle * 2 * PI/360
    
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = abs(angular_speed)

    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
      velocity_publisher.publish(vel_msg)
      t1 = rospy.Time.now().to_sec()
      current_angle = angular_speed * (t1-t0)
   
    stop()

def stop():
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg) # forces stop
    time.sleep(1)

# Interrupt Handler
def interrupt_handler(signum, frame):
    createMQTTClient.unsubscribe("/voice/drive")
    sys.exit()

# Custom MQTT message callback
def driveCallback(client, userdata, message):
    print("Received a new message: ")
    print(message.payload)
    print("from topic: ")
    print(message.topic)
    print("--------------\n\n")

    print("Command:")
    payload = json.loads(message.payload)
    command = payload['data']
    print(command)

    if command == "forward":
        forward()
        stop()
    elif command == "spin":
        spin()

print("Listening on /voice/drive")

createMQTTClient.subscribe("/voice/drive", 1, driveCallback)

while True:
    signal.signal(signal.SIGINT, interrupt_handler)
    time.sleep(1)

createMQTTClient.unsubscribe("/voice/drive")
