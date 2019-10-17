#!/usr/bin/python3

import rospy
import boto3
import sys
import signal
import time
import json
import os
from io import BytesIO
from PIL import Image

from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient

from Drive import *
from picamera import PiCamera

endpoint = os.environ['AWS_IOT_ENDPOINT'] # Set this up in your bash profile

collectionId = 'family_and_friends'

# See https://github.com/aws/aws-iot-device-sdk-python/blob/master/samples/basicPubSub/basicPubSub.py
createMQTTClient = AWSIoTMQTTClient("TwitchRobotRaspberryPi")
createMQTTClient.configureEndpoint(endpoint, 443)

# Change these certificate names
createMQTTClient.configureCredentials("/home/pi/TwitchRobot/certs/AmazonRootCA1.crt", "/home/pi/TwitchRobot/certs/TwitchRobot.pem.key", "/home/pi/TwitchRobot/certs/TwitchRobot.pem.crt")
createMQTTClient.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
createMQTTClient.configureDrainingFrequency(2)  # Draining: 2 Hz
createMQTTClient.configureConnectDisconnectTimeout(10)  # 10 sec
createMQTTClient.configureMQTTOperationTimeout(5)  # 5 sec

createMQTTClient.connect()

# Initialize the node
rospy.init_node('alexa', anonymous=True)
drive = Drive()

# Initialize the camera, since it takes a couple of seconds for the camera to adjust to light levels
camera = PiCamera()
camera.rotation = 180

# Bucket for saving files
bucket = "jeffnunn"

# Initialize Boto clients
s3_client = boto3.client('s3')
rekognition_client = boto3.client('rekognition')
dynamodb_client = boto3.client('dynamodb')
dynamodb_resource = boto3.resource('dynamodb')

def unsubscribe_topics():
    """Unbsubscribes from AWS IoT topics before exiting
    """

    topics = [
        '/voice/drive',
        '/camera'
    ]

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

def upload_image_to_s3(filename):
    """Uploads an image to S3, prepends timestamp with "twitchrobot", e.g.,

        s3://jeffnunn/twitchrobot/1234.5678.jpg
    """
    response = s3_client.upload_file(f"twitchrobot/{filename}", bucket, f"twitchrobot/{filename}")
    print(response)
    
def detect_faces(filename):
    """Send an image to Amazon Rekognition for analysis
    """
    print("Looking for faces")
    with open(filename, 'rb') as image:
        try:
            response = rekognition_client.search_faces_by_image(CollectionId = collectionId, Image={'Bytes': image.read()})
        except:
            print("No faces detected")
            response = None
   
    return response

def cameraPhotoCallback(client, userdata, message):
    """Takes a picture, sends to Amazon Rekognition, updates the AWS IoT shadow so Alexa can describe the picture
    """
    print(f"Received {message.payload} from {message.topic}")
    payload = json.loads(message.payload)
    command = payload['directive']
    data = payload['data']
    print(f"Processing command: {command}")
    
    if command == "take a picture":
        filename = f"/tmp/{time.time()}.jpg"
        camera.capture(filename)
        print(f"Saved {filename}")

        response = detect_faces(filename)
        print(f"Rekognition response: {response}")

        if response:
            faceMatches=response['FaceMatches']
            print("Found " + str(len(faceMatches)) + " face(s).")
            people = []
            for match in faceMatches:
                    print ('FaceId:' + match['Face']['FaceId'])
                    print ('Similarity: ' + "{:.2f}".format(match['Similarity']) + "%")

                    # Lookup the person name
                    face = dynamodb_client.get_item(
                        TableName='rekognition_family_and_friends',  
                        Key={'RekognitionId': {'S': match['Face']['FaceId']}}
                        )
                    
                    if 'Item' in face:
                        people.append(face['Item']['FullName']['S'])
                        print ("Found:", face['Item']['FullName']['S'])
                    else:
                        print ('no match found in person lookup')
            
            people = list(set(people))
            print(f"Recognized {len(people)} people:", people)

            # Update the DDB table with this data
            if 'session_id' in data:
                table = dynamodb_resource.Table('TwitchRobot_Scenes')
                try:
                    response = table.put_item(
                        Item={
                                'session_id': data['session_id'],
                                'people': people
                            }
                    )
                    print("DynamoDB response: ", response)
                except ClientError as e:
                    if e.response['Error']['Code'] == 'EntityAlreadyExists':
                        print("User already exists")
                    else:
                        print("Unexpected error: %s" % e)
    else:
        print("Unrecognized command")

# Subscribe to topics
createMQTTClient.subscribe("/voice/drive", 1, driveCallback)
print("Listening on /voice/drive")

createMQTTClient.subscribe("/camera", 1, cameraPhotoCallback)
print("Listening on /camera")

while True:
    signal.signal(signal.SIGINT, interrupt_handler)
    time.sleep(1)

unsubscribe_topics()