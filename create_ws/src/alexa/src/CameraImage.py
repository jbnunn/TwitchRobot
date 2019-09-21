import boto3
import io
import rospy
import sys

from io import BytesIO
from picamera import PiCamera
from PIL import Image, ImageDraw, ExifTags, ImageColor, ImageFont

AWS_REKOGNITION_COLLECTION = 'family_and_friends'

# Initialize boto resources
s3_client = boto3.client('s3')
rekognition_client = boto3.client('rekognition')
dynamodb_resource = boto3.resource('dynamodb')
dynamodb_client = boto3.client('dynamodb')

# Initialize the camera, since it takes a couple of seconds for the camera to adjust to light levels
camera = PiCamera()
camera.rotation = 180

# Load font for labels
font = ImageFont.truetype("./Amazon-Ember-Regular.ttf",12)

class CameraImage():
    """Functions to interact with images from Raspberry Pi camera
    """
    def __init__(self, filename):

        camera.capture(filename) # Saves the image to a file

        self.image = Image.open(filename)
        self.annotated_image = self.image
        self.bucket = "jeffnunn-public" # change to your own bucket
        self.draw = ImageDraw.Draw(self.annotated_image)
        self.filename = filename
        self.labels = []
        self.label_confidence_threshold = 90
        self.recognized_people = []
       
    def upload_annotated_image_to_s3(self, filename):
        """Uploads an annotated (labeled) image to S3.
        """
        response = s3_client.upload_file(filename, self.bucket, filename, ExtraArgs={'ACL':'public-read'})
    
        return response
    
    def recognize_people(self):
        """Sends an image to Amazon Rekognition to do facial recognition
        """

        with open(self.filename, 'rb') as image:
            response = rekognition_client.search_faces_by_image(CollectionId = AWS_REKOGNITION_COLLECTION, Image={'Bytes': image.read()})
            
        if response:

            imgWidth, imgHeight = self.annotated_image.size
            
            for match in response['FaceMatches']:
                
                # Lookup the person's name
                face = dynamodb_client.get_item(
                    TableName='rekognition_family_and_friends',  
                    Key={'RekognitionId': {'S': match['Face']['FaceId']}}
                )
                
                if 'Item' in face:

                    if face['Item']['FullName']['S'] not in self.recognized_people:
                        person = face['Item']['FullName']['S']
                        print(f"I recognized {person}")
                        self.recognized_people.append(person)

                        # Get details for annotation
                        box = response['SearchedFaceBoundingBox']
                        left = imgWidth * box['Left']
                        top = imgHeight * box['Top']
                        width = imgWidth * box['Width']
                        height = imgHeight * box['Height']

                        points = (
                            (left,top),
                            (left + width, top),
                            (left + width, top + height),
                            (left , top + height),
                            (left, top)
                        )

                        # Outline face
                        self.draw.line(points, fill='#ff9900', width=3)
                        
                        if top <= 30: # add label to bottom of box
                            text_top = top + height + 5
                        else:
                            text_top = top - 20 # add label to top of box
                        
                        # Label face
                        self.draw.text((left, text_top), face['Item']['FullName']['S'], (255, 153, 0), font=font)

            return self.recognized_people

    def detect_labels(self):
        """Sends an image to Amazon Rekognition to detect lables
        """
      
        with open(self.filename, 'rb') as image:
            response = rekognition_client.detect_labels(Image={'Bytes': image.read()}, MaxLabels=5)

        if response:

            imgWidth, imgHeight = self.annotated_image.size
            
            for label in response['Labels']:
                if label['Confidence'] >= self.label_confidence_threshold:

                    self.labels.append(label['Name'])
                    print(f"Label detected: {label}")

                    # Get details for annotation
                    for instance in label['Instances']:
                        box = instance['BoundingBox']
                        left = imgWidth * box['Left']
                        top = imgHeight * box['Top']
                        width = imgWidth * box['Width']
                        height = imgHeight * box['Height']

                        points = (
                            (left,top),
                            (left + width, top),
                            (left + width, top + height),
                            (left , top + height),
                            (left, top)
                        )

                        # Outline object
                        self.draw.line(points, fill='#fa5ea7', width=2)
                    
                        if top <= 30: # add label to bottom of box
                            text_top = top + height + 5
                        else:
                            text_top = top - 20 # add label to top of box
                    
                        # Label object
                        print(f"labeling {label['Name']}")
                        self.draw.text((left, text_top), label['Name'], (250, 94, 167), font=font)

        return self.labels

    def save_scene(self, session_id):
        """Saves annotations and writes results (objects/people in scene) to DDB
        """

        print("Saving annotated picture")
        self.annotated_image.save(f'./annotated.jpg')
        self.upload_annotated_image_to_s3('annotated.jpg')

        table = dynamodb_resource.Table('TwitchRobot_Scenes')
        payload = {
            'session_id': session_id,
            'recognized_people': self.recognized_people,
            'labels': self.labels
        }
        
        try:
            response = table.put_item(
                Item=payload
            )
            return "Scene saved"
        except ClientError as e:
            if e.response['Error']['Code'] == 'EntityAlreadyExists':
                error = "Entry already exists"
                print(error)
            else:
                error = e
                print(f"Unexpected error: {error}")
            
            return error
    
    