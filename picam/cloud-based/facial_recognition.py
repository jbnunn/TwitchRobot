'''
This class handles the image/vision functions required by the robot. It can

* Send images to AWS Rekognition for 
    * object detection
    * face detection (https://docs.aws.amazon.com/rekognition/latest/dg/API_DetectFaces.html) and recognition
'''

import boto3
import sys
import io

from PIL import Image, ImageDraw, ImageFont

# S3 Parameters
REGION = 'us-east-1'
BUCKET = 'jeffnunn-public'

# Rekognition Parameters
MAX_LABELS = 10
MIN_CONFIDENCE = 80
COLLECTION_ID = 'family_and_friends'
DDB_TABLE = 'rekognition_family_and_friends'

rekognition = boto3.client('rekognition', region_name='us-east-1')
dynamodb = boto3.client('dynamodb', region_name='us-east-1')

font = ImageFont.truetype('./arial.ttf', 40)

class Vision():

    def __init__(self, image):
        self.bucket = BUCKET
        self.image = image
        self.annotated_image = None
        self.orginal = image
        self.faces = []
        self.labels = []
        self.found_faces = []
   
    '''
    Upload an image to S3. Set the bucket in `# S3 Parameters` above

    '''
    def upload_to_s3(self):
        boto3.client('s3').upload_file(self.image, self.bucket, self.image)

    '''
    Use AWS Rekognition to detect faces,

        https://boto3.amazonaws.com/v1/documentation/api/latest/reference/services/rekognition.html#Rekognition.Client.detect_faces

    then annotate them with bounding boxes and labels.
    '''
    def detect_faces(self):
        image = Image.open(self.image)
        stream = io.BytesIO()
        image.save(stream,format="JPEG")
        image_binary = stream.getvalue()

        response = rekognition.detect_faces(
            Image={'Bytes':image_binary}                                        
        )
            
        all_faces=response['FaceDetails']
            
        # Initialize list object
        boxes = []
        recognized_faces = {}
        unknown_faces = []

        # Get image dimensions
        image_width = image.size[0]
        image_height = image.size[1]
        
        # Crop face from image
        for face in all_faces:
            box = face['BoundingBox']
            x1 = int(box['Left'] * image_width) * 0.9
            y1 = int(box['Top'] * image_height) * 0.9
            x2 = int(box['Left'] * image_width + box['Width'] * image_width) * 1.10
            y2 = int(box['Top'] * image_height + box['Height']  * image_height) * 1.10
            image_crop = image.crop((x1,y1,x2,y2))
            
            stream = io.BytesIO()
            image_crop.save(stream,format="JPEG")
            image_crop_binary = stream.getvalue()

            # Submit individually cropped image to Amazon Rekognition
            response = rekognition.search_faces_by_image(
                CollectionId=COLLECTION_ID,
                Image={'Bytes':image_crop_binary}                                       
            )
            
            if len(response['FaceMatches']) > 0:
                for match in response['FaceMatches']:
                        
                    face = dynamodb.get_item(
                        TableName=DDB_TABLE,               
                        Key={'RekognitionId': {'S': match['Face']['FaceId']}}
                    )
            
                    if 'Item' in face:
                        recognized_faces[face['Item']['FullName']['S']] = box
                        break
            
            else:
                unknown_faces.append(box)

        annotated_image = ImageDraw.Draw(image)

        for name in recognized_faces:
            left = recognized_faces[name]['Left']
            top = recognized_faces[name]['Top']
            width = recognized_faces[name]['Width']
            height = recognized_faces[name]['Height']

            x0 = int(image.width * left)
            y0 = int(image.height * top)
            x1 = int(x0 + image.width * width)
            y1 = int(y0 + image.height * height)

            annotated_image.rectangle([(x0, y0), (x1, y1)], outline="green", width=10)

            self.add_label(annotated_image, x0, y1, name)

        for u in unknown_faces:
            left = u['Left']
            top = u['Top']
            width = u['Width']
            height = u['Height']

            x0 = int(image.width * left)
            y0 = int(image.height * top)
            x1 = int(x0 + image.width * width)
            y1 = int(y0 + image.height * height)

            annotated_image.rectangle([(x0, y0), (x1, y1)], outline="white", width=10)
            self.add_label(annotated_image, x0, y1, "Person")

        annotated_filename = 'annotated_' + self.image
        image.save(annotated_filename)
        print(f"Saved {annotated_filename}")
        self.image = image

    '''
    Use AWS Rekognition to detect labels in the image
        (Incomplete)
        https://boto3.amazonaws.com/v1/documentation/api/latest/reference/services/rekognition.html#Rekognition.Client.detect_labels
    '''
    def detect_labels(self):
        resp = rekognition.detect_labels(
            Image = {
                'S3Object': {
                    'Bucket': self.bucket,
                    'Name': self.image
                }
            }, 
            MaxLabels = MAX_LABELS,
            MinConfidence = MIN_CONFIDENCE
        )

        image = Image.open(self.image)
        annotated_image = ImageDraw.Draw(image)
        
        for label in resp['Labels']:
            # self.add_label(annotated_image, )
            print(label)

        
        return resp     

    '''
    Adds an ROI label to the image
    '''
    def add_label(self, annotated_image, x, y, label):
        write_x = x
        write_y = y + 10
        
        annotated_image.text((write_x, write_y), label, font=font, fill=(255,255,255,255))

        self.annotated_image = annotated_image

if __name__ == "__main__":
    try:
        filename = sys.argv[1]
    except:
        sys.exit("Please specify a filename")

    img = Vision(filename)
    img.upload_to_s3()
    img.detect_faces()

