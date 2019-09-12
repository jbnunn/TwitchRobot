# Using PiCam for Twitch Bot

We'll tackle two approaches -- onboard processing and processing in the cloud.

# Face/Object Detection via Onboard Processing 

## Prerequisites:

1. Install OpenCV, 

        sudo apt install python3-opencv python3-pyqt5 

2. Install MXNet

        cd ~/TwitchRobot/picam/onboard/
        wget https://mxnet-public.s3.amazonaws.com/install/raspbian/mxnet-1.5.0-py2.py3-none-any.whl


# Face/Object Detection via the Cloud

We can allow our robot to identify faces it detects by building a simple facial recognition system. In this example, my robot will
recognize faces of family and friends I add to a collection I curate. 

## Create a Collection and Setup Permissions

1. Create a collection to hold your faces:

        aws rekognition create-collection --collection-id family_and_friends --region us-east-1

2. Create an Amazon DynamoDB table to hold your __faceprints__. Rekognition does not store face images, but feature representations represented by mathematical vectors.

        aws dynamodb create-table --table-name rekognition_family_and_friends \
            --attribute-definitions AttributeName=RekognitionId,AttributeType=S \
            --key-schema AttributeName=RekognitionId,KeyType=HASH \
            --provisioned-throughput ReadCapacityUnits=1,WriteCapacityUnits=1 \
            --region us-east-1

3. Create an S3 bucket to hold the image you want Rekognition to index, e.g.,

        aws s3 mb s3://jeffnunn-rekognition/

4. Create a role that allows access to the S3 bucket and DynamoDB table

        aws iam create-role --role-name LambdaRekognitionRole --assume-role-policy-document file://policies/trust_policy.json

5. Attach a trust policy for Lambda to do the indexing to the role

        aws iam put-role-policy --role-name LambdaRekognitionRole --policy-name LambdaPermissions --policy-document file://policies/access_policy.json

## Create an Indexer with AWS Lambda

Visit [the Lambda console](https://console.aws.amazon.com/lambda/home?region=us-east-1#/create) to create a function that gets triggerd every time we add a new picture to our S3 bucket. 

1. Choose "Author from Scratch" and choose the Python 3.6 runtime.
2. For **Permissions**, choose the "Existing Role" option and specify the `LambdaRekognitionRole` you just created.
3. Click "Create Function", then click the "+ Add Trigger" button. Your trigger will be S3 and the name of your bucket.
4. For the event type, choose "All object create events," and give a prefix for your specific collection, e.g., `family-and-friends/`
5. Click "Add" to add the trigger.
6. Now add the following code (note the table_name variable and update accordingly)

        import boto3
        from decimal import Decimal
        import json
        import urllib

        print('Loading function')

        dynamodb = boto3.client('dynamodb')
        s3 = boto3.client('s3')
        rekognition = boto3.client('rekognition')
        table_name = 'rekognition_family_and_friends'

        # --------------- Helper Functions ------------------

        def index_faces(bucket, key):

            response = rekognition.index_faces(
                Image={"S3Object":
                    {"Bucket": bucket,
                    "Name": key}},
                    CollectionId="family_and_friends")
            return response
            
        def update_index(tableName,faceId, fullName):
            response = dynamodb.put_item(
                TableName=tableName,
                Item={
                    'RekognitionId': {'S': faceId},
                    'FullName': {'S': fullName}
                    }
                ) 
            
        # --------------- Main handler ------------------

        def lambda_handler(event, context):

            # Get the object from the event
            bucket = event['Records'][0]['s3']['bucket']['name']
            key = urllib.parse.unquote_plus(
                event['Records'][0]['s3']['object']['key']
            )

            try:

                # Calls Amazon Rekognition IndexFaces API to detect faces in S3 object 
                # to index faces into specified collection
                
                response = index_faces(bucket, key)
                
                # Commit faceId and full name object metadata to DynamoDB
                
                if response['ResponseMetadata']['HTTPStatusCode'] == 200:
                    faceId = response['FaceRecords'][0]['Face']['FaceId']

                    ret = s3.head_object(Bucket=bucket,Key=key)
                    personFullName = ret['Metadata']['fullname']

                    update_index(table_name,faceId,personFullName)

                # Print response to console
                print(response)
                return response
                
            except Exception as e:
                print(e)
                print("Error processing object {} from bucket {}. ".format(key, bucket))
                raise e

## Another Option to Explore: RTSP

We can use the Real Time Streaming Protocol (RTSP) to stream the video from the Pi in real time.

## Prerequisites

Add VLC,

    sudo apt-get install -y vlc

## Launch the camera

Launch the camera:

    raspivid -o - -t 0 -rot 180 -w 600 -h 400 -fps 30 -n | cvlc -vvv stream:///dev/stdin --sout '#rtp{sdp=rtsp://:8554/}' :demux=h264

The flags above are:

    -o          Output written to stdout
    -t 0        Disables the camera timeout
    -rot 180    Rotates the image 180 degrees
    -w          Width of the video output
    -h          Height of the video output
    -fps 30     Frames per second
    -n          Stops the preview of the video from going to HDMI output
    
The feed is then piped to `clvc`, the console VLC player, `-vvv` is an argument of where VLC can get the stream (stdin), and `sout` specifices the RTP port

## View the feed

Using VLC or similar, open a network stream to your Pi's addres and port 8554, e.g.,

    rtsp://192.168.86.25:8554/

## Resources

* [https://raspberry-projects.com/pi/pi-hardware/raspberry-pi-camera/streaming-video-using-vlc-player](https://raspberry-projects.com/pi/pi-hardware/raspberry-pi-camera/streaming-video-using-vlc-player)

