'''
Add images to faces/<person>/ and update the images list below. Run with

    python indexer.py

'''
import boto3

s3 = boto3.resource('s3')

# Get list of objects for indexing
images=[
            ('faces/hank/1.jpg','Hank Nunn'),
            ('faces/hank/2.jpg','Hank Nunn'),
            ('faces/hank/3.jpg','Hank Nunn'),
            ('faces/hank/4.jpg','Hank Nunn'),
            ('faces/hank/5.jpg','Hank Nunn'),
            ('faces/hank/6.jpg','Hank Nunn'),
            ('faces/hank/7.jpg','Hank Nunn'),
            ('faces/hank/8.jpg','Hank Nunn'),
            ('faces/hank/9.jpg','Hank Nunn'),
            ('faces/hank/10.jpg','Hank Nunn'),
            ('faces/hank/11.jpg','Hank Nunn'),
            ('faces/hank/12.jpg','Hank Nunn'),
       ]

# Iterate through list to upload objects to S3   
for image in images:
    file = open(image[0],'rb')
    object = s3.Object('jeffnunn-rekognition','family-and-friends/'+ image[0])
    ret = object.put(Body=file,Metadata={'FullName':image[1]})