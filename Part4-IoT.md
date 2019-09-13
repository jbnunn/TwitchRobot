# [Twitch Robot](./README.md) > Creating an IoT Thing and Certificates

This should take 5-10 minutes. 

## Steps

### Create Thing and Certificates

1. You'll need to create certificates in order to securely communicate with the robot over MQTT. Follow [Create and Register an AWS IoT Device Certificate](https://docs.aws.amazon.com/iot/latest/developerguide/device-certs-create.html) and save the certificates you generate to the `certs/` and `lambda/certs/` folder.

2. Activate the Thing, then attach the following policy to allow communications between the IoT service and your device.

    {
        "Version": "2012-10-17",
        "Statement": [{
                "Effect": "Allow",
                "Action": [
                    "iot:Publish",
                    "iot:Subscribe",
                    "iot:Connect",
                    "iot:Receive"
                ],
                "Resource": [
                    "*"
                ]
            },
            {
                "Effect": "Allow",
                "Action": [
                    "iot:GetThingShadow",
                    "iot:UpdateThingShadow",
                    "iot:DeleteThingShadow"
                ],
                "Resource": [
                    "*"
                ]
            },
            {
                "Effect": "Allow",
                "Action": [
                    "greengrass:*"
                ],
                "Resource": [
                    "*"
                ]
            }
        ]
    }

### Setup OS Environment variable

You'll need an AWS IOT endpoint with which to communicted to/from your robot. From the command line, issue the following command:

    $ aws iot describe-endpoint --endpoint-type iot:Data-ATS

You should see a result similar to:

    {
        "endpointAddress": "abcdefghijk123-ats.iot.us-east-1.amazonaws.com"
    }

Copy the `endpointAddress` value (you'll also need this in step 5).

Open your `.bashrc` file and add this environment variable:

    nano ~/.bashrc

At the end of the file, add the following (be sure to replace the sample endpoint with your real endpoint):

    # ENV Variable for Twitch Robot and AWS IoT
    export AWS_IOT_ENDPOINT=abcdefghijk123-ats.iot.us-east-1.amazonaws.com

Save the file and exit. Source your bash file,

    source ~/.bashrc

### Install the official [AWS IoT Device SDK](https://github.com/aws/aws-iot-device-sdk-python) for Python 

Install the AWS IoT Device SDK for Python to easily configure parameters for communication between your robot and the cloud.

    $ pip install AWSIoTPythonSDK

### Update your certificate names in `/create_ws/src/alexa/src/listener.py`.

You'll need to change the names of the certificates in the `listener.py` code. Open that file, and search for the line

    # Change these certificate names

Modify the certificate names to be the ones you downloaded.


### On to the next step...

When you've finished these steps, [you're ready to configure your AWS Lambda function and ASK Skill](./Part5-Lambda-ASK.md).
