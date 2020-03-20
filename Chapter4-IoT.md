# [Twitch Robot](./README.md) > Creating an IoT Thing and Certificates

This should take less than 30 minutes. 

## Steps

### Create Thing and Certificates

You'll need to create certificates in order to securely communicate with the robot over MQTT. Visit the [AWS IoT console](https://us-east-1.console.aws.amazon.com/iot/home?region=us-east-1). If this is your first time visiting the AWS IoT console, you may need to click the Get started button to get to the main dashboard.

When the IoT console loads, look to the left navigation panel, click "Onboard", then click the Get started button to start the connection wizard.

1. Click **Get started** to move past the overview screen

2. Choose "Linux/OSX" as your platform and "Python" as the AWS IoT Device SDK, then click **Next step**.

3. In the "Register a thing" step, give your "thing" (a robot, in this case) a name, "TwitchRobot", then click **Next step**.

4. Click the "Download connection kit for Linux/OSX" button. This downloads a `connect_device_package.zip` file containing the certificate and private key you'll need a bit later. When the file has downloaded, click **Next step**.

5. You do not need to follow the steps on the "Configure and test your device" section. Click **Done**, and then **Done** again to take you to your "Things" hub.

### Attach a new policy to your certificate

1. Select your "TwitchRobot" from the [Thing hub](https://us-east-1.console.aws.amazon.com/iot/home?region=us-east-1#/thinghub)

2. Choose "Security" from the left navigation menu.

3. Click the box containing your certificate's identification. This box will contain a string of characters (e.g., "2d9c56...").

4. Certificate details will load. Click on the "Actions" dropdown in the upper right, then "Attach Policy."

5. Check the box for "TwitchRobot-Policy", then click the **Attach** button.

### Update the policy

1. Click "Policies" on the left navigation menu, then select "TwitchRobot-Policy" to load the policy.

2. Click "Edit policy document" and replace the existing policy with the one below:

```
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
```

3. Click "Save as new version" to save the policy. 

**IMPORTANT: THE POLICY ABOVE SHOULD BE USED FOR TESTING. IN PRODUCTION ENVIRONMENTS, LIMIT THE RESOURCE ACCESS TO SPECIFIC RESOURCE(S), RATHER THAN ALL RESOURCES ("*").**

### Upload certificates to Raspberry Pi

1. The certificates you downloaded earlier in this step are located in the `connect_device_package.zip` file. Unzip this file. You should have TwitchRobot.cert.pem, TwitchRobot.private.key, and TwitchRobot.public.key. (Ignore the start.sh file this extracted).

2. From a terminal prompt on your computer, copy these files to your Raspberry Pi. 

```
cd ~/Downloads/connect_device_package # Or similar location
scp TwitchRobot.cert.pem pi@192.168.X.Y:/home/pi/TwitchRobot/certs
scp TwitchRobot.private.key pi@192.168.X.Y:/home/pi/TwitchRobot/certs
```

Your certs directory should now have three files:

```
pi@raspberrypi:~/TwitchRobot/certs $ ls
AmazonRootCA1.crt  TwitchRobot.cert.pem  TwitchRobot.private.key
```

### Install and configure the AWS CLI tool

On your Raspberry Pi, install the AWS CLI tools:

```
pip install awscli --upgrade --user
echo "PATH=/home/pi/.local/bin:$PATH" >> ~/.bashrc
source ~/.bashrc
```

Now, configure the AWS CLI tool. You'll need to create an [IAM user on AWS] (https://docs.aws.amazon.com/IAM/latest/UserGuide/id_users_create.html) that has access to call the AWS IoT service. Use the `AWS Access Key ID` and `AWS Secret Access Key` generated to configure the CLI tool.

```
aws configure

    AWS Access Key ID [None]: AKIA****************
    AWS Secret Access Key [None]: Ga45*********************
    Default region name [None]: us-east-1
    Default output format [None]: json
```

### Setup OS Environment variable

You'll need an AWS IOT endpoint with which to communicted to/from your robot. From the command line, issue the following command:

```
aws iot describe-endpoint --endpoint-type iot:Data-ATS
```

You should see a result similar to:

    {
        "endpointAddress": "abcdefghijk123-ats.iot.us-east-1.amazonaws.com"
    }

Copy the `endpointAddress` value (you'll also need this in step 5).

Add this value to your `.bashrc` file (**REPLACE THE ENDPOINT WITH YOUR OWN**)

```
echo "export AWS_IOT_ENDPOINT=abcdefghijk123-ats.iot.us-east-1.amazonaws.com" >> ~/.bashrc
source ~/.bashrc
```

### Install the official [AWS IoT Device SDK](https://github.com/aws/aws-iot-device-sdk-python) for Python 

Install the AWS IoT Device SDK for Python to easily configure parameters for communication between your robot and the cloud.

```
pip install AWSIoTPythonSDK
```

### On to the next step...

When you've finished these steps, [you're ready to configure your AWS Lambda function and ASK Skill in Chapter 5](./Chapter5-Lambda-ASK.md).
