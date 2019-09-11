# [Twitch Robot](./README.md) > Installing the Robot Code

You'll be cloning a repo and updateing a few dependencies. This might take 20-30 minutes. 

## Steps

### SSH into your Raspberry Pi

    ssh pi@192.168.X.Y

### Clone this repo

Clone this repo into `~/TwitchRobot/`:

    $ git clone https://github.com/jbnunn/TwitchRobot.git ~/

### Get the Submodules

This project uses Git submodules as references to other repos. After cloning, make sure you've got the latest the submodules:

    $ git pull --recurse-submodules

### Setup dependencies

    $ cd ~/TwitchRobot/create_ws
    $ sudo apt-get update && sudo apt-get install --only-upgrade python-catkin-pkg && sudo apt-get install -y python-catkin-tools
    $ rosdep update
    $ rosdep install --from-paths src --ignore-src -r -y

### Build

    $ ~/TwitchRobot/create_ws
    $ catkin build -j4

### Establish Serial Communciations with the Create 2

    $ sudo usermod -a -G dialout $USER

### Configure AWS IoT Certificates

You'll need to create certificates in order to securely communicate with the robot over MQTT. Follow [Create and Register an AWS IoT Device Certificate](https://docs.aws.amazon.com/iot/latest/developerguide/device-certs-create.html) and save the certificates you generate to the `certs/` and `lambda/certs/` folder.

### Install the official [AWS IoT Device SDK](https://github.com/aws/aws-iot-device-sdk-python) for Python 

Install the AWS IoT Device SDK for Python to easily configure parameters for communication between your robot and the cloud.

    $ pip3 install AWSIoTPythonSDK

### On to the next step...

When you've finished these steps, [you're ready to configure your AWS Lambda function](./Part4-Lambda.md).