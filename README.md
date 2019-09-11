# Twitch Robot

This is the robot code, Alexa Skills Kit code, and AWS Lambda code you'll need to control an [iRobot Create 2](https://www.irobot.com/about-irobot/stem/create-2) via voice. 

This code follows the live-coding streams found at [https://twitch.tv/jbnunn](https://twitch.tv/jbnunn), [https://twitch.tv/amazonalexa](https://twitch.tv/amazonalexa), and the [Amazon Alexa Voice + Robotics Youtube Channel](https://www.youtube.com/playlist?list=PL2KJmkHeYQTNKbeNmYxs-CY3AhPJcl61U).

## Requirements

* This assumes you have Raspberry Pi 4. It is possible to get this working on a Raspberry Pi >= 2B, but it will require more time to build. 
* Use a 16GB or 32GB card if possible, but an 8GB mini-SD card will work as well.

Follow the instructions below to install the OS, the Robotic Operating System (ROS), AWS Lambda code, creation of an Alexa Skills Kit skill, and finally the code which will run on your robot to control it.

## Install Raspian Buster

**Estimated Time: 15-25 minutes**

Follow [the instructions](./Part1-Raspbian.md) to install Raspian Buster and update the core files. 

## Install ROS Melodic

**Estimated Time: 90-120+ minutes**

You're now ready to [install the Robotic Operating System](./Part2-ROS.md). 

## Install the Twitch Robot code

**Estimated Time: 20-30 minutes**

Now that ROS is setup on your Raspberry Pi, [follow the instructions to install the code to control the robot](./Part3-RobotCode.md). 

## Setup AWS IoT Service

**Estimated Time: 5-10 minutes**

[In this section](./Part4-IoT), you'll need to register an IoT "Thing" and create certificates that will allow secure communication.

## Create AWS Lambda function code and ASK function

**Estimated Time: < 30 minutes**

Follow [the instructions to create an AWS Lambda function and ASK Skill](./Part5-Lambda-ASK.md). The Lambda function will receive voice commands from your Echo device (via an Alexa Skills Kit skill), and pass those messages on to the Create 2 robot. 

## Connect to and drive the Create 2 robot

You're now ready to drive the robot! Let's SSH in first to make sure it responds to commands directly on the robot:

TBD

## Sources

Installing ROS is rarely a cakewalk, but the installation of Raspbian and ROS Melodic was made easier by the blog post at [https://www.instructables.com/id/ROS-Melodic-on-Raspberry-Pi-4-RPLIDAR/](https://www.instructables.com/id/ROS-Melodic-on-Raspberry-Pi-4-RPLIDAR/).

