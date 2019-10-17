# Twitch Robot

This is the robot code, Alexa Skills Kit code, and AWS Lambda code you'll need to control an [iRobot Create 2](https://www.irobot.com/about-irobot/stem/create-2) via voice. 

This code follows the live-coding streams found at [https://twitch.tv/jbnunn](https://twitch.tv/jbnunn), [https://twitch.tv/amazonalexa](https://twitch.tv/amazonalexa), and the [Amazon Alexa Voice + Robotics Youtube Channel](https://www.youtube.com/playlist?list=PL2KJmkHeYQTNKbeNmYxs-CY3AhPJcl61U).

## Requirements

* This assumes you have Raspberry Pi 4. It is possible to get this working on a Raspberry Pi >= 2B, but it will require more time to build. 
* Use a 16GB or 32GB card if possible, but an 8GB mini-SD card will work as well.

Follow the instructions below to install the OS, the Robotic Operating System (ROS), AWS Lambda code, creation of an Alexa Skills Kit skill, and finally the code which will run on your robot to operate it and have it recognize people and objects.

## Chapter 1: Install Raspian Buster

**Estimated Time: 15-25 minutes**

Follow [the instructions](./Chapter1-Raspbian.md) to install Raspian Buster and update the core files. 

## Chapter 2: Install ROS Melodic

**Estimated Time: 90-120+ minutes**

You're now ready to [install the Robotic Operating System](./Chapter2-ROS.md). 

## Chapter 3: Install the Twitch Robot code

**Estimated Time: 20-30 minutes**

Now that ROS is setup on your Raspberry Pi, [follow the instructions to install the code to control the robot](./Chapter3-RobotCode.md). 

## Chapter 4: Setup AWS IoT Service

**Estimated Time: 5-10 minutes**

[In this section](./Chapter4-IoT), you'll need to register an IoT "Thing" and create certificates that will allow secure communication.

## Chapter 5: Create AWS Lambda function code and ASK function

**Estimated Time: < 30 minutes**

Follow [the instructions to create an AWS Lambda function and ASK Skill](./Chapter5-Lambda-ASK.md). The Lambda function will receive voice commands from your Echo device (via an Alexa Skills Kit skill), and pass those messages on to the Create 2 robot. 

## Chapter 6: Connect to and drive the Create 2 robot

**Estimated Time: 5-10 minutes**

You're now ready to [control the Robot](./Chapter6-Control.md). 

## Chapter 7: Person and Object Detection

**Estimated Time: 5-10 minutes**

You have a robot you can control with voice, now we'll add extra functionality to it to make it more useful. [Follow the instructions in Chapter 7](./Chater7-Person-Object-Detection.md) to give your robot the ability to detect and people and objects and do facial recognition.

## Sources

Installing ROS is rarely a cakewalk, but the installation of Raspbian and ROS Melodic was made easier by the blog post at [https://www.instructables.com/id/ROS-Melodic-on-Raspberry-Pi-4-RPLIDAR/](https://www.instructables.com/id/ROS-Melodic-on-Raspberry-Pi-4-RPLIDAR/).

