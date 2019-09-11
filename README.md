# Twitch Robot

This is the robot code, Alexa Skills Kit code, and AWS Lambda code you'll need to control an [iRobot Create 2](https://www.irobot.com/about-irobot/stem/create-2) via voice. 

This code follows the live-coding streams found at [https://twitch.tv/jbnunn](https://twitch.tv/jbnunn), [https://twitch.tv/amazonalexa](https://twitch.tv/amazonalexa), and the [Amazon Alexa Voice + Robotics Youtube Channel](https://www.youtube.com/playlist?list=PL2KJmkHeYQTNKbeNmYxs-CY3AhPJcl61U).

## Requirements

* This assumes you have Raspberry Pi 4 and a 16GB mini-SD card. It is possible to get this working on a Raspberry Pi >= 2B, but it will require more time to build. 
* Min 8GB mini-SD card

Follow the isntructions below to install the OS, the Robotic Operating System (ROS), and configure your Alexa Skills Kit skill and Lambda function.

## Install Raspbian Buster

* [Raspbian Buster](https://www.raspberrypi.org/downloads/raspbian/). Desktop is fine if you have a >8 GB card.
* ROS Melodic

## Install ROS Melodic

### Install Dependencies

    $ sudo sh -c 'echo "deb  http://packages.ros.org/ros/ubuntu  $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    $ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    $ sudo apt-get update
    $ sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential  cmake

### Initialize Rosdep and Update

    $ sudo rosdep init
    $ rosdep update

### Setup your ROS Workspace

    $ mkdir ~/ros_catkin_ws
    $ cd ~/ros_catkin_ws

### Install the ROS Desktokp 

This will include GUI tools like `rqt` and `rviz`:

    $ rosinstall_generator desktop --rosdistro melodic --deps --wet-only --tar > melodic-desktop-wet.rosinstall 

For Raspberry Pi 4,

    $ wstool init -j8 src melodic-desktop-wet.rosinstall

For older Raspberry Pis:

    $ wstool init -j2 src melodic-desktop-wet.rosinstall

### Add the Open Asset Import Library 

[Open Asset Import Library](http://www.assimp.org/) (short name: Assimp) is a portable Open Source library to import various well-known 3D model formats in a uniform manner. The most recent version also knows how to export 3d files and is therefore suitable as a general-purpose 3D model converter.

    $ mkdir -p ~/ros_catkin_ws/external_src 
    $ cd ~/ros_catkin_ws/external_src
    $ wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
    $ unzip assimp-3.1.1_no_test_models.zip
    $ cd assimp-3.1.1
    $ cmake .
    $ make
    $ sudo make install

### Install OGRE for rviz

    $ sudo apt-get install -y libogre-1.9-dev

### Install the rest of the dependencies

    $ rosdep install --from-paths src --ignore-src --rosdistro melodic -y

### Build the Catkin packages

**Warning: This takes about 2 hours on a Raspberry Pi 4.**

    $ sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j2

### Finally, source the installation and test the install

    $ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    $ echo "source ~/TwitchRobot/create_ws/devel/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc

Now, test the installation by entering the `roscore` command. You should see:

    $ roscore

    ... logging to /home/pi/.ros/log/a94ec556-11e9-d40e-a4bc-dca63206194e/roslaunch-raspberrypi-12692.log
    Checking log directory for disk usage. This may take awhile.
    Press Ctrl-C to interrupt
    Done checking log file disk usage. Usage is <1GB.

    started roslaunch server http://raspberrypi:35885/
    ros_comm version 1.14.3


    SUMMARY
    ========

    PARAMETERS
    * /rosdistro: melodic
    * /rosversion: 1.14.3

    NODES

    auto-starting new master
    process[master]: started with pid [12702]
    ROS_MASTER_URI=http://raspberrypi:11311/

    ...

## Install the Twitch Robot code

Now that ROS is setup on your Raspberry Pi, you'll want to clone this repo into ~/TwitchRobot/.

    $ git clone https://github.com/jbnunn/TwitchRobot.git ~/

### Submodules

This project uses Git submodules as references to other repos. After cloning, make sure you've got the latest the submodules:

    $ git pull --recurse-submodules
    
### Configure AWS IoT Certificates

You'll need to create certificates in order to securely communicate with the robot over MQTT. Follow [Create and Register an AWS IoT Device Certificate](https://docs.aws.amazon.com/iot/latest/developerguide/device-certs-create.html) and save the certificates you generate to the `certs/` and `lambda/certs/` folder.

### Install the official [AWS IoT Device SDK](https://github.com/aws/aws-iot-device-sdk-python) for Python 

Install the AWS IoT Device SDK for Python to easily configure parameters for communication between your robot and the cloud.

    $ pip3 install AWSIoTPythonSDK
    
### Establish Serial Communciations

    $ sudo usermod -a -G dialout $USER

### Setup dependencies

    $ cd ~/TwitchRobot/create_ws
    $ sudo apt-get update && sudo apt-get install --only-upgrade python-catkin-pkg && sudo apt-get install -y python-catkin-tools
    $ rosdep update
    $ rosdep install --from-paths src --ignore-src -r -y

### Build

    $ ~/TwitchRobot/create_ws
    $ catkin build -j4

## Lambda

TBD

## Alexa SKills Kit

TBD

## Connect to and Drive the Robot

TBD

## Sources

* https://www.instructables.com/id/ROS-Melodic-on-Raspberry-Pi-4-RPLIDAR/
