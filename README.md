# Twitch Robot - Robot Code

This is the ROS-based code that receives messages from ASK/Lambda and controls the robot from your Raspberry Pi

## Step 1: Setup the Raspberry Pi

You'll need to configure your Pi's SD card with [Ubuntu Mate](https://ubuntu-mate.org/raspberry-pi/) first, then install [ROS Melodic](http://wiki.ros.org/melodic/). The blog post at [https://www.intorobotics.com/installing-ros-melodic-on-raspberry-pi-3b-running-ubuntu-mate-18-04-2-bionic/](https://www.intorobotics.com/installing-ros-melodic-on-raspberry-pi-3b-running-ubuntu-mate-18-04-2-bionic/) is helpful.

After you've successfully installed ROS Melodic on your Raspberry Pi, you're ready for the next step.

## Step 2: Install the robot code

Clone the `robot` branch of repo into your root folder on the Raspberry Pi. This contains the `create_ws` folder which contains the code needed to control your robot.


    git clone -b robot https://github.com/jbnunn/TwitchRobot.git

## Step 3: Control the robot

Full details are at [https://github.com/jbnunn/create_autonomy](https://github.com/jbnunn/create_autonomy), but the steps below should get you going. 

### Setup

Before connecting the serial USB cable to the Create, add your user to the Pi's `dialout` group to ensure USB communication.

    $ sudo usermod -a -G dialout $USER

Next, logout and login for the permission to take effect. Then:

1. Connect the Raspberry Pi to the Create's 7-pin serial port
2. Launch the driver file

### Launch file

    $ roslaunch ca_driver create_2.launch

Upon launching, you should see the following

    $ [ INFO] ... [CREATE] Ready.

### Control the robot

You can move the robot around by sending [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) messages to the topic cmd_vel:

    $ rostopic pub /cmd_vel geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

This should make the robot jump forward.



    
    


