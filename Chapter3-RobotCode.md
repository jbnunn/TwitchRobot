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

### On to the next step...

If you see the output above, you're ready to setup AWS IoT in [Chapter 4](./Chapter4-IoT.md).