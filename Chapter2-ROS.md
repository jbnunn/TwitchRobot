# [Twitch Robot](./README.md) > Installing the Robotic Operating System (ROS)

This is the longest portion of the entire install, and may take 2 hours to complete, due to building ROS from source.

## Steps

### SSH into your Raspberry Pi and install dependencies

```
ssh pi@192.168.X.Y
```

```
sudo sh -c 'echo "deb  http://packages.ros.org/ros/ubuntu  $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
pip install rospkg
```

### Initialize Rosdep and Update

```
sudo rosdep init
rosdep update
```

### Setup your ROS Workspace

```
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
```

### Install the ROS Desktokp 

This will include GUI tools like `rqt` and `rviz`:

```
rosinstall_generator desktop --rosdistro melodic --deps --wet-only --tar > melodic-desktop-wet.rosinstall 
```

For Raspberry Pi 4,

```
wstool init -j8 src melodic-desktop-wet.rosinstall
```

For older Raspberry Pis:

```
wstool init -j2 src melodic-desktop-wet.rosinstall
```

### Add the Open Asset Import Library 

[Open Asset Import Library](http://www.assimp.org/) (short name: Assimp) is a portable Open Source library to import various well-known 3D model formats in a uniform manner. The most recent version also knows how to export 3d files and is therefore suitable as a general-purpose 3D model converter.

```
mkdir -p ~/ros_catkin_ws/external_src 
cd ~/ros_catkin_ws/external_src
wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
unzip assimp-3.1.1_no_test_models.zip
cd assimp-3.1.1
cmake .
make
sudo make install
```

### Install OGRE for rviz

```
sudo apt-get install -y libogre-1.9-dev
```

### Install the rest of the dependencies

```
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
```

### Build the Catkin packages

**Warning: This takes about 2 hours on a Raspberry Pi 4.**

Note: I installed with the `-j2` switch. You can try `-j8` or `-j4` on a Raspberry Pi 4 and potentially save a lot of time.

```
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j2
```

### Init some default shell scripts

```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/TwitchRobot/create_ws/devel/setup.bash" >> ~/.bashrc
```

### Setup an alias to Python 3

```
echo "alias python=python3" >> ~/.bashrc
echo "alias pip=pip3" >> ~/.bashrc
```

### Finally, source the changes

```
source ~/.bashrc
```

Now, test the installation by entering the `roscore` command. You should see:

```
roscore

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
```

### On to the next step...

If you see the output above, you're ready to install the Robot Code in [Chapter 3](./Chapter3-RobotCode.md).
