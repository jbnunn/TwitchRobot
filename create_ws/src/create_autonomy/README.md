# create_autonomy

[ROS](http://ros.org) driver for iRobot [Create 2](http://www.irobot.com/About-iRobot/STEM/Create-2.aspx).
This package is based off of [https://github.com/AutonomyLab/create_autonomy](https://github.com/AutonomyLab/create_autonomy), but specifically for ROS Melodic and the Create 2. It wraps the C++ library [libcreate][libcreate], which uses iRobot's [Open Interface Specification][oi_spec]. 

<!--[](* Documentation: TODO)-->
* ROS wiki page: http://wiki.ros.org/create_autonomy
* Support: [ROS Answers (tag: create_autonomy)](http://answers.ros.org/questions/scope:all/sort:activity-desc/tags:create_autonomy/page:1/)
* Original Author: [Jacob Perron](http://jacobperron.ca) ([Autonomy Lab](http://autonomylab.org), [Simon Fraser University](http://www.sfu.ca))
* Melodic/Create 2 Maintainer: [Jeff Nunn](https://twitter.com/jbnunn)

## Raspberry Pi

This is tested and working on a Raspberry Pi 2 Model B v 1.1, and will likely work on models newer than that. Build switches have been applied to facilitate installation on a Raspberry Pi.

## Supported Robots

| Model     | Support    |
|-----------|------------|
| Create 2  _(firmware >= 3.2.6)_ |  Yes       |

## Features

|  Feature          |  Status       |
|-------------------|---------------|
|  Odometry         | Available     |
|  Safe mode        | Planned [#13](https://github.com/AutonomyLab/create_autonomy/issues/13) |
|  Clean demo       | Planned [#14](https://github.com/AutonomyLab/create_autonomy/issues/14) |
|  Dock demo        | Available     |
|  Drive wheels     | N/A           |
|  Drive (v,w)      | Available     |
|  Brush motors     | Planned [#15](https://github.com/AutonomyLab/create_autonomy/issues/15) |
|  LEDs             | Available     |
|  Digit LEDs       | Available     |
|  Sound            | Available     |
|  Wheeldrop        | Available     |
|  Bumpers          | Available     |
|  Cliff sensor     | Planned [#22](https://github.com/AutonomyLab/create_autonomy/issues/22)      |
|  Dirt detect      | N/A           |
|  Omni IR sensor   | Available     |
|  Left IR sensor   | N/A           |
|  Right IR sensor  | N/A           |
|  Battery info     | Available     |
|  Light sensors    | Available     |
| **_Diagnostics_** |               |
|  Corrupt packets  | Planned       |
|  Physical tests   | Planned       |
|  Overcurrent info | Planned       |

## Install

#### Prerequisites

* Internet connection
* [ROS](http://wiki.ros.org/ROS/Installation) _Melodic_
* Ubuntu packages: `python-rosdep`, `python-catkin-tools`

``` bash
$ sudo apt-get install python-rosdep python-catkin-tools
```

#### Compiling

1. Create a catkin workspace  
    ``` bash
    $ cd ~
    $ mkdir -p create_ws/src  
    $ cd create_ws  
    $ catkin init  
    ```

2. Clone this repo  
    ``` bash
    $ cd ~/create_ws/src
    $ git clone https://github.com/jbnunn/create_autonomy.git  
    ```
  
3. Install dependencies  
    ``` bash
    $ sudo apt-get update && sudo apt-get install --only-upgrade python-catkin-pkg
    $ cd ~/create_ws/src
    $ git clone https://github.com/AutonomyLab/libcreate
    $ git clone https://github.com/ros/diagnostics.git
    $ cd ~/create_ws
    $ rosdep update  
    $ rosdep install --from-paths src --ignore-src -r -y
    ```

4. Build  
    ``` bash
    $ cd ~/create_ws
    $ catkin build -j1
    ```

5. Source the workspace
    ``` bash
    $ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    $ echo "source $HOME/create_ws/devel/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc
    ```

#### USB Permissions
6. In order to connect to Create over USB, ensure your user is in the dialout group
    ``` bash
    $ sudo usermod -a -G dialout $USER
    ```

7. Logout and login for permission to take effect

## Running the driver

### Setup

1. Connect computer to Create's 7-pin serial port

2. Launch one of the existing launch files or adapt them to create your own.

### Launch files

``` bash
$ roslaunch ca_driver create_2.launch
```

Upon launching, you should see the following

``` bash
$ [ INFO] ... [CREATE] Ready.
```

#### Launch file arguments

* **config** - Absolute path to a configuration file (YAML). Default: `ca_driver/config/default.yaml`
* **desc** - Enable robot description (URDF/mesh). Default: `true`

For example, if you would like to disable the robot description and provide a custom configuration file:

```bash
$ roslaunch ca_driver create_2.launch config:=/abs/path/to/config.yaml desc:=false
```


### Parameters

 Name         |  Description |  Default
--------------|--------------|----------
`dev`         |  Device path of robot |  `/dev/ttyUSB0`
`base_frame`  |  The robot's base frame ID | `base_footprint`
`odom_frame`  |  The robot's odometry frame ID | `odom`
`latch_cmd_duration` | If this many seconds passes without receiving a velocity command the robot stops | `0.2`
`loop_hz`     |  Frequency of internal update loop |  `10.0`
`publish_tf`  |  Publish the transform from `odom_frame` to `base_frame` | `true`  
`robot_model` |  The type of robot being controlled (supported values: `ROOMBA_400`, `CREATE_1` and `CREATE_2`) | `CREATE_2`
`baud`        |  Serial baud rate | Inferred based on robot model, but is overwritten upon providing a value

### Publishers

 Topic       | Description  | Type
-------------|--------------|------
 `battery/capacity` | The estimated charge capacity of the robot's battery (Ah) | [std_msgs/Float32][float32]
 `battery/charge` | The current charge of the robot's battery (Ah) | [std_msgs/Float32][float32]
 `battery/charge_ratio` | Charge / capacity | [std_msgs/Float32][float32]
 `battery/charging_state` | The chargins state of the battery | [ca_msgs/ChargingState][chargingstate_msg]
 `battery/current` | Current flowing through the robot's battery (A). Positive current implies charging | [std_msgs/Float32][float32]
 `battery/temperature` | The temperature of the robot's battery (degrees Celsius) | [std_msgs/Int16][int16]
 `battery/voltage` | Voltage of the robot's battery (V) | [std_msgs/Float32][float32]
 `bumper` | Bumper state message (including light sensors on bumpers) | [ca_msgs/Bumper][bumper_msg]
 `clean_button` | 'clean' button is pressed | [std_msgs/Empty][empty]
 `day_button` |  'day' button is pressed | [std_msgs/Empty][empty]
 `hour_button` | 'hour' button is pressed | [std_msgs/Empty][empty]
 `minute_button` | 'minute' button is pressed | [std_msgs/Empty][empty]
 `dock_button` | 'dock' button is pressed | [std_msgs/Empty][empty]
 `spot_button` | 'spot' button is pressed | [std_msgs/Empty][empty]
 `ir_omni` | The IR character currently being read by the omnidirectional receiver. Value 0 means no character is being received | [std_msgs/UInt16][uint16]
 `joint_states` | The states (position, velocity) of the drive wheel joints | [sensor_msgs/JointState][jointstate_msg]
 `mode` | The current mode of the robot (See [OI Spec][oi_spec] for details)| [ca_msgs/Mode][mode_msg]
 `odom` |  Robot odometry according to wheel encoders | [nav_msgs/Odometry][odometry]
 `wheeldrop` | At least one of the drive wheels has dropped | [std_msgs/Empty][empty]
 `/tf` | The transform from the `odom` frame to `base_footprint`. Only if the parameter `publish_tf` is `true` | [tf2_msgs/TFMessage](http://docs.ros.org/jade/api/tf2_msgs/html/msg/TFMessage.html)


### Subscribers

Topic       | Description   | Type
------------|---------------|------
`cmd_vel` | Drives the robot's wheels according to a forward and angular velocity | [geometry_msgs/Twist][twist]
`debris_led` | Enable / disable the blue 'debris' LED | [std_msgs/Bool][bool]
`spot_led`   | Enable / disable the 'spot' LED | [std_msgs/Bool][bool]
`dock_led`   | Enable / disable the 'dock' LED | [std_msgs/Bool][bool]
`check_led`  | Enable / disable the 'check robot` LED | [std_msgs/Bool][bool]
`power_led`  | Set the 'power' LED color and intensity. Accepts 1 or 2 bytes, the first represents the color between green (0) and red (255) and the second (optional) represents the intensity with brightest setting as default (255) | [std_msgs/UInt8MultiArray][uint8multiarray]
`set_ascii` | Sets the 4 digit LEDs. Accepts 1 to 4 bytes, each representing an ASCII character to be displayed from left to right | [std_msgs/UInt8MultiArray][uint8multiarray]
`dock` | Activates the demo docking behaviour. Robot enters _Passive_ mode meaning the user loses control (See [OI Spec][oi_spec]) | [std_msgs/Empty][empty]
`undock` | Switches robot to _Full_ mode giving control back to the user | [std_msgs/Empty][empty]
`define_song` | Define a song with up to 16 notes. Each note is described by a MIDI note number and a float32 duration in seconds. The longest duration is 255/64 seconds. You can define up to 4 songs (See [OI Spec][oi_spec]) | [ca_msgs/DefineSong][definesong_msg]
`play_song` | Play a predefined song | [ca_msgs/PlaySong][playsong_msg]

## Commanding your Create

You can print output of a publisher topic by sending an echo command to the topic, e.g.:

``` bash
$ rostopic echo /battery/charge
```

You can move the robot around by sending [geometry_msgs/Twist][twist] messages to the topic `cmd_vel`:

``` bash
$ rostopic pub /cmd_vel geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
```

The `geometry_msgs/Twist` values follow:

```
linear.x  (+)     Move forward (m/s)
          (-)     Move backward (m/s)
angular.z (+)     Rotate counter-clockwise (rad/s)
          (-)     Rotate clockwise (rad/s)
```
#### Velocity limits

`-0.5 <= linear.x <= 0.5` and `-4.25 <= angular.z <= 4.25`

### Teleoperation

#### Keyboard

You can use [https://github.com/ros-teleop/teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard) to control the Create 2 via the keyboard:

``` bash
$ cd ~/create_ws/src
$ git clone https://github.com/ros-teleop/teleop_twist_keyboard.git
$ cd ~/create_ws
$ catkin build teleop_twist_keyboard
$ source ~/.bashrc
```

You can run the teleop keyboard with

    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py

or with custom values,

    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.5 _turn:=0.5

#### Joystick

`ca_tools` comes with a launch file for teleoperating Create with a joystick.

``` bash
$ roslaunch ca_tools joy_teleop.launch [joy_config:=xbox360]
```

There exists configuration files for the [Xbox 360 wired controller](https://www.amazon.ca/Microsoft-Xbox-360-Wired-Controller/dp/B003ZSN600) and the [Logitech F710 controller](http://gaming.logitech.com/en-ca/product/f710-wireless-gamepad). You can adapt these files for your preferred joystick configuration.

[libcreate]:  https://github.com/AutonomyLab/libcreate
[oi_spec]:  https://www.adafruit.com/datasheets/create_2_Open_Interface_Spec.pdf
[odometry]:  http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
[empty]:  http://docs.ros.org/api/std_msgs/html/msg/Empty.html
[uint16]:  http://docs.ros.org/api/std_msgs/html/msg/UInt16.html
[int16]:  http://docs.ros.org/api/std_msgs/html/msg/Int16.html
[twist]:  http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[bool]:  http://docs.ros.org/api/std_msgs/html/msg/Bool.html
[uint8multiarray]:  http://docs.ros.org/api/std_msgs/html/msg/UInt8MultiArray.html
[float32]:  http://docs.ros.org/api/std_msgs/html/msg/Float32.html
[ca_msgs]:  http://github.com/AutonomyLab/create_autonomy/tree/indigo-devel
[bumper_msg]:  https://github.com/AutonomyLab/create_autonomy/blob/indigo-devel/ca_msgs/msg/Bumper.msg
[mode_msg]:  https://github.com/AutonomyLab/create_autonomy/blob/indigo-devel/ca_msgs/msg/Mode.msg
[chargingstate_msg]:  https://github.com/AutonomyLab/create_autonomy/blob/indigo-devel/ca_msgs/msg/ChargingState.msg
[jointstate_msg]:  http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
[definesong_msg]:  https://github.com/AutonomyLab/create_autonomy/blob/indigo-devel/ca_msgs/msg/DefineSong.msg
[playsong_msg]:  https://github.com/AutonomyLab/create_autonomy/blob/indigo-devel/ca_msgs/msg/PlaySong.msg
