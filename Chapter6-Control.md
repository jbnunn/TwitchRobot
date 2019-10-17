# [Twitch Robot](./README.md) > Controlling the Robot

These control tests should take 5-10 minutes.

## Steps

### Direct Drive on the Robot

Connect the serial cable that came with your Create 2 to a USB port on your Raspberry pi. Let's SSH in first to make sure it responds to commands directly on the robot:

    $ ssh pi@192.168.X.Y
    $ cd ~/TwitchRobot/create_ws
    $ source devel/setup.bash
    $ roslaunch ca_driver create_2.launch

If connected to the robot, you should see something similar to:

    ... logging to /home/pi/.ros/log/53d36a08-d29c-11e9-af1c-d3206ca6194e/roslaunch-raspberrypi-27330.log
    Checking log directory for disk usage. This may take awhile.
    Press Ctrl-C to interrupt
    Done checking log file disk usage. Usage is <1GB.

    xacro.py is deprecated; please use xacro instead
    started roslaunch server http://raspberrypi:36719/

    SUMMARY
    ========

    PARAMETERS
    * /ca_driver/base_frame: base_footprint
    * /ca_driver/dev: /dev/ttyUSB0
    * /ca_driver/latch_cmd_duration: 0.2
    * /ca_driver/loop_hz: 10.0
    * /ca_driver/odom_frame: odom
    * /ca_driver/publish_tf: True
    * /ca_driver/robot_model: CREATE_2
    * /robot_description: <?xml version="1....
    * /rosdistro: melodic
    * /rosversion: 1.14.3

    NODES
    /
        ca_driver (ca_driver/ca_driver)
        robot_state_publisher (robot_state_publisher/robot_state_publisher)

    auto-starting new master
    process[master]: started with pid [27343]
    ROS_MASTER_URI=http://localhost:11311

In another terminal, SSH into your Pi, and issue the following command. If the robot lurches forward, you've succesfully setup the robot code!

    $ rostopic pub /cmd_vel geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
    
### Controlling the Robot via Voice

Run the launch file:

    $ roslaunch alexa twitch_robot.launch

Upon launching, you should see something simliar to the following:

    $ [ INFO] ... [CREATE] Ready.

This launches the Create Autonomy pacakge and ASK listener. The ASK listener listens on an MQTT topic (`/voice/drive`)for commands. The Create Autonomy package sets up the following:

#### Parameters

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

#### Publishers

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

#### Subscribers

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

### Commanding your Create

You can print output of a publisher topic by sending an echo command to the topic, e.g.:

    $ rostopic echo /battery/charge

You can move the robot around by sending [geometry_msgs/Twist][twist] messages to the topic `cmd_vel`:

    $ rostopic pub /cmd_vel geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

The `geometry_msgs/Twist` values follow:

    linear.x  (+)     Move forward (m/s)
              (-)     Move backward (m/s)
    angular.z (+)     Rotate counter-clockwise (rad/s)
              (-)     Rotate clockwise (rad/s)

#### Velocity limits

    -0.5 <= linear.x <= 0.5` and `-4.25 <= angular.z <= 4.25

### Wrap with a test from your Echo device

Now the final test. Invoke your ASK Skill using the invocation phrase you previously setup when your created the skill, e.g., "Alexa, open Twitch Robot". Then, give it a command like "spin around." Your Create 2 should start spinning around.

### On to the next step...

You're now ready for the next chapter, [Person and Object Detection](./Chapter7-Person-Object-Detection.md).