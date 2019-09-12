## Agenda

1. Connect to the Pi Camera (done)
2. Send video to Amazon Rekognition via Kinesis video (done)

(follow https://aws.amazon.com/kinesis/video-streams/raspberry-pi-tutorial/)

## Fixes

Follow the [instructions for Raspbian](https://github.com/awslabs/amazon-kinesis-video-streams-producer-sdk-cpp/blob/master/install-instructions-linux.md#install-steps-for-ubuntu-17x-and-raspbian-stretch-using-apt-get) in the Kinesis Video Streams Producer repo.

### Install kvssink plugin

cd /home/pi/amazon-kinesis-video-streams-producer-sdk-cpp/kinesis-video-native-build
cp libgstkvssink.so downloads/local/lib/

### Setup paths

Add the following to your ~/.bashrc or ~/.zshrc:

    export PATH=/home/pi/amazon-kinesis-video-streams-producer-sdk-cpp/kinesis-video-native-build/downloads/local/bin:$PATH
    export LD_LIBRARY_PATH=/home/pi/amazon-kinesis-video-streams-producer-sdk-cpp/kinesis-video-native-build/downloads/local/lib:$LD_LIBRARY_PATH
    export GST_PLUGIN_PATH=/home/pi/amazon-kinesis-video-streams-producer-sdk-cpp/kinesis-video-native-build/downloads/local/lib:$GST_PLUGIN_PATH

## Stream from USB Camera


Original (7s latency)

    gst-launch-1.0 -v v4l2src do-timestamp=TRUE device=/dev/video0 ! videoconvert ! video/x-raw,format=I420,width=640,height=480,framerate=30/1 ! omxh264enc periodicty-idr=45 inline-header=FALSE ! h264parse ! video/x-h264,stream-format=avc,alignment=au ! kvssink stream-name=TwitchRobotCam

Less verbose, lower quality, but higher latency (10s):

    gst-launch-1.0 v4l2src do-timestamp=TRUE device=/dev/video0 ! videoconvert ! video/x-raw,format=I420,width=320,height=240,framerate=30/1 ! omxh264enc periodicty-idr=45 inline-header=FALSE ! h264parse ! video/x-h264,stream-format=avc,alignment=au ! kvssink stream-name=TwitchRobotCam

### Flipping image

Settings: https://gstreamer.freedesktop.org/documentation/videofilter/videoflip.html?gi-language=c

    gst-launch-1.0 -v v4l2src do-timestamp=TRUE device=/dev/video0 ! videoflip method=rotate-180 ! videoconvert ! video/x-raw,format=I420,width=640,height=480,framerate=30/1 ! omxh264enc periodicty-idr=45 inline-header=FALSE ! h264parse ! video/x-h264,stream-format=avc,alignment=au ! kvssink stream-name=TwitchRobotCam


# Kinesis Video works, let's try to get bounding boxes

https://docs.aws.amazon.com/rekognition/latest/dg/images-displaying-bounding-boxes.html



