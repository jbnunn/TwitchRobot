# [Twitch Robot](./README.md) > Installing Raspian Buster

This should take less than 30 minutes, but it's possible it may take longer when you update the OS (below) on a slow connection.

## Steps

### Download the OS

Download [Raspbian Buster](https://www.raspberrypi.org/downloads/raspbian/) and flash an SD card. You can use the "Desktop" version if you have a card >8 GB. Boot your Raspberry Pi with the card, then choose the appropriate locale, language, and keyboard settings. Be sure to [enable WiFi and SSH](https://www.raspberrypi.org/documentation/remote-access/ssh/).

### Update the OS

     sudo apt-get upgrade

### On to the next step...

When the upgrades have completed, you're ready to install the Robotic Operating System in [Chapter 2](./Chapter2-ROS.md).