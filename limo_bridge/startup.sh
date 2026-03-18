#!/bin/bash

#######################################################
# LIMO Startup Scrip
#
# The purpose of this script is to startup the robot
# for EK505 students. The script performs the following
# high level actions.
#
# 1. Ensure the system is connected to the wifi
# 2. Startup the LIMO sensor publishing
# 3. Run the ROS1-2 bridge
#######################################################
## Settings
RASTIC_WIFI_SSID=rastic
RASTIC_WIFI_PSWD=

## Wifi Connect
nmcli device wifi connect $RASTIC_WIFI_SSID password $RASTIC_WIFI_PSWD

## Start ROS1 sensor publishing
roscore &

# Test for ROS core being upp

roslaunch limo_bringup limo_start.launch &
roslaunch astra_camera dabai_u3.launch &

## Start ROS1-2 Bridge
docker-compose up
