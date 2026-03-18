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

## Helper Functions
# Stops the current running tasks
cleanup() {
    echo "Main process exiting. Killing background tasks..."
    # Kill all processes in the current process group except the shell itself
    trap - SIGTERM # Prevent infinite loops
    kill 0
}

# Handles running a script until it is successful
retry_with_timeout() {
    local timeout_secs="$1"
    shift # Remove the timeout from the argument list, leaving only the command
    local cmd="$*"

    # Run in a subshell to isolate signals
    (
        # Ignore SIGTERM/EXIT so they don't bubble up
        trap '' EXIT SIGTERM

        local end_time=$((SECONDS + timeout_secs))

        until $cmd; do
            if [ $SECONDS -ge $end_time ]; then
                echo "Error: Command '$cmd' timed out after ${timeout_secs}s."
                exit 1
            fi
            echo "Command failed. Retrying in 1s... (Remaining: $((end_time - SECONDS))s)"
            sleep 1
        done

        echo "Command succeeded within timeout!"
    )
}

trap cleanup EXIT SIGINT SIGTERM

## Wifi Connect
#nmcli device wifi connect $RASTIC_WIFI_SSID password $RASTIC_WIFI_PSWD

## Start ROS1 sensor publishing
roscore &
retry_with_timeout 30 rostopic list

# Test for ROS core being upp

roslaunch limo_bringup limo_start.launch &
# roslaunch astra_camera dabai_u3.launch &

## Start ROS1-2 Bridge
#docker-compose up
