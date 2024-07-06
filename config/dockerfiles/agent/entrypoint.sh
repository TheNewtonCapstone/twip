#!/bin/bash

#set -e

source "/opt/ros/humble/setup.bash"
source "/uros_ws/install/setup.bash"


if [ "$MICROROS_DISABLE_SHM" = "1" ] ; then
    if [ "$ROS_LOCALHOST_ONLY" = "1" ] ; then
        export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_fastdds_shm_localhost_only.xml
    else
        export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_fastdds_shm.xml
    fi
fi

DEV=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0

while true; do
    while [ ! -e "$DEV" ]; do
        sleep 1
        echo "Waiting for device"
    done

    echo "Serial detected. Running agent."

    ros2 run micro_ros_agent micro_ros_agent serial --dev $DEV -v6 &
    export APP_PID=$!

    while [ -e "$DEV" ]; do
        sleep 1
    done

    echo "Serial disconnected. Running agent."

    kill -9 $APP_PID
done
#exec ros2 run micro_ros_agent micro_ros_agent "$@"
function agent(){
    #TODO : MODIFY TO MAP TO YOUR DEVICE

}
