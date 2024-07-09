#!/bin/bash

set -e

echo "source /opt/ros/humble/install/setup.bash" >> ~/.bashrc
sudo rosdep init
rosdep update


#echo "Provided arguments: $@"

#exec $@
