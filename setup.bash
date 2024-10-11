#! /bin/bash

work_dir=$(pwd)
echo "working dir is ${work_dir}"
function run_mr_agent(){
    docker run -it --rm --net=host -v /dev/shm/:/dev/shm  --privileged -v /dev:/dev microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -v6
}

function run_onnx(){
    docker run -it --net=host \
        -e DISPLAY=$DISPLAY \
        -v /dev:/dev \
        --device-cgroup-rule='c *:* rmw' \
        --device=/dev \
        -v /${work_dir}:/${work_dir}\
        twip:l4t-onnx-ros-r36.3.0 \
    bin/bash
}
