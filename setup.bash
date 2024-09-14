#! /bin/bash


function run_mr_agent(){
    docker run -it --rm --net=host -v /dev/shm/:/dev/shm  --privileged -v /dev:/dev microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -v6

}

function run_onnx(){
docker run -it --net=host \
    -e DISPLAY=$DISPLAY \
    -v /dev:/dev \
    --device-cgroup-rule='c *:* rmw' \
    --device=/dev \
    -v /home/nyquist/workspace/twip:/~ \
    twip:l4t-onnx-ros-r36.3.0 \
    bin/bash
}
