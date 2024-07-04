#! /bin/bash


function run_mr_agent(){
    docker run -it --rm --net=host -v /dev/shm/:/dev/shm  --privileged -v /dev:/dev microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -v6

}



#run micro ros agent container
function agent(){
    ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/&> /dev/null && pwd )"
    echo $ROOT_DIR
    IMAGE=$1
    docker run -it --rm \
        --net=host \
        -v $ROOT_DIR:/workspace \
        -v /dev/shm/:/dev/shm \
        --privileged \
        --device=/dev \
        --device-cgroup-rule='c *:* rmw' \
        -v /dev:/dev \
        -v6 \
        $IMAGE

}
# run ros container
function ros(){
    ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/&> /dev/null && pwd )"
    echo $ROOT_DIR
    IMAGE=$1

    docker run -it --rm \
        --net=host \
        --user=ros \
        -v $ROOT_DIR:/workspace \
        -v /dev/shm/:/dev/shm \
        --device=/dev \
        --device-cgroup-rule='c *:* rmw' \
        -v /dev:/dev \
        $IMAGE
}
