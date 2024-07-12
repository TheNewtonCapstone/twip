#! /bin/bash
# spawn a docker container running, there is no options to interact with the container 
function run_agent(){
    docker run -it --rm --net=host -v /dev/shm/:/dev/shm  --privileged -v /dev:/dev microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -v6
}

#run micro ros agent container with the option to interact with the container 
function agent(){
    ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/&> /dev/null && pwd )"
    echo $ROOT_DIR
    IMAGE=$1
    docker run -it --rm \
        --net=host \
        -v $ROOT_DIR:/workspace \
        -v /dev/shm/:/dev/shm \
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

// build onnx
function build_onnx(){
    ./onnxruntime/build.sh \
            --use_cuda \
            --cuda_home /usr/local/cuda \
            --cudnn_home /usr/lib/aarch64-linux-gnu \
            --use_tensorrt \
            --tensorrt_home /usr/lib/aarch64-linux-gnu \
            --config RelWithDebInfo \
            --build_shared_lib \
            --build \
            --build_wheel \
            --update \
            --skip_submodule_sync \
            --skip_tests \
            --arm64 \
            --cmake_extra_defines ONNXRUNTIME_VERSION=1.9.1 CMAKE_INSTALL_PREFIX=~/code/ZeroShotRT/onnxruntime/install CUDA_CMAKE_COMPILE=/usr/local/cuda/nvcc

}