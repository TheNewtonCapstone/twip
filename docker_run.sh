#! /bin/bash
ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/../ &> /dev/null && pwd )"
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
