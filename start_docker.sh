#!/bin/sh
uid=$(eval "id -u")
gid=$(eval "id -g")
docker build --build-arg UID="$uid" --build-arg GID="$gid" --build-arg DOMAIN_ID=0 -t iras/r2e .

SRC_CONTAINER=/home/robot/ros_ws/src
SRC_HOST="$(pwd)"/src

docker run \
    -it \
    --name r2e \
    --privileged \
    --rm \
    --net host \
    --ipc host \
    -e DISPLAY=$DISPLAY \
    --volume=$SRC_HOST:$SRC_CONTAINER:rw \
    --volume=/dev:/dev \
    iras/r2e

