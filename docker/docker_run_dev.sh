#!/bin/bash

docker run -it \
           --rm \
           --privileged \
           --net=host \
           --runtime=nvidia \
           --gpus all \
           --name piper_env \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -e DISPLAY=$DISPLAY \
	       -v /tmp/.X11-unix/:/tmp/.X11-unix/:rw \
           -v "$(pwd)/../data:/app/data" \
           -v /dev:/dev \
           piper_ros_sim:humble
