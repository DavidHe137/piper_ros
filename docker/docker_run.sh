#!/bin/bash

docker run -it \
           --rm \
           --privileged \
           --net=host \
           --gpus all \
           --name piper_ros \
           -e DISPLAY \
	       -v /tmp/.X11-unix/:/tmp/.X11-unix/:rw \
           -v "$(pwd)/../data:/app/data" \
           -v "$(pwd)/../../piper_sdk:/piper_sdk" \
           piper_ros_sim:humble