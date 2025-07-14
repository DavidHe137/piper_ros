#!/bin/bash

docker run -it \
           --rm \
           --gpus all \
           --name piper_ros \
           -e DISPLAY=host.docker.internal:0 \
           -v "$(pwd)/../data:/app/data" \
           piper_ros_sim:humble