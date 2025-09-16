#!/bin/bash

BUILD_TYPE=${1:-*}

case $BUILD_TYPE in
    "mujoco")
        DOCKERFILE="Dockerfile.mujoco"
        TAG="piper_sim:mujoco"
        ;;
    "gazebo")
        DOCKERFILE="Dockerfile"
        TAG="piper_sim:ros2humble"
        ;;
    *)
        echo "Usage: $0 [mujoco|gazebo]"
        echo "  mujoco      - Build for MuJoCo simulation"
        echo "  gazebo      - Build for ROS 2 Humble Gazebo simulation"
        exit 1
        ;;
esac

sudo docker build --no-cache \
                  --tag $TAG \
                  --build-arg USERNAME=$2 \
                  --build-arg PASSWORD=$3 \
                  -f $DOCKERFILE \
                  .