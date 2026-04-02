#!/bin/bash

set -e
source /opt/ros/humble/setup.bash
export PYTHONPATH=/piper_sdk:$PYTHONPATH
cd /piper_ros && rm -rf build install log && colcon build
chmod -R 777 /piper_ros/build /piper_ros/install /piper_ros/log
cd -
source /piper_ros/install/setup.bash
cd -
exec "$@"
