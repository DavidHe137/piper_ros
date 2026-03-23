#!/bin/bash

set -e
source /opt/ros/humble/setup.bash
export PYTHONPATH=/piper_sdk:$PYTHONPATH
cd /piper_ros && rm -rf build install log && colcon build
cd -
source /piper_ros/install/setup.bash
exec "$@"