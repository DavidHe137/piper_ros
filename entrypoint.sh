#!/bin/bash

set -e
source /opt/ros/humble/setup.bash
source /piper_ros/install/setup.bash
exec "$@"