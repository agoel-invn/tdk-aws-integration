#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.sh"
source "/opt/aws_cloud/setup.bash"
exec "$@"