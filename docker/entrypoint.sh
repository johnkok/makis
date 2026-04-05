#!/bin/bash
set -e

# Source ROS 2 base
source /opt/ros/humble/setup.bash

# Source workspace overlay if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Set up udev rules for serial devices inside container
if [ -e /dev/ttyACM0 ]; then
    chmod 666 /dev/ttyACM0 2>/dev/null || true
fi
if [ -e /dev/ttyUSB0 ]; then
    chmod 666 /dev/ttyUSB0 2>/dev/null || true
fi
if [ -e /dev/i2c-1 ]; then
    chmod 666 /dev/i2c-1 2>/dev/null || true
fi

exec "$@"
