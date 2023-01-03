#!/bin/sh
. /opt/ros/humble/setup.sh
BUILD_TYPE=release
colcon build --mixin $BUILD_TYPE compile-commands ccache
