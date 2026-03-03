#!/bin/bash
mkdir -p build
cd build
source /opt/ros/jazzy/setup.bash
cmake ..
make -j$(nproc)
