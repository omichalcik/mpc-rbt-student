#!/bin/bash
source /opt/ros/jazzy/setup.bash
export LOG_LEVEL=3
./build/sender_node config.json
