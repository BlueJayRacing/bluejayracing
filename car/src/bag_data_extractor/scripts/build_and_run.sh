#!/bin/bash

# Source ROS 2
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd ~/ros2_ws

# Build the package
colcon build --packages-select bag_data_extractor

# Source the workspace
source install/setup.bash

# Run the node
ros2 run bag_data_extractor bag_extractor