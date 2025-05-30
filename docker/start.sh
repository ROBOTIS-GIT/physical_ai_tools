#!/bin/bash

# Source ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Run the web video server in background
ros2 run web_video_server web_video_server &

# Wait a moment for web_video_server to start
sleep 2

# Run the rosbridge server (this will keep the container running)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
