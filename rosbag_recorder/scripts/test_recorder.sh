#!/bin/bash
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Test script for image_bag_recorder

set -e

echo "=== ROSbag Image Recorder Test Script ==="
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS2 is not sourced. Please run:"
    echo "  source /opt/ros/jazzy/setup.bash"
    exit 1
fi

# Test output directory
TEST_OUTPUT="/tmp/rosbag_test_$(date +%Y%m%d_%H%M%S)"

echo "Test output directory: $TEST_OUTPUT"
echo ""

# Function to call service
call_service() {
    local command=$1
    local topics=$2
    local uri=$3

    if [ -n "$topics" ] && [ -n "$uri" ]; then
        ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
            "{command: $command, topics: [$topics], uri: '$uri'}"
    elif [ -n "$topics" ]; then
        ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
            "{command: $command, topics: [$topics]}"
    elif [ -n "$uri" ]; then
        ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
            "{command: $command, uri: '$uri'}"
    else
        ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
            "{command: $command}"
    fi
}

# Check if node is running
echo "1. Checking if recorder node is running..."
if ! ros2 node list | grep -q "image_bag_recorder"; then
    echo "ERROR: image_bag_recorder node is not running!"
    echo "Please start it with:"
    echo "  ros2 run rosbag_recorder image_bag_recorder"
    echo "or"
    echo "  ros2 launch rosbag_recorder image_bag_recorder.launch.py"
    exit 1
fi
echo "   ✓ Node is running"
echo ""

# PREPARE
echo "2. Preparing recorder..."
call_service 0 "'/camera/image_raw', '/robot/joint_states'"
sleep 1
echo "   ✓ Recorder prepared"
echo ""

# Check if test image publisher is available
echo "3. Checking for test image publisher..."
if ! ros2 topic list | grep -q "/camera/image_raw"; then
    echo "   WARNING: No /camera/image_raw topic found"
    echo "   You may want to start a test publisher:"
    echo "     ros2 run image_tools cam2image --ros-args -r image:=/camera/image_raw"
    echo ""
    read -p "   Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "   Aborting test"
        exit 1
    fi
else
    echo "   ✓ Image topic found"
fi
echo ""

# START
echo "4. Starting recording..."
call_service 1 "" "$TEST_OUTPUT"
sleep 1
echo "   ✓ Recording started"
echo ""

# Record for a few seconds
echo "5. Recording for 5 seconds..."
for i in {5..1}; do
    echo "   $i..."
    sleep 1
done
echo ""

# STOP
echo "6. Stopping recording..."
call_service 2
sleep 1
echo "   ✓ Recording stopped"
echo ""

# Verify output
echo "7. Verifying output..."
if [ -d "$TEST_OUTPUT" ]; then
    echo "   ✓ Output directory exists"
    echo ""
    echo "   Contents:"
    ls -lh "$TEST_OUTPUT"
    echo ""

    if [ -d "$TEST_OUTPUT/videos" ]; then
        echo "   Video files:"
        ls -lh "$TEST_OUTPUT/videos"
        echo ""
    else
        echo "   WARNING: No videos directory found"
        echo "   (This is normal if no image topics were published)"
        echo ""
    fi

    # Check MCAP file
    if ls "$TEST_OUTPUT"/*.db3 1> /dev/null 2>&1; then
        echo "   ✓ MCAP file found"

        # Try to get info
        echo ""
        echo "   ROSbag info:"
        ros2 bag info "$TEST_OUTPUT" || echo "   (Could not read bag info)"
    else
        echo "   WARNING: No MCAP file found"
    fi
else
    echo "   ERROR: Output directory not found!"
    exit 1
fi

echo ""
echo "=== Test Complete ==="
echo ""
echo "Output saved to: $TEST_OUTPUT"
echo ""
echo "To clean up:"
echo "  rm -rf $TEST_OUTPUT"
echo ""
